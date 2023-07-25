#!/usr/bin/python3
import os
import rospkg
import rospy
import sys
import numpy as np
import utils
import torch

from scipy import signal
from enum import IntEnum, auto
from lflh_utils import Params, LfDModel

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import LaserScan

import actionlib
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


class States(IntEnum):
    LfLH = auto()
    HardTurn = auto()
    BackTrack = auto()
    Forward = auto()
    SendGoal = auto()
    Stop = auto()


class LfLHController:
    def __init__(self, world_idx):
        self.world_idx = world_idx

        self._init_dir()
        self._init_model()
        self._init_params()
        self._init_pubs_and_subs()

        rospy.sleep(0.25)
        assert self._lidar_dim, f"Lidar dims: `{self._lidar_dim}`"

        self._logging_file = f"{self._repo_path}/barn_result/world_{self.world_idx}.txt"
        if self._log:
            print(f"logging: {self._logging_file}")
            with open(self._logging_file, "a") as f:
                f.write("\n")
                f.write("----------------------------\n")

        rospy.loginfo("LfLH Controller node ready...")

    #########################################
    ## Initialization
    #########################################
    def _init_dir(self):
        self._rospack = rospkg.RosPack()
        self._repo_path = self._rospack.get_path("lflh")
        self._config_path = os.path.join(self._repo_path, "configs", "model")
        self._params_path = os.path.join(self._config_path, "lflh_params.json")
        self._params = Params(self._params_path)

    def _init_model(self):
        self._model_params = self._params.model
        self._device = torch.device(self._model_params.device)
        self._model = LfDModel(self._model_params).to(self._device)

        self._weight_path = os.path.join(self._config_path, self._model_params.weight)
        assert os.path.exists(self._weight_path)
        self._model.load_state_dict(
            torch.load(self._weight_path, map_location=self._device)
        )

    def _init_params(self):
        self._log = self._params.log
        self._main_rate = self._params.rate.main
        self._update_dt = self._params.rate.update_dt

        self._local_start_dist = self._params.local.start_dist
        self._local_dir_dist = self._params.local.dir_dist
        self._local_goal_dist = self._params.local.goal_dist

        # Parameters related to safety check.
        self._safety_dt = self._params.safety.dt
        self._safety_step = self._params.safety.step

        self._init_lidar_params()
        self._init_robot_boundaries()
        self._init_costmap_params()
        self._init_variables()

        # Parameters for our recovery behavior, backtracking.
        self._backtrack_align_angle = np.radians(self._params.backtrack.align_deg)
        self._backtrack_target_dist_m = self._params.backtrack.target_dist_m
        self._backtrack_l2_dist_m = self._params.backtrack.l2_dist_m
        self._backtrack_rev_vel = self._params.backtrack.rev_vel
        self._backtrack_timeout = self._params.backtrack.timeout

        # Parameters related to hard turn.
        self._turn_max_angle = np.radians(self._params.hard_turn.max_deg)
        self._turn_stop_angle = np.radians(self._params.hard_turn.stop_deg)
        self._turn_p_gain = self._params.hard_turn.p_gain

        # Speed params.
        self._rotate_align_min_w = self._params.speed.rotate_align_min_w
        self._linear_v_gain = self._params.speed.linear_v_gain
        self._rotate_w_gain = self._params.speed.rotate_w_gain
        self._front_pad_v = self._params.speed.front_pad_v
        self._limit_v = self._params.speed.limit_v
        self._acc_v = self._params.speed.acc_v
        self._max_v = self._params.speed.max_v
        self._max_w = self._params.speed.max_w
        self._stuck_v = self._params.speed.stuck_v
        self._stuck_w = self._params.speed.stuck_w

    def _init_costmap_params(self):
        self._costmap_resolution_m = rospy.get_param(
            "/move_base/local_costmap/resolution"
        )
        costmap_width_m = rospy.get_param("/move_base/local_costmap/width")
        costmap_height_m = rospy.get_param("/move_base/local_costmap/height")
        self._costmap_width_pixel = int(costmap_width_m / self._costmap_resolution_m)
        self._costmap_height_pixel = int(costmap_height_m / self._costmap_resolution_m)
        self._costmap_x_center = self._costmap_width_pixel // 2
        self._costmap_y_center = self._costmap_height_pixel // 2

        self._costmap_img = np.zeros(
            (self._costmap_height_pixel, self._costmap_width_pixel, 3), dtype=np.uint8
        )

        self._robot_w_pixel = int(self._robot_width_m / self._costmap_resolution_m)
        self._robot_h_pixel = int(self._robot_length_m / self._costmap_resolution_m)

        self._rear_len_m = self._params.robot.rear_length_m
        self._rear_len_pixel = self._rear_len_m / self._costmap_resolution_m
        self._costmap_max_val = self._params.costmap.max_val
        self._costmap_rear_weight = self._params.costmap.rear_weight
        self._costmap_points_m = np.zeros(
            [self._costmap_height_pixel, self._costmap_width_pixel]
        )

    def _init_lidar_params(self):
        self._lidar_offset_y_m = self._params.lidar.offset_y_m
        lidar_coll_max_m = self._params.lidar.coll_max_m
        lidar_coll_min_m = self._params.lidar.coll_min_m
        lidar_coll_offset_m = self._params.lidar.coll_offset_m

        self._lidar_dim = self._params.lidar.dimension
        lidar_angle_min = self._params.lidar.angle_min
        lidar_angle_max = self._params.lidar.angle_max
        self._lidar_max_range = self._params.lidar.max_range
        self._lidar_angles = np.linspace(
            lidar_angle_max, lidar_angle_min, self._lidar_dim
        )
        self._lidar_front_idx = np.where(
            np.abs(self._lidar_angles) <= np.radians(self._params.lidar.front_angle)
        )
        self._lidar_front_dist_clear_m = self._params.lidar.front_dist_clear_m

        lidar_threshold = np.linspace(
            lidar_coll_max_m + lidar_coll_offset_m,
            lidar_coll_min_m + lidar_coll_offset_m,
            self._lidar_dim // 2,
        )
        self._lidar_threshold = np.hstack([lidar_threshold, lidar_threshold[::-1]])

    def _init_robot_boundaries(self):
        self._robot_width_m = self._params.robot.width_m
        self._robot_length_m = self._params.robot.length_m
        self._robot_half_width_m = self._robot_width_m / 2
        self._robot_half_length_m = self._robot_length_m / 2

        # Robot polygon for rotate collision usage.
        self._robot_polygon = utils.robot_polygon(
            x=self._robot_half_width_m, y=self._robot_half_length_m
        )

    def _init_variables(self):
        self._X, self._Y, self._PSI = 0.0, 0.0, 0.0
        self._local_goal = None
        self._local_path_dir = 0.0
        self._lidar_points = None
        self._lidar_clipped_ranges = None

        self._state = None
        self._hist_point = []
        self._is_collide = False

    def _init_pubs_and_subs(self):
        # Subsriber
        odom_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, callback=self._cb_robot_odom, queue_size=1
        )
        global_path_sub = rospy.Subscriber(
            "/move_base/NavfnROS/plan",
            Path,
            callback=self._cb_global_path,
            queue_size=1,
        )
        lidar_sub = rospy.Subscriber(
            "/front/scan", LaserScan, callback=self._cb_lidar, queue_size=1
        )
        costmap_sub = rospy.Subscriber(
            "/move_base/local_costmap/costmap",
            OccupancyGrid,
            callback=self._cb_costmap,
            queue_size=1,
        )
        costmap_updates_sub = rospy.Subscriber(
            "/move_base/local_costmap/costmap_updates",
            OccupancyGridUpdate,
            callback=self._cb_costmap_updates,
            queue_size=1,
        )

        # Publishers.
        self._velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Services.
        self._clear_costmap_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

    #########################################
    ## ROS Subscriber Callback
    #########################################
    def _cb_robot_odom(self, msg):
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q0 = msg.pose.pose.orientation.w
        self._X = msg.pose.pose.position.x
        self._Y = msg.pose.pose.position.y
        self._PSI = np.arctan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2**2 + q3**2)))

        if self._state == States.LfLH:
            self._hist_point.append([self._X, self._Y])

    def _cb_global_path(self, msg):
        gp = []
        for pose in msg.poses:
            gp.append([pose.pose.position.x, pose.pose.position.y])
        gp = np.array(gp)
        if not len(gp):
            return

        x = gp[:, 0]
        try:
            xhat = signal.savgol_filter(x, 19, 3)
        except:
            xhat = x

        y = gp[:, 1]
        try:
            yhat = signal.savgol_filter(y, 19, 3)
        except:
            yhat = y
            
        gphat = np.column_stack((xhat, yhat))
        self._normalized_path = utils.inv_transform(
            points=gphat, X=self._X, Y=self._Y, PSI=self._PSI
        )

        self._local_goal = utils.get_local_goal(
            norm_path=self._normalized_path,
            min_start_dist=self._local_start_dist,
            local_goal_dist=self._local_goal_dist,
        )
        self._local_path_dir = utils.get_local_path_dir(
            norm_path=self._normalized_path,
            min_start_dist=self._local_start_dist,
            min_dist=self._local_dir_dist,
        )

    def _cb_lidar(self, msg):
        self._lidar_raw_ranges = utils.resample_lidar(msg.ranges)
        self._lidar_clipped_ranges = np.minimum(
            self._lidar_raw_ranges, self._lidar_max_range
        ).astype(np.float32)

        lidar_x = self._lidar_raw_ranges * np.sin(self._lidar_angles)
        lidar_y = self._lidar_offset_y_m + self._lidar_raw_ranges * np.cos(
            self._lidar_angles
        )
        self._lidar_points = np.vstack([lidar_x, lidar_y]).T
        self._is_collide = np.any(self._lidar_raw_ranges <= self._lidar_threshold)

    def _cb_costmap(self, msg):
        # only update during linear movement.
        self._update_costmap(msg)

    def _cb_costmap_updates(self, msg):
        # only update not during linear movement.
        self._update_costmap(msg)

    def _update_costmap(self, msg):
        np_costmap = (
            np.array(msg.data, dtype=np.uint8)
            .reshape(self._costmap_height_pixel, self._costmap_width_pixel)
            .T
        )
        self._costmap_img[:, :, 0] = np_costmap
        self._costmap_img[:, :, 1] = np_costmap
        self._costmap_img[:, :, 2] = np_costmap

        np_costmap = np.rot90(np_costmap)
        costmap_idx_obstacle = np.argwhere(np_costmap >= 99)
        costmap_coor_pixel = costmap_idx_obstacle - np.array(
            [self._costmap_y_center, self._costmap_x_center]
        )
        costmap_points_m = costmap_coor_pixel * self._costmap_resolution_m
        self._costmap_points_m = utils.inv_transform(
            points=costmap_points_m, X=0, Y=0, PSI=self._PSI
        )

    ## ROS Publisher
    #########################################
    def _cmd_vel(self, v, w):
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        
        self._logging(f"v: {vel_msg.linear.x:.2f}, w:{vel_msg.angular.z:.2f}")
        self._velocity_pub.publish(vel_msg)

    #########################################
    ## Safety Checks
    #########################################
    def _lflh_safety_check(self, v, w, points):
        X, Y, PSI = 0, 0, 0
        for i in range(self._safety_step):
            X_DOT = v * np.sin(PSI)
            Y_DOT = v * np.cos(PSI)
            PSI_DOT = w
            X = X + X_DOT * self._safety_dt
            Y = Y + Y_DOT * self._safety_dt
            PSI = PSI + PSI_DOT * self._safety_dt

            pred_robot_polygon = utils.transform_polygon(self._robot_polygon, X, Y, PSI)
            is_collide = utils.check_point_in_polygon(
                polygon_points=pred_robot_polygon, lidar_points=points
            )
            if is_collide:
                return True

        return False

    def _check_rear_collision(self):
        (robot_rfront, robot_rrear, robot_lrear, robot_lfront,) = utils.get_footprint(
            center=[self._costmap_x_center, self._costmap_y_center],
            size=[self._robot_w_pixel, self._robot_h_pixel],
            angle=-self._PSI,
        )

        # Calculate rear point area.
        new_rrear = utils.extent_robot_point(
            pt1=robot_rrear,
            pt2=robot_rfront,
            length=self._rear_len_pixel,
        )
        new_lrear = utils.extent_robot_point(
            pt1=robot_lrear,
            pt2=robot_lfront,
            length=self._rear_len_pixel,
        )
        robot_rear_points = np.vstack([new_rrear, new_lrear, robot_lrear, robot_rrear])

        rear_area, rear_area_weight, _ = utils.get_pixel_area(
            costmap_img=self._costmap_img,
            footprint=robot_rear_points,
            padding=1,
            costmap_max_val=self._costmap_max_val,
        )
        return (
            np.any(rear_area >= self._costmap_max_val)
            or rear_area_weight > self._costmap_rear_weight
        )

    #########################################
    ## LfLH
    #########################################
    def _lflh_inference(self):
        if self._lidar_clipped_ranges is None or self._local_goal is None:
            return States.LfLH

        # Run model inference.
        scan = torch.from_numpy(self._lidar_clipped_ranges[None]).to(self._device)
        local_goal = torch.from_numpy(self._local_goal[None]).to(self._device)
        cmd = self._model(scan, local_goal)
        cmd = cmd[0].detach().cpu().numpy()  # remove batch size
        v, w = cmd  # Retrieving linear and angular vel commands.

        boost_v = v > self._acc_v and np.all(
            self._lidar_raw_ranges[self._lidar_front_idx]
            > self._lidar_front_dist_clear_m
        )
        v = v if boost_v else np.minimum(self._limit_v, v)

        # 1. Safety check. Do we think we'll crash with these commands?
        if self._lflh_safety_check(v=v, w=w, points=self._lidar_points):
            self._logging("LfLH about to collide")
            # We're about to collide, go to recovery state.
            self._cmd_vel(v=0.0, w=0.0)
            return States.BackTrack
        # 2. Angle is too large for LfLH, go to manual turn state.
        elif np.abs(self._local_path_dir) > self._turn_max_angle:
            self._cmd_vel(v=0.0, w=0.0)
            return States.HardTurn
        else:
            # 3. Everything should be fine, send command and stay in LfLH state.
            self._cmd_vel(v=v, w=w)
            return States.LfLH

    #########################################
    ## Recovery Behavior
    #########################################
    def _hard_turn(self):
        # Intendend command.
        turn_vel = self._local_path_dir * self._turn_p_gain
        turn_vel = max(self._rotate_align_min_w, abs(turn_vel)) * np.sign(turn_vel)

        # If turning angle is small enough we can give back command to the LfLH.
        if np.abs(self._local_path_dir) < self._turn_stop_angle:
            self._cmd_vel(v=0.0, w=0.0)
            return States.LfLH

        # If footprint is not clear, we go to backtrack.
        if (
            self._lflh_safety_check(v=0.0, w=turn_vel, points=self._costmap_points_m)
            or self._is_collide
        ):
            self._logging(f"!!! Hard turn giving up !!!")
            self._cmd_vel(v=0.0, w=0.0)
            return States.BackTrack

        # Keep turning and stay in HardTurn state.
        self._cmd_vel(v=0.0, w=turn_vel)
        return States.HardTurn

    def _find_point_in_hist(self, target_dist):
        idx = len(self._hist_point) - 1
        cur_dist = 0.0
        err_angle = float("inf")
        cur_point = None

        # iterate history points to find point that more than target_dist and behind the robot.
        while (
            np.abs(cur_dist) < target_dist or np.abs(err_angle) > self._turn_max_angle
        ):
            if idx < 0:
                return [0.0, 0.0]

            cur_point = self._hist_point[idx]
            cur_dist = utils.distance_to_robot_pos(
                target_pos=cur_point, robot_pos=[self._X, self._Y]
            )
            target_angle = np.arctan2(
                self._Y - cur_point[1],
                self._X - cur_point[0],
            )
            err_angle = utils.norm_error_theta(target_angle - self._PSI)
            idx -= 1

        # Discard history later than our target point.
        self._hist_point = self._hist_point[: idx + 1]

        return cur_point

    def _rotate_align(self, target_angle):
        rate = rospy.Rate(self._main_rate)
        err_angle = utils.norm_error_theta(target_angle - self._PSI)

        while np.abs(err_angle) > self._backtrack_align_angle:
            self._logging(f"Rotate align.")
            turn_vel = err_angle * self._turn_p_gain

            if self._lflh_safety_check(
                v=0.0, w=turn_vel, points=self._costmap_points_m
            ):
                # We might crash during align. Stop here and just move back.
                self._logging("### About to hit during rotate align. Giving up. ###")
                self._cmd_vel(v=0.0, w=0.0)
                return False

            turn_vel = max(self._rotate_align_min_w, abs(turn_vel)) * np.sign(turn_vel)
            self._cmd_vel(v=0.0, w=turn_vel)
            rate.sleep()
            err_angle = utils.norm_error_theta(target_angle - self._PSI)

        return True

    def _backtrack_path_history(self):
        rate = rospy.Rate(self._main_rate)
        # Search for the first point in the path farther away than _backtrack_target_dist_m
        goal_point = self._find_point_in_hist(target_dist=self._backtrack_target_dist_m)
        if not goal_point:
            self._logging("`Backtrack`: target point is None")
            return

        # Step 1, rotate to align with goal point.
        target_angle = np.arctan2(
            self._Y - goal_point[1],
            self._X - goal_point[0],
        )
        self._rotate_align(target_angle)

        # Step 2, move backwards
        init_pos = [self._X, self._Y]
        dist2init = utils.distance_to_robot_pos(
            target_pos=init_pos, robot_pos=[self._X, self._Y]
        )
        prev_dist = dist2init

        while dist2init <= self._backtrack_l2_dist_m and prev_dist <= dist2init:
            prev_dist = dist2init
            # Check for costmap in rear of the robot.
            if self._check_rear_collision():
                self._cmd_vel(v=0.0, w=0.0)
                self._logging("### Backward dangerous###")
                return States.Forward

            self._logging("`Backtrack`: Move back.")
            self._cmd_vel(v=self._backtrack_rev_vel, w=0.0)

            rate.sleep()
            dist2init = utils.distance_to_robot_pos(
                target_pos=init_pos, robot_pos=[self._X, self._Y]
            )

        return States.HardTurn

    def _forward_recovery(self):
        ticks = 0
        while ticks * self._update_dt <= self._backtrack_timeout:
            ang_vel = np.sign(self._local_path_dir) * self._stuck_w
            if not self._lflh_safety_check(
                v=self._stuck_v, w=ang_vel, points=self._lidar_points
            ):
                self._logging("### Forward a bit ###")
                self._cmd_vel(v=self._stuck_v, w=ang_vel)
            else:
                self._logging("### Stuck ####")
                self._cmd_vel(v=-self._stuck_v, w=ang_vel)

            ticks += 1
            rospy.sleep(self._update_dt)

        self._cmd_vel(v=0.0, w=0.0)
        return States.HardTurn

    #########################################
    ## Utils
    #########################################
    def _logging(self, state):
        if self._log:
            str_state = f"State: `{state}`."
            print(str_state)
            with open(self._logging_file, "a") as f:
                f.write(str_state + "\n")

    
    def _send_goal(self, x, y):
        nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = "odom"
        mb_goal.target_pose.pose.position.x = x
        mb_goal.target_pose.pose.position.y = y
        mb_goal.target_pose.pose.position.z = 0
        mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        nav_as.wait_for_server()
        nav_as.send_goal(mb_goal)


    #########################################
    ## Run Loop
    #########################################
    def run(self):
        self._state = States.SendGoal
        rate = rospy.Rate(self._main_rate)
        goal_x = 20.0
        goal_y = 0.0

        while not rospy.is_shutdown():
            if self._state == States.SendGoal:
                self._logging("SendGoal")
                self._send_goal(x=goal_x, y=goal_y)
                rospy.sleep(1)
                self._state = States.LfLH

            elif self._state == States.LfLH:
                self._logging("LfLH")
                self._state = self._lflh_inference()

            elif self._state == States.HardTurn:
                self._logging("HardTurn")
                self._state = self._hard_turn()

            elif self._state == States.BackTrack:
                self._logging("Backtrack")
                self._state = self._backtrack_path_history()

            elif self._state == States.Forward:
                self._logging("Forward")
                self._state = self._forward_recovery()

            elif self._state == States.Stop:
                self._logging("Stop")
            
            if utils.distance_to_robot_pos(
                target_pos=[goal_x, goal_y], robot_pos=[self._X, self._Y]
            ) <= 0.1:
                self._state = States.Stop

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lflh", anonymous=True)
    world_idx = sys.argv[1]
    # rospy.loginfo(f"World idx: `{world_idx}`")

    node = LfLHController(world_idx)
    node.run()
