import time
import argparse
import subprocess
import os
from os.path import join

import numpy as np
import rospy
import rospkg

from gazebo_simulation import GazeboSimulation
from scipy.spatial.transform import Rotation as R

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position


def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="test BARN navigation challenge")
    parser.add_argument("--world_idx", type=int, default=0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--out_group", type=int, default=0)
    args = parser.parse_args()

    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################

    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"

    world_name = "BARN/world_%d.world" % (args.world_idx)
    print(
        ">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<"
        % (world_name)
    )
    rospack = rospkg.RosPack()
    base_path = rospack.get_path("jackal_helper")

    launch_file = join(base_path, "launch", "gazebo_launch.launch")
    world_name = join(base_path, "worlds", world_name)

    gazebo_process = subprocess.Popen(
        [
            "roslaunch",
            launch_file,
            "world_name:=" + world_name,
            "gui:=" + ("true" if args.gui else "false"),
        ]
    )
    time.sleep(5)  # sleep to wait until the gazebo being created

    rospy.init_node("gym", anonymous=True)  # , log_level=rospy.FATAL)
    rospy.set_param("/use_sim_time", True)

    # GazeboSimulation provides useful interface to communicate with gazebo
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)

    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    goal_coor = (
        INIT_POSITION[0] + GOAL_POSITION[0],
        INIT_POSITION[1] + GOAL_POSITION[1],
    )

    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)
    collided = True

    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset()  # Reset to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        time.sleep(1)


    if args.gui:
        rviz_base_path = rospack.get_path("jackal_viz")
        rviz_launch_file = join(rviz_base_path, "launch/view_robot.launch")
        rviz_process = subprocess.Popen(
            ["roslaunch", rviz_launch_file, "config:=lflh_navigation"]
        )

    ##########################################################################################
    ## 1. Launch your navigation stack
    ## (Customize this block to add your own navigation stack)
    ##########################################################################################

    launch_file = join(base_path, "../../", "lflh/launch/barn_icra_2023.launch")
    print(f"LfLH launch files: {launch_file}")
    nav_stack_process = subprocess.Popen(
        [
            "roslaunch",
            launch_file,
            "world_idx:=" + str(args.world_idx),
        ]
    )

    # Make sure your navigation stack recives a goal of (0, 10, 0), which is 10 meters away
    # along postive y-axis.
    import actionlib
    from geometry_msgs.msg import Quaternion
    from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

    nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = "odom"
    mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
    mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
    mb_goal.target_pose.pose.position.z = 0
    mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    nav_as.wait_for_server()
    nav_as.send_goal(mb_goal)

    ##########################################################################################
    ## 2. Start navigation
    ##########################################################################################

    curr_time = rospy.get_time()
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)

    # check whether the robot started to move
    while compute_distance(init_coor, curr_coor) < 0.1:
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        time.sleep(0.01)
    

    # start navigation, check position, time and collision
    start_time = curr_time
    start_time_cpu = time.time()
    collided = False

    while (
        compute_distance(goal_coor, curr_coor) > 1
        and not collided
        and curr_time - start_time < 100
    ):
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        ori = gazebo_sim.get_model_state().pose.orientation
        curr_coor = (pos.x, pos.y)
        print(
            "Time: %.2f (s), x: %.2f (m), y: %.2f (m)"
            % (curr_time - start_time, *curr_coor),
            end="\r",
        )
        collided = gazebo_sim.get_hard_collision()
        while rospy.get_time() - curr_time < 0.1:
            time.sleep(0.01)

    ##########################################################################################
    ## 3. Report metrics and generate log
    ##########################################################################################

    print(">>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
    success = False
    if collided:
        status = "collided"
    elif curr_time - start_time >= 100:
        status = "timeout"
    else:
        status = "succeeded"
        success = True
    print("Navigation %s with time %.4f (s)" % (status, curr_time - start_time))

    path_file_name = join(
        base_path, "worlds/BARN/path_files", "path_%d.npy" % args.world_idx
    )
    path_array = np.load(path_file_name)
    path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
    path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
    path_array = np.insert(
        path_array,
        len(path_array),
        (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]),
        axis=0,
    )
    path_length = 0
    for p1, p2 in zip(path_array[:-1], path_array[1:]):
        path_length += compute_distance(p1, p2)

    # Navigation metric: 1_success *  optimal_time / clip(actual_time, 4 * optimal_time, 8 * optimal_time)
    optimal_time = path_length / 2
    actual_time = curr_time - start_time
    nav_metric = (
        int(success)
        * optimal_time
        / np.clip(actual_time, 4 * optimal_time, 8 * optimal_time)
    )
    print("Navigation metric: %.4f" % (nav_metric))

    out_file_name = f"out_{args.out_group}.txt" if args.out_group else "out.txt"
    output_file = join(out_file_name)
    with open(output_file, "a") as f:
        f.write(
            "%d %d %d %d %.4f %.4f\n"
            % (
                args.world_idx,
                success,
                collided,
                (curr_time - start_time) >= 100,
                curr_time - start_time,
                nav_metric,
            )
        )

    output_log = join("log.txt")
    if status != "succeeded":
        _, _, yaw = R.from_quat([ori.x, ori.y, ori.z, ori.w]).as_euler("xyz")

        with open(output_log, "a") as f:
            f.write(f"{args.world_idx} {actual_time:.2f} {pos.x:.2f} {pos.y:.2f} {yaw:.2f}\n")


    gazebo_process.terminate()
    gazebo_process.wait()
    nav_stack_process.terminate()
    nav_stack_process.wait()
    if args.gui:
        rviz_process.terminate()
        rviz_process.wait()
