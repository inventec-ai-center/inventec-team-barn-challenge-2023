#!/usr/bin/python3
import cv2
import numpy as np
import matplotlib.path as mplPath


def check_point_in_polygon(polygon_points, lidar_points):
    polygon = mplPath.Path(polygon_points)
    is_collide = polygon.contains_points(lidar_points)
    return np.any(is_collide)


def define_robot_boundary(robot_half_width_m, robot_half_length_m, n_boundary):
    y_top = robot_half_width_m
    y_bottom = -robot_half_width_m

    x_right = robot_half_length_m
    x_left = -robot_half_length_m

    boundary = np.zeros((4 * n_boundary, 2))
    xx = np.linspace(x_left, x_right, n_boundary)
    yy = np.linspace(y_bottom, y_top, n_boundary)

    # order is U, B, L, R
    boundary[:n_boundary][:, 0] = xx
    boundary[n_boundary : 2 * n_boundary][:, 0] = xx
    boundary[2 * n_boundary : 3 * n_boundary][:, 0] = x_left
    boundary[3 * n_boundary :][:, 0] = x_right

    boundary[:n_boundary][:, 1] = y_top
    boundary[n_boundary : 2 * n_boundary][:, 1] = y_bottom
    boundary[2 * n_boundary : 3 * n_boundary][:, 1] = yy
    boundary[3 * n_boundary :][:, 1] = yy

    return boundary


def distance_to_robot_pos(target_pos, robot_pos):
    target_pos = np.array(target_pos)
    robot_pos = np.array(robot_pos)

    return np.linalg.norm(robot_pos - target_pos)


def extent_robot_point(pt1, pt2, length):
    diff_pt = pt1 - pt2
    theta = np.arctan2(diff_pt[1], diff_pt[0])
    pt_x = pt1[0] + (length * np.cos(theta))
    pt_y = pt1[1] + (length * np.sin(theta))

    return np.array([pt_x, pt_y], dtype=np.uint8)


def get_footprint(center, size, angle):
    robot_rect = (
        center,
        size,
        np.degrees(angle),
    )
    footprint_points = cv2.boxPoints(robot_rect)
    robot_box = np.int0(footprint_points)

    return robot_box


def get_local_goal(norm_path, min_start_dist, local_goal_dist):
    local_goal = np.zeros(2)
    start_point = np.zeros(2)

    if len(norm_path) > 0:
        if np.linalg.norm(norm_path[0] - start_point) > min_start_dist:
            start_point = norm_path[0]

        for point in norm_path:
            # TODO: Try with accumulating the distance ?
            dist = np.linalg.norm(point - start_point)
            if dist > local_goal_dist:
                break

        local_goal = point - start_point
        local_goal /= np.linalg.norm(local_goal)

    return local_goal.astype(np.float32)


def get_local_path_dir(norm_path, min_start_dist, min_dist):
    local_path_dir = 0.0
    prev_point = np.zeros(2)
    start_point = np.zeros(2)
    cum_dist = 0.0
    if len(norm_path) > 0:
        if np.linalg.norm(norm_path[0] - prev_point) > min_start_dist:
            prev_point = norm_path[0]
            start_point = norm_path[0]

        for point in norm_path:
            start_dist = np.linalg.norm(point - start_point)
            if start_dist > 0.1:
                local_path_dir += (point - start_point) / start_dist

            cum_dist += np.linalg.norm(point - prev_point)
            prev_point = point
            if cum_dist > min_dist:
                break

        if isinstance(local_path_dir, np.ndarray):
            local_path_dir = np.arctan2(local_path_dir[1], local_path_dir[0])

    return local_path_dir


def get_pixel_area(costmap_img, footprint, padding, costmap_max_val):
    mask_img = np.zeros_like(costmap_img)
    cv2.drawContours(mask_img, [footprint], 0, (255, 255, 255), -1)
    area_img = cv2.bitwise_and(costmap_img, mask_img)
    (area, _, _) = cv2.split(area_img)

    area_roi = 0
    area_weight = 0.0

    if not np.any(area):
        return area_roi, area_weight, area_img

    row_idx, column_idx = np.where(area > 0)
    row_start = np.amin(row_idx) - padding
    row_end = np.amax(row_idx) + padding + 1
    col_start = np.amin(column_idx) - padding
    col_end = np.amax(column_idx) + padding + 1

    area_roi = area[row_start:row_end, col_start:col_end]
    area_sum = np.sum(area_roi)
    area_npixel = np.sum(area_roi > 0)

    if area_sum:
        area_weight = area_sum / (area_npixel * costmap_max_val)

    return area_roi, area_weight, area_img


def get_randomized_angle(cur_angle, align_angle, deviation, num, sign):
    rotate_right_ll = cur_angle - align_angle - deviation
    rotate_right_ul = cur_angle - align_angle
    rotate_right_angles = np.linspace(rotate_right_ll, rotate_right_ul, num)

    rotate_left_ul = cur_angle + align_angle + deviation
    rotate_left_ll = cur_angle + align_angle
    rotate_left_angles = np.linspace(rotate_left_ll, rotate_left_ul, num)

    angle = (
        np.random.choice(rotate_left_angles, 1)[0]
        if sign >= 1.0
        else np.random.choice(rotate_right_angles, 1)[0]
    )

    return norm_error_theta(angle)


def norm_error_theta(error_th):
    if error_th < -np.pi:
        return error_th % np.pi
    elif error_th > np.pi:
        return -np.pi + (error_th % np.pi)

    return error_th


def rectangle_polygon(x, y, pad_x=0.0, pad_yfront=0.0, pad_yrear=0.0):
    return np.array(
        [
            [-x - pad_x, -y - pad_yrear],
            [-x - pad_x, +y + pad_yfront],
            [+x + pad_x, +y + pad_yfront],
            [+x + pad_x, -y - pad_yrear],
            [-x - pad_x, -y - pad_yrear],
        ]
    )


def transform_matrix(x, y, angle):
    cos, sin = np.cos(angle), np.sin(angle)

    return np.matrix(
        [
            [cos, -sin, x],
            [sin, cos, y],
            [0, 0, 1],
        ]
    )


def rotate_points(points, angle):
    rotate_matrix = transform_matrix(x=0, y=0, angle=angle)
    points = np.concatenate([points, np.ones_like(points[:, :1])], axis=-1)
    rotated_points = rotate_matrix.dot(points.T)

    return np.asarray(rotated_points[:2, :]).T


def transform_path(global_path, X, Y, PSI):
    inverse_transform_matrix = np.linalg.inv(transform_matrix(x=X, y=Y, angle=PSI))
    xy_path = np.concatenate([global_path, np.ones_like(global_path[:, :1])], axis=-1)
    norm_path = np.matmul(inverse_transform_matrix, xy_path.T)

    return np.asarray(norm_path[:2, :]).T  # X, Y
    