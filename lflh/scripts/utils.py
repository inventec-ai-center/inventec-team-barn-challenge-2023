#!/usr/bin/python3
import cv2
import numpy as np
import matplotlib.path as mplPath


def check_point_in_polygon(polygon_points, lidar_points):
    polygon = mplPath.Path(polygon_points)
    is_collide = polygon.contains_points(lidar_points)

    return np.any(is_collide)


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


def norm_error_theta(error_th):
    if error_th < -np.pi:
        return error_th % np.pi
    elif error_th > np.pi:
        return -np.pi + (error_th % np.pi)

    return error_th


def resample_lidar(points, n=720):
    N = len(points)
    inds = (np.arange(n) / n) * N
    resampled = []
    for ind in inds:
        lower_i = int(ind)
        upper_i = int(ind) + 1
        ratio = ind % 1
        new_sample = (1 - ratio) * points[lower_i] + ratio * points[upper_i]
        resampled.append(new_sample)

    return np.array(resampled)


def robot_polygon(x, y, pad_x=0.0, pad_y=0.0):
    return np.array(
        [
            [-x - pad_x, +y + pad_y, 1],
            [+x + pad_x, +y + pad_y, 1],
            [+x + pad_x, -y - pad_y, 1],
            [-x - pad_x, -y - pad_y, 1],
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


def inv_transform(points, X, Y, PSI):
    inverse_transform_matrix = np.linalg.inv(transform_matrix(x=X, y=Y, angle=PSI))
    xyz = np.concatenate([points, np.ones_like(points[:, :1])], axis=-1)
    norm_points = np.matmul(inverse_transform_matrix, xyz.T)

    return np.asarray(norm_points[:2, :]).T  # X, Y


def transform_polygon(polygon, X, Y, PSI):
    transformed_polygon = transform_matrix(x=X, y=Y, angle=PSI).dot(polygon.T)

    return np.asarray(transformed_polygon[:2, :].T)
