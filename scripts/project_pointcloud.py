import numpy as np


def project_pc2img(pc_array):
    image_w = 1920.0
    image_h = 1280.0
    K = np.identity(3)
    K[0, 0] = 1192.71
    K[1, 1] = 1174.01
    K[0, 2] = image_w / 2.0
    K[1, 2] = image_h / 2.0

    # 雷达到相机的外参转换
    T = np.array([[-0.0120068, -0.999915, -0.00512968, -0.0142018],
                       [-0.0503408, 0.00572801, -0.998716, 0.0788452],
                       [0.99866, -0.0117332, -0.0504053, 0.00911619],
                       [0, 0, 0, 1]])

    pc_array = np.hstack((pc_array, np.ones((pc_array.shape[0], 1))))
    pc_array = pc_array.T
    point_in_camera_coords_slam = T @ pc_array
    point_in_camera_coords_slam = point_in_camera_coords_slam.T
    point_in_camera_coords_slam = point_in_camera_coords_slam[:, :3]

    points_2d = np.dot(K, point_in_camera_coords_slam.T)
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :]])
    points_2d = points_2d.T

    points_in_canvas_mask = \
        (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) & \
        (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) & \
        (points_2d[:, 2] > 0.0)

    points_2d = points_2d[points_in_canvas_mask]
    u_coord = points_2d[:, 0].astype(int)
    v_coord = points_2d[:, 1].astype(int)
    z = points_2d[:, 2]

    points_2d[:, 0] *= points_2d[:, 2]
    points_2d[:, 1] *= points_2d[:, 2]

    return u_coord, v_coord, z