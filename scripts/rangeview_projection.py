import numpy as np
import matplotlib.pyplot as plt
import math


def rv_project(points, fov_up=10*np.pi/180, fov_down=-30*np.pi/180, H=32, W=1024):
	fov = abs(fov_down) + abs(fov_up)  # get field of view total in rad
	# get depth of all points
	depth = np.linalg.norm(points[:, :3], 2, axis=1)

	scan_x = points[:, 0]
	scan_y = points[:, 1]
	scan_z = points[:, 2]
	intensity = points[:, 3]
	labels = points[:, 4]

	yaw = -np.arctan2(scan_y, scan_x)
	pitch = np.arcsin(scan_z / depth)
	r = np.linalg.norm(points[:, :3], axis=1)
	
	yaw = (yaw + np.pi) / (2 * np.pi)
	pitch = (fov_up - pitch) / fov
	
	yaw *= W
	pitch *= H
	
	yaw = np.floor(yaw).astype(int)
	pitch = np.floor(pitch).astype(int)

	pitch[pitch > 31] = 31

	# canvas is for visualization. to see the category of each pixel in projection
	canvas = np.zeros((H, W))
	canvas[pitch, yaw] = labels
	# canvas[pitch, yaw] = r
	
	# proj = np.zeros([H, W, 3])
	# proj[pitch, yaw] = points[:, :3]

	proj = np.zeros([H, W, 2])
	proj[pitch, yaw, 0] = r
	proj[pitch, yaw, 1] = intensity
	
	# canvas = canvas[:, 384:641]
	# proj = proj[:, 384:641]
	
	# plt.imshow(proj[:, :, 0])
	# plt.show()
	
	# # 计算每一列的均值
	# # 计算每个二维数组的均值
	# mean_values = np.mean(proj, axis=(0, 1))
	# std_dev_values = np.std(proj, axis=(0, 1))
	#
	# mean_std = [mean_values[0], mean_values[1], mean_values[2],
	#             std_dev_values[0], std_dev_values[1], std_dev_values[2]]
	
	return proj
