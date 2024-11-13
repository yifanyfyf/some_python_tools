import numpy as np


def local_to_global(points_local, origin_local, theta):
	"""
	将多个二维平面坐标系中的局部坐标点批量转换为全局坐标。

	参数：
	- points_local: 局部坐标中的多个点，格式为 Nx2 的数组。
	- origin_local: 局部坐标系的原点在全局坐标系中的位置，格式为 (x_local, y_local)。
	- theta: 局部坐标系相对于全局坐标系的旋转角度（以弧度为单位）。

	返回值：
	- 全局坐标中的多个点，格式为 Nx2 的数组。
	"""
	# 构建旋转矩阵
	rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
								[np.sin(theta), np.cos(theta)]])

	# 旋转并平移所有点
	points_global = rotation_matrix @ points_local.T
	points_global = points_global.T
	points_global += origin_local

	return points_global


def global_to_local(points_global, origin_local, theta):
	"""
	将一系列全局坐标转换为局部坐标。

	参数：
	- points_global: 全局坐标中的多个点，格式为 Nx2 的数组。
	- origin_local: 局部坐标系的原点在全局坐标系中的位置，格式为 (x_local, y_local)。
	- theta: 局部坐标系相对于全局坐标系的旋转角度（以弧度为单位）。

	返回值：
	- 局部坐标中的多个点，格式为 Nx2 的数组。
	"""
	# 将全局坐标系中的点平移到局部坐标系的原点
	translated_points = points_global - origin_local

	# 构建旋转矩阵
	rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
								[np.sin(theta), np.cos(theta)]])

	# 旋转所有点以完成坐标转换
	points_local = np.linalg.inv(rotation_matrix) @ translated_points.T

	return points_local.T


if __name__ == "__main__":
	# 定义全局坐标中的多个点
	# points = np.array([[6, 6], [7, 8], [9, 10]])
	points = np.array([[-1., 11.],
	 [-3., 12.],
	 [-5., 14.]])

	# 定义局部坐标系的原点和角度
	origin_local = (5, 5)
	theta = np.pi / 180 * 90

	# 转换为局部坐标
	points_local = global_to_local(points, origin_local, theta)
	print("全局坐标转换为局部坐标:\n", points_local)
