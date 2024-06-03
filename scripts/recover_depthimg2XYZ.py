def recover3d(self, depth, intrinsic_):
		fx = intrinsic_[0]
		fy = intrinsic_[1]
		cx = intrinsic_[2]
		cy = intrinsic_[3]

		row_indices, col_indices = np.indices(depth.shape)
		depth = np.stack((row_indices.ravel(), col_indices.ravel(), depth.ravel()), axis=-1)
		depth = depth[depth[:, 2] < 50]

		u = depth[:, 1]
		v = depth[:, 0]
		z = depth[:, 2]

		# 计算X, Y, Z
		X = (u - cx) * z / fx
		Y = (v - cy) * z / fy
		Z = z

		points_3D = np.zeros((X.shape[0], 3))
		points_3D[:, 0] = Z
		points_3D[:, 1] = -X
		points_3D[:, 2] = -Y

		return points_3D
