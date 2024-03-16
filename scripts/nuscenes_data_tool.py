import os
import time
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud
from nuscenes.utils.geometry_utils import transform_matrix
import cv2
import numpy as np
import open3d as o3d
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
from rangeview_projection import rv_project


rospy.init_node("pc_pub_node")
pc_pub = rospy.Publisher("/nuscenes/top_lidar/pointcloud", PointCloud2, queue_size=10)
img_pub = rospy.Publisher("nuscenes/camera_front/image", Image, queue_size=10)
bridge = CvBridge()

root_path = "/home/robotics/Downloads/nuscenes"
nusc = NuScenes(version='v1.0-mini', dataroot=root_path, verbose=True)

my_scene = nusc.scene[0]
for scene_i in nusc.scene:
	first_sample_token = my_scene['first_sample_token']
	all_sample = []
	sample = nusc.get('sample', first_sample_token)
	all_sample.append(sample)
	while True:
		try:
			sample = nusc.get('sample', sample["next"])
			all_sample.append(sample)
		except:
			break

	sample = all_sample[2]

	cam_data_token1 = sample['data']['CAM_FRONT_LEFT']
	cam_data_token2 = sample['data']['CAM_FRONT']
	cam_data_token3 = sample['data']['CAM_FRONT_RIGHT']
	cam_data_token4 = sample['data']['CAM_BACK_LEFT']
	cam_data_token5 = sample['data']['CAM_BACK']
	cam_data_token6 = sample['data']['CAM_BACK_RIGHT']
	cam_data1 = nusc.get("sample_data", cam_data_token1)
	cam_data2 = nusc.get("sample_data", cam_data_token2)
	cam_data3 = nusc.get("sample_data", cam_data_token3)
	cam_data4 = nusc.get("sample_data", cam_data_token4)
	cam_data5 = nusc.get("sample_data", cam_data_token5)
	cam_data6 = nusc.get("sample_data", cam_data_token6)

	image1 = cv2.imread(os.path.join(root_path, cam_data1["filename"]))
	image2 = cv2.imread(os.path.join(root_path, cam_data2["filename"]))
	image3 = cv2.imread(os.path.join(root_path, cam_data3["filename"]))
	image4 = cv2.imread(os.path.join(root_path, cam_data4["filename"]))
	image5 = cv2.imread(os.path.join(root_path, cam_data5["filename"]))
	image6 = cv2.imread(os.path.join(root_path, cam_data6["filename"]))

	top_lidar_token = sample['data']['LIDAR_TOP']
	top_lidar_data = nusc.get('sample_data', top_lidar_token)
	pcd_bin_file = os.path.join(nusc.dataroot, top_lidar_data['filename'])
	pc = LidarPointCloud.from_file(pcd_bin_file)
	pcd = pc.points.T
	intensity = pcd[:, 3][:, np.newaxis]
	pcd = pcd[:, :3]

	# 定义欧拉角
	theta = np.radians(270)  # 将角度转换为弧度
	rotation_matrix = np.array([
		[np.cos(theta), -np.sin(theta), 0],
		[np.sin(theta), np.cos(theta), 0],
		[0, 0, 1]
	])
	pcd = rotation_matrix @ pcd.T
	pcd = pcd.T

	# ego_pose = nusc.get('ego_pose', top_lidar_data['ego_pose_token'])
	# cal_sensor = nusc.get('calibrated_sensor', top_lidar_data['calibrated_sensor_token'])
	#
	# car_from_senor = transform_matrix(cal_sensor['translation'], Quaternion(cal_sensor['rotation']), inverse=False)
	# global_from_car = transform_matrix(ego_pose['translation'], Quaternion(ego_pose['rotation']), inverse=False)
	# global_from_sensor = np.dot(global_from_car, car_from_senor)
	#
	# ones_column = np.ones((pcd.shape[0], 1))
	# pc_one = np.hstack((pcd, ones_column))
	# pc_worldcoord = global_from_sensor @ pc_one.T
	# pc_worldcoord = (pc_worldcoord.T)[:, :3]

	gt_from = 'lidarseg'
	lidarseg_labels_filename = os.path.join(nusc.dataroot,
											nusc.get(gt_from, sample["data"]['LIDAR_TOP'])['filename'])
	points_label = np.fromfile(lidarseg_labels_filename, dtype=np.uint8)[:, np.newaxis]
	pcd = np.hstack((pcd, intensity, points_label))

	# Calculate the pairwise distances between points
	distances = np.linalg.norm(pcd[:, :3], ord=2, axis=1)
	pcd = pcd[distances > 2.5]

	proj = rv_project(pcd)

	# point_cloud = o3d.geometry.PointCloud()
	# point_cloud.points = o3d.utility.Vector3dVector(pcd)
	# o3d.visualization.draw_geometries([point_cloud])

	while True:
		header = rospy.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'map'
		pc_msg = point_cloud2.create_cloud_xyz32(header, pcd[:, :3])
		pc_pub.publish(pc_msg)

		img_msg = bridge.cv2_to_imgmsg(image2, encoding='bgr8')
		img_msg.header.stamp = rospy.Time.now()
		img_pub.publish(img_msg)

		rospy.loginfo("publishing pointcloud")
		time.sleep(0.1)



