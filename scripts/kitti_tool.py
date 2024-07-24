import pickle
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray


def generate_pc_msg(pc):
	if pc.shape[1] == 4:
		fields = [
				PointField('x', 0, PointField.FLOAT32, 1),
				PointField('y', 4, PointField.FLOAT32, 1),
				PointField('z', 8, PointField.FLOAT32, 1),
				PointField('intensity', 12, PointField.FLOAT32, 1),
		]
	else:
		fields = [
				PointField('x', 0, PointField.FLOAT32, 1),
				PointField('y', 4, PointField.FLOAT32, 1),
				PointField('z', 8, PointField.FLOAT32, 1)
		]
	header = rospy.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = "map"
	return point_cloud2.create_cloud(header, fields, pc)


def generate_lidar_gt_boxes(gt_boxes_lidar):
	arr_bbox = BoundingBoxArray()
	arr_bbox.header.frame_id = "map"
	arr_bbox.header.stamp = rospy.Time.now()
	for box_lidar in gt_boxes_lidar:
		bbox = BoundingBox()
		bbox.header.stamp = rospy.Time.now()
		bbox.header.frame_id = "map"
		bbox.pose.position.x = box_lidar[0]
		bbox.pose.position.y = box_lidar[1]
		bbox.pose.position.z = box_lidar[2]
		bbox.dimensions.x = box_lidar[3]
		bbox.dimensions.y = box_lidar[4]
		bbox.dimensions.z = box_lidar[5]
		arr_bbox.boxes.append(bbox)

	return arr_bbox


def draw_2d_bbox(image, bboxes):
	for rect in bboxes:
		rect = rect.astype(int)
		top_left = (rect[0], rect[1])
		bottom_right = (rect[2], rect[3])
		color = (0, 255, 0)  # 使用绿色绘制矩形框
		thickness = 2  # 矩形框的线条粗细
		image = cv2.rectangle(image, top_left, bottom_right, color, thickness)

	return image


def read_calib_file(calib_filepath):
	""" Read a calibration file from KITTI dataset. """
	calib = {}
	with open(calib_filepath, 'r') as f:
		for line in f.readlines():
			if line.strip():  # 添加这行代码来检查是否是空行或者行只包含空白字符
				key, value = line.split(':', 1)
				# 将读取的矩阵字符串转换为NumPy数组
				calib[key] = np.array([float(x) for x in value.split()]).reshape(3, -1)
	return calib


def main():
	rospy.init_node("kitti_pc_pub")
	pc_pub = rospy.Publisher("/lidar/pointcloud", PointCloud2, queue_size=10)
	bbox_pub = rospy.Publisher("/lidar/gt_bbox", BoundingBoxArray, queue_size=1)
	image_pub = rospy.Publisher("/camera/image_2", Image, queue_size=10)
	bridge = CvBridge()

	root_path = '/home/robotics/kitti_openpcdet/kitti'
	info_path = os.path.join(root_path, 'kitti_infos_train.pkl')
	with open(info_path, 'rb') as f:
		infos = pickle.load(f)

	for i in range(len(infos)):
		i = 2
		info = infos[i]
		idx = info['point_cloud']['lidar_idx']
		pc_path = os.path.join(root_path, 'training/velodyne', idx) + '.bin'
		pc = np.fromfile(pc_path, dtype=np.float32).reshape((-1, 4))

		image_path = os.path.join(root_path, 'training/image_2', idx) + '.png'
		image = cv2.imread(image_path)

		annos = info['annos']
		bboxes = annos['bbox']
		gt_boxes_lidar = annos['gt_boxes_lidar']

		calib_path = os.path.join(root_path, 'training/calib', idx) + '.txt'
		calib = read_calib_file(calib_path)
		R0_rect = calib['R0_rect']
		R0_rect = np.insert(R0_rect, 3, values=[0,0,0], axis=0)
		R0_rect = np.insert(R0_rect, 3, values=[0,0,0,1], axis=1)

		velo = np.insert(pc[:, :3], 3, 1,axis=1).T
		pc_camframe = R0_rect.dot(velo).T
		pc_camframe = pc_camframe[:, :3]

		gt_boxes_lidar_camframe = np.insert(gt_boxes_lidar[:, :3], 3, 1,axis=1).T
		gt_boxes_lidar_camframe = R0_rect.dot(gt_boxes_lidar_camframe).T
		gt_boxes_lidar_camframe = np.hstack((gt_boxes_lidar_camframe[:, :3], gt_boxes_lidar[:, 3:]))

		image = draw_2d_bbox(image, bboxes)

		img_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
		img_msg.header.stamp = rospy.Time.now()

		pc_msg = generate_pc_msg(pc_camframe)
		arr_bbox = generate_lidar_gt_boxes(gt_boxes_lidar_camframe)

		while True:
			rospy.loginfo('publishing...')
			image_pub.publish(img_msg)
			pc_pub.publish(pc_msg)
			bbox_pub.publish(arr_bbox)
			time.sleep(0.1)


if __name__ == "__main__":
	main()
