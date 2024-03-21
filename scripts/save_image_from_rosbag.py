"""
播放rosbag，从中保存图片和点云，用于OpenCalib标定雷达到相机的外参
"""
import numpy as np
import rospy
import message_filters
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import cv2


class ROS_Tkinter_App:
	def __init__(self, master):
		self.master = master
		master.title("ROS Tkinter App")
 
		self.save_button = tk.Button(self.master, text="Save Current Image", command=self.button_click)
		self.save_button.pack()
 
		# ROS Initialization
		rospy.init_node('ros_tkinter_app', anonymous=True)
 
		self.pc_sub = message_filters.Subscriber("/ouster/points", PointCloud2)
		self.img_sub = message_filters.Subscriber("c1_camera/image_raw", Image)
		ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.img_sub], queue_size=1, slop=0.1)
		ts.registerCallback(self.callback)
		self.bridge = CvBridge()

		self.pc_pub = rospy.Publisher("/processed/pointcloud/", PointCloud2, queue_size=10)
		self.img_pub = rospy.Publisher("/processed/image", Image, queue_size=10)
 
	def callback(self, msg_pc, msg_img):
		rospy.loginfo("ROS Topic Data")

		self.image_name = msg_img.header.stamp
		self.cv_image = self.bridge.imgmsg_to_cv2(msg_img, "bgr8")

		pc_data = point_cloud2.read_points(msg_pc, field_names=('x', 'y', 'z', "intensity"), skip_nans=True)
		pc_array = np.array(list(pc_data))

		distances = np.linalg.norm(pc_array[:, :3], axis=1)
		self.pc_array = pc_array[distances >= 1]

		header = rospy.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "map"
		pc_msg = point_cloud2.create_cloud_xyz32(header, self.pc_array[:, :3])
		self.pc_pub.publish(pc_msg)

		img_msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
		img_msg.header.stamp = rospy.Time.now()
		self.img_pub.publish(img_msg)

		# self.button_click()

	def button_click(self):
		save_path = "/backup/0310/saved_data/image/" + self.image_name + ".jpg"
		cv2.imwrite(save_path, self.cv_image)

		num_points = self.pc_array.shape[0]
		WIDTH = "WIDTH " + str(num_points)
		POINTS = "POINTS " + str(num_points)
		header_lines = ["# .PCD v0.7 - Point Cloud Data file format",
						"VERSION 0.7",
						"FIELDS x y z intensity",
						"SIZE 4 4 4 4",
						"TYPE F F F F",
						"COUNT 1 1 1 1",
						WIDTH,
						"HEIGHT 1",
						"VIEWPOINT 0 0 0 1 0 0 0",
						POINTS,
						"DATA ascii"]

		# 设置保存的格式，前三列保留6位小数，第4列保留整数
		fmt = "%.6f %.6f %.6f %d"
		pcd_savepath = "/backup/0310/saved_data/pcd/" + self.image_name + '/.pcd.txt'
		np.savetxt(pcd_savepath, self.pc_array, fmt=fmt, header='\n'.join(header_lines), comments='')

		print("data saved!")
 
 
def main():
	root = tk.Tk()
	root.geometry("200x200")    # width height
	app = ROS_Tkinter_App(root)
	root.mainloop()
 
 
if __name__ == '__main__':
	main()
