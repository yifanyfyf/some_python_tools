import rospy
import cv2
import threading
import time
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from bev_projection import plot_bev


class processor:
	def __init__(self):
		rospy.init_node("pc2bev_node")
		self.pc_sub = rospy.Subscriber("/denoise_node/pointcloud_desnowed", PointCloud2, callback=self.callback)
		
		self.display_img = None
		
		self.display_thread = threading.Thread(target=self.display_result)
		self.display_thread.start()
		
		rospy.loginfo("initialized!")
		
	def callback(self, msg):
		pc = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
		pc_array = np.array(list(pc))
		
		# 设置鸟瞰图范围 创建图像数组
		side_range = (-15, 15)  # 左右距离
		fwd_range = (-5, 25)  # 后前距离
		res = 0.05  # 分辨率
		x_max = 1 + int((side_range[1] - side_range[0]) / res)
		y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res)
		im = np.zeros([y_max, x_max], dtype=np.uint8)
		
		im = plot_bev(pc_array, side_range, fwd_range, res, im, im_label=1)
		color_mapped_image = cv2.applyColorMap(im, cv2.COLORMAP_JET)
		color_mapped_image[im == 0] = [0, 0, 0]  # 将值为35 可走行区域 的元素映射为绿色 BGR
		color_mapped_image[im == 1] = [255, 255, 255]  # 将值为11 人 的元素映射为红色
		
		self.display_img = color_mapped_image
		
	def display_result(self):
		while True:
			if self.display_img is not None:
				cv2.imshow("img", self.display_img)
				cv2.imwrite("test.jpg", self.display_img)
				cv2.waitKey(1)
			time.sleep(0.1)


if __name__ == "__main__":
	p = processor()
	rospy.spin()
