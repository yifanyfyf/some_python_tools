import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image
import message_filters
from cv_bridge import CvBridge
import ros_numpy
import numpy as np


class PCIMG_Ts:
    def __init__(self):
        rospy.init_node("pointcloud_image_ts")

        self.pc_sub = message_filters.Subscriber("/ouster/points", PointCloud2)
        self.img_sub = message_filters.Subscriber("c1_camera/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.img_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)
        self.bridge = CvBridge()

        self.pc_pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=10)
        self.img_pub = rospy.Publisher("/image", Image, queue_size=10)

        rospy.loginfo("initialized!")

    def callback(self, msg_pc, msg_img):
        rospy.loginfo("received msg")

        pc_data2 = ros_numpy.numpify(msg)
		pc_x = pc_data2[:, :]["x"].flatten()
		pc_y = pc_data2[:, :]["y"].flatten()
		pc_z = pc_data2[:, :]["z"].flatten()
		pc_intensity = pc_data2[:, :]["intensity"].flatten()
		pc_array = np.vstack([pc_x, pc_y, pc_z, pc_intensity]).T

		fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1),
			PointField('intensity', 12, PointField.FLOAT32, 1),
		]

		header = rospy.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "map"  # Set your frame ID here

		pc_msg = point_cloud2.create_cloud(header, fields, pc_array)

        self.pc_pub.publish(pc_msg)

        self.showimg = self.bridge.imgmsg_to_cv2(msg_img, "bgr8")
        img_msg = self.bridge.cv2_to_imgmsg(self.showimg, encoding='bgr8')
        img_msg.header.stamp = rospy.Time.now()
        self.img_pub.publish(img_msg)


if __name__ == "__main__":
    ts = PCIMG_Ts()
    rospy.spin()
