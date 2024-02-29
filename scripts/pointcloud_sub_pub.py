import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np


class PointCloud_SubPub:
    def __init__(self):
        rospy.init_node("pointcloud_sub_pub")

        self.pc_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.callback)
        self.pc_pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=10)
        rospy.loginfo("initialized!")

    def callback(self, msg):
        pc_data2 = ros_numpy.numpify(msg)
        pc_x = pc_data2[:, :]["x"].flatten()
        pc_y = pc_data2[:, :]["y"].flatten()
        pc_z = pc_data2[:, :]["z"].flatten()
        pc_array = np.vstack([pc_x, pc_y, pc_z]).T

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pc_msg = point_cloud2.create_cloud_xyz32(header, pc_array)

        self.pc_pub.publish(pc_msg)

        rospy.loginfo("PointCloud published")


if __name__ == "__main__":
    pointcloud_subpub = PointCloud_SubPub()
    rospy.spin()