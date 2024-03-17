import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy
import numpy as np
import time


class PointCloud_SubPub:
    def __init__(self):
        rospy.init_node("pointcloud_sub_pub")

        self.pc_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.callback)
        self.pc_pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=10)
        rospy.loginfo("initialized!")

        lidar_path = '/home/robotics/Downloads/cadc_data/cadcd/2019_02_27/0027/labeled/lidar_points/data/0000000000.bin'
        scan_data = np.fromfile(lidar_path, dtype=np.float32)

        # 2D array where each row contains a point [x, y, z, intensity]
        self.lidar = scan_data.reshape((-1, 4))

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

    def publish_pc(self):
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"  # Set your frame ID here

        pc_msg = point_cloud2.create_cloud(header, fields, self.lidar)

        self.pc_pub.publish(pc_msg)

        rospy.loginfo("PointCloud published")


if __name__ == "__main__":
    pointcloud_subpub = PointCloud_SubPub()
    while True:
        pointcloud_subpub.publish_pc()
        time.sleep(0.1)