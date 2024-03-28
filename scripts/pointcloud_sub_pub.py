import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy
import numpy as np
import time


class PointCloud_SubPub:
    def __init__(self):
        rospy.init_node("pointcloud_sub_pub")

        # self.pc_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.callback)
        self.pc_pub = rospy.Publisher("/denoise_node/pointcloud_desnowed", PointCloud2, queue_size=10)
        rospy.loginfo("initialized!")

        # read .bin
        # lidar_path = '/home/robotics/Downloads/cadc_data/cadcd/2019_02_27/0027/labeled/lidar_points/data/0000000000.bin'
        # scan_data = np.fromfile(lidar_path, dtype=np.float32)
        # self.lidar = scan_data.reshape((-1, 4))

        # read .pcd
        filename = '/home/robotics/Downloads/kccs0217/saved_data/pcd/1708213950763757202.pcd'
        # Load data from the text file, start from x-th lines
        self.lidar = np.loadtxt(filename, skiprows=11)

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
        # pc_msg1 = point_cloud2.create_cloud_xyz32(header, self.lidar)

        self.pc_pub.publish(pc_msg)

        # Extract the 4th column intensity
        col_4 = self.lidar[:, 3]
        max_val = np.max(col_4)
        min_val = np.min(col_4)
        normalized_col_4 = (col_4 - min_val) / (max_val - min_val)
        self.lidar[:, 3] = normalized_col_4

        # np.save('/home/robotics/Downloads/data4openpcdet/point_cloud.npy', self.lidar)

        rospy.loginfo("PointCloud published")


if __name__ == "__main__":
    pointcloud_subpub = PointCloud_SubPub()
    while True:
        pointcloud_subpub.publish_pc()
        time.sleep(0.01)
