import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

def publish_bboxes():
    # 初始化ROS节点
    rospy.init_node('bbox_publisher', anonymous=True)

    # 创建一个发布器
    pub = rospy.Publisher('bounding_boxes', BoundingBoxArray, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # 创建BoundingBoxArray消息
        bbox_array_msg = BoundingBoxArray()

        # 设置Header（例如时间戳和坐标系）
        bbox_array_msg.header.frame_id = "map"
        bbox_array_msg.header.stamp = rospy.Time.now()

        # 创建一个BoundingBox消息并添加到数组
        bbox = BoundingBox()
        bbox.header.frame_id = "map"
        bbox.header.stamp = rospy.Time.now()

        # 设置边界框的中心位置
        bbox.pose.position.x = .0
        bbox.pose.position.y = .0
        bbox.pose.position.z = .0

        # 设置边界框的大小
        bbox.dimensions.x = 15  # x轴方向总长
        bbox.dimensions.y = 2.0  # y轴方向总长
        bbox.dimensions.z = 1.0  # z

        # 设置旋转角度（以弧度表示）
        roll = 0.1   # 绕 x 轴旋转
        pitch = 0.2  # 绕 y 轴旋转
        yaw = 0.3    # 绕 z 轴旋转
        
        # 将 roll, pitch, yaw 转换为四元数
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        # 设置四元数到 BoundingBox 的 pose.orientation
        bbox.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        # 添加到数组
        bbox_array_msg.boxes.append(bbox)

        # 发布消息
        pub.publish(bbox_array_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_bboxes()
    except rospy.ROSInterruptException:
        pass
