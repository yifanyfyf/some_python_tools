import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray


# 初始化ROS节点
	rospy.init_node('bbox_publisher', anonymous=True)
	pub = rospy.Publisher('bounding_boxes', BoundingBoxArray, queue_size=10)
	bbox_array_msg = BoundingBoxArray()
	bbox_array_msg.header.frame_id = "map"
	bbox_array_msg.header.stamp = rospy.Time.now()

	lidar_boxes = example_lidar_pc.lidar_boxes

	for box in lidar_boxes:
		x, y, z = box.x, box.y, box.z
		x -= ego_x
		y -= ego_y
		z -= ego_z
		width, length, height = box.width, box.length, box.height

		bbox = BoundingBox()
		bbox.header.frame_id = "map"
		bbox.header.stamp = rospy.Time.now()

		# 设置边界框的中心位置
		bbox.pose.position.x = x
		bbox.pose.position.y = y
		bbox.pose.position.z = z

		# 设置边界框的大小
		bbox.dimensions.x = width  # width
		bbox.dimensions.y = length  # length
		bbox.dimensions.z = height  # height

		# 添加到数组
		bbox_array_msg.boxes.append(bbox)

	pub.publish(bbox_array_msg)
	time.sleep(0.1)
	rospy.loginfo('publishing')
