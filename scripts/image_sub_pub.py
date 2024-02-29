import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time


class Image_SubPub:
    def __init__(self):
        self.showimg = None
        self.display_thread = threading.Thread(target=self.display_img)
        self.display_thread.start()

        rospy.init_node("image_sub_display_node")
        self.image_sub = rospy.Subscriber("c1_camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("c1_camera/image", Image, queue_size=10)

        rospy.loginfo("initialized!")

    def callback(self, msg):
        rospy.loginfo("received image")
        self.showimg = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        img_msg = self.bridge.cv2_to_imgmsg(self.showimg, encoding='bgr8')
        img_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish(img_msg)

    def display_img(self):
        while True:
            if self.showimg is not None:
                img_show = cv2.resize(self.showimg, (1280, 720))
                cv2.imshow("image", img_show)
                cv2.waitKey(1)
            time.sleep(0.1)


if __name__ == "__main__":
    da_ros = Image_SubPub()
    rospy.spin()
