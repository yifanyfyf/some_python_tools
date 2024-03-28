import cv2
import torch
import time
import numpy as np
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


rospy.init_node('rgb_image_publisher', anonymous=True)
image_pub = rospy.Publisher('/rgb_image', Image, queue_size=10)
bridge = CvBridge()
rate = rospy.Rate(10)  # 10Hz

dog1_int0 = cv2.imread('/home/robotics/Downloads/kccs0310/saved_data/image/1710026654970724161.jpg')
dog1_int0 = cv2.resize(dog1_int0, (448, 448))
dog1_int = cv2.cvtColor(dog1_int0, cv2.COLOR_BGR2RGB)
dog1_int = torch.from_numpy(dog1_int).permute(2, 0, 1)

# plt.imshow(dog1_int.permute(1, 2, 0).numpy())
# plt.show()

weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
transforms = weights.transforms()

model = fasterrcnn_resnet50_fpn(weights=weights, progress=False)
model = model.eval().cuda()
score_threshold = .85

dog_list = [dog1_int]
images = [transforms(d).cuda() for d in dog_list]
with torch.no_grad():
    outputs = model(images)
outputs = np.hstack((outputs[0]['boxes'].cpu().numpy(),
                    outputs[0]['labels'].cpu().numpy()[:, np.newaxis],
                    outputs[0]['scores'].cpu().numpy()[:, np.newaxis]))
outputs = outputs[outputs[:, 5] > score_threshold]
num = outputs.shape[0]
for i in range(outputs.shape[0]):
    start_point = (int(outputs[i, 0]), int(outputs[i, 1]))  # Top left corner of the rectangle
    end_point = (int(outputs[i, 2]), int(outputs[i, 3]))    # Bottom right corner of the rectangle
    if outputs[i, 4] == 1:
        color = (0, 0, 255)  # Green in BGR
    elif outputs[i, 4] == 3:
        color = (0, 165, 255)  # Green in BGR
    thickness = 2        # Thickness of the rectangle border in pixels
    image_with_rectangle = cv2.rectangle(dog1_int0, start_point, end_point, color, thickness)


while not rospy.is_shutdown():
    ros_image = bridge.cv2_to_imgmsg(image_with_rectangle, "bgr8")
    image_pub.publish(ros_image)
    rate.sleep()
    rospy.loginfo(num)
