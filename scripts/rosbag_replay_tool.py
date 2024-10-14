import rospy
import rosbag
from sensor_msgs.msg import JointState, Image, PointCloud2
from std_msgs.msg import Header
import queue
from tqdm import tqdm


class RosbagPlayer:
    def __init__(self, topic_name='/lidar_pc', type=PointCloud2):
        self.topic_name = topic_name
        self.pub = rospy.Publisher(topic_name, type, queue_size=10)
        self.content = []

    def content_length(self):
        return len(self.content)

class RosbagPlayerManager:
    def __init__(self, bag, topics_dict):
        self.players = {}       # It is a dict. key is the topic name, value is the instance of Class RosbagPlayer

        for key, value in topics_dict.items():
            print(f"generate player: {key}:, type: {value}")
            self.players[key] = RosbagPlayer(topic_name=key, type=value)

        selected_topics = []
        for topic_name, topic_type in topics_dict.items():
            selected_topics.append(topic_name)

        total_messages = bag.get_message_count(selected_topics)
        for topic, msg, t in tqdm(bag.read_messages(topics=selected_topics), total=total_messages):
            self.players[topic].content.append(msg)
            min_qsize = self.find_min_qsize()
            if self.players[topic].content_length() > min_qsize + 1:
                self.players[topic].content.pop(0)

        print("\nStarting loading messages......\n")
        for key, value in self.players.items():
            self.total_msg_number = value.content_length()
            print(f"There is {self.total_msg_number} messages in topic: {key}")

        print("\nAll messages loaded! \nReplay will start soon!")
        rospy.sleep(3)

    def replay_from_middle(self, start_idx=100):
        print(f"\n replay from {start_idx}")
        for i in range(start_idx, self.total_msg_number):
            for topic_name, player in self.players.items():
                msg = player.content[i]
                msg.header.stamp = rospy.Time.now()
                player.pub.publish(msg)
                rospy.loginfo(f"playing frame: {i} / {self.total_msg_number}")
                rospy.sleep(0.01)

    def find_min_qsize(self):
        min_qsize = 1e8
        for name, player in self.players.items():
            if player.content_length() < min_qsize:
                min_qsize = player.content_length()
        return min_qsize


def main():
    """
    fill in bag address
    fill in the topics_dict with the topics that you want to replay
    :return:
    """
    bag_addr = '/home/robotics/data_1013/2024-10-13-20-02-09.bag'

    rospy.init_node('rosbag_player_node')

    rospy.loginfo('Opening bag')
    bag = rosbag.Bag(bag_addr)
    info = bag.get_type_and_topic_info()
    for topic, topic_info in info.topics.items():
        msg_type = topic_info.msg_type
        print(f"Topic: {topic}, Type: {msg_type}")
    print("Bag Loaded! \n")

    topics_dict = {'/lidar_pc': PointCloud2,
                   '/detection_model/vertices': PointCloud2,
                   '/whill/states/jointState': JointState,
                   '/detection_model/bev_mask/car': Image,
                   '/detection_model/bev_mask/human': Image,
                   '/detection_model/bev_mask/waypoints': Image,
                   '/detection_model/bev_mask/rendered': Image,
                   }

    player_manager = RosbagPlayerManager(bag, topics_dict)

    player_manager.replay_from_middle(100)

if __name__ == "__main__":
    main()

