import rospy
import rosbag
from sensor_msgs.msg import JointState, Image, PointCloud2
from std_msgs.msg import Header
import queue


class RosbagPlayer:
    def __init__(self, topic_name='/lidar', type=PointCloud2):
        self.pub = rospy.Publisher(topic_name, type, queue_size=10)
        self.q = queue.Queue()


class RosbagPlayerManager:
    def __init__(self, bag, topics_dict):
        self.players = {}

        for key, value in topics_dict.items():
            print(f"generate player: {key}:, type: {value}")
            self.players[key] = RosbagPlayer(topic_name=key, type=value)

        selected_topics = []
        for topic_name, topic_type in topics_dict.items():
            selected_topics.append(topic_name)

        for topic, msg, t in bag.read_messages(topics=selected_topics):
            self.players[topic].q.put(msg)
            min_qsize = self.find_min_qsize()
            if self.players[topic].q.qsize() > min_qsize + 1:
                self.players[topic].q.get()

        for name, player in self.players.items():
            print(player.q.qsize())
        print('All Messages Loaded!\n')

    def find_min_qsize(self):
        min_qsize = 1e8
        for name, player in self.players.items():
            if player.q.qsize() < min_qsize:
                min_qsize = player.q.qsize()
        return min_qsize

    def replay(self):
        print("starting replay")
        while True:
            if self.find_min_qsize() > 0:
                print(f"---------------------- left msgs number: {self.find_min_qsize()} ------------------------")
                for name, player in self.players.items():
                    rospy.loginfo(f"publishing: {name} topic")
                    msg = player.q.get()
                    msg.header.stamp = rospy.Time.now()
                    player.pub.publish(msg)

                    rospy.sleep(0.01)
            else:
                break


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
    print("bag loaded! \n")

    topics_dict = {'/lidar_pc': PointCloud2,
                   '/detection_model/vertices': PointCloud2,
                   '/whill/states/jointState': JointState,
                   '/detection_model/bev_mask/car': Image,
                   '/detection_model/bev_mask/human': Image,
                   '/detection_model/bev_mask/waypoints': Image,
                   '/detection_model/bev_mask/rendered': Image,
                   }

    player_manager = RosbagPlayerManager(bag, topics_dict)

    player_manager.replay()

if __name__ == "__main__":
    main()

