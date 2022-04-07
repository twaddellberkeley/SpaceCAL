import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from interfaces.msg import DisplayData, MotorData
from datetime import date, datetime

import rosbag2_py


class BagRecorder(Node):
    # Here we add topics that we want to record with the following format:
    # dictionary object with tupels containing (topicName, msgType, ROS2Type, etc..)
    topicList = [('buttons_topic', 'std_msgs/msg/String', String),
                 ('display_topic', 'interfaces/msg/DisplayData', DisplayData)]
    subList = []

    def __init__(self):
        super().__init__('bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        # get current date to make unique bag
        dt = "" + date.today().strftime("%b-%d-%Y") + datetime.now().strftime("%H:%M:%S")
        print("d4 =" + dt)
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='my_bags/' + dt,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Create topic metadata and subcribers for the topic
        for index in range(len(self.topicList)):
            # topic metadta
            topic = self.topicList[index]
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic[0],  # topic name
                type=topic[1],  # topic message type
                serialization_format='cdr')
            self.writer.create_topic(topic_info)

            # create topic subscriber
            self.subList.append(self.create_subscription(
                topic[2],  # ROS2 message type
                topic[0],  # Topic name
                self.topic_callback,
                10))

    def topic_callback(self, msg):
        for topic in self.topicList:
            print(type(msg) == type(topic[2]))
            print(type(msg))
            print(type(topic[2]))
            if type(msg) == type(topic[2]):
                self.writer.write(
                    topic[0],
                    serialize_message(msg),
                    self.get_clock().now().nanoseconds)
                return True
        print('Error recording msg')
        return False


def main(args=None):
    rclpy.init(args=args)
    sbr = BagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
