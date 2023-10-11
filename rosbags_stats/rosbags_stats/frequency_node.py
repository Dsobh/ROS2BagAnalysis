import rclpy
from rclpy.node import Node
from time import time
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from tf2_msgs.msg import TFMessage


class FrequencyNode(Node):
    def __init__(self):
        super().__init__('frequency_node')
        self.last_times = {}
        self.num_topics = 0
        self.total_rate = 0.0

        # Subscribe to each topic and set up its callback
        self.subscribe_to_topic("/cmd_vel", Twist, self.cmd_vel_callback)
        self.subscribe_to_topic("/imu", Imu, self.imu_callback)
        self.subscribe_to_topic("/rosout", Log, self.rosout_callback)
        self.subscribe_to_topic("/xtion/rgb/image_raw",
                                Image, self.rgb_image_raw_callback)
        self.subscribe_to_topic(
            "/joint_states", JointState, self.joint_states_callback)
        self.subscribe_to_topic("/xtion/rgb/camera_info",
                                CameraInfo, self.camera_info_callback)
        self.subscribe_to_topic(
            "/mobile_base_controller/odom", Odometry, self.odom_callback)
        self.subscribe_to_topic("/sonar_base", Range, self.sonar_base_callback)
        self.subscribe_to_topic("/xtion/depth/image_raw",
                                Image, self.depth_image_raw_callback)
        self.subscribe_to_topic("/scan", LaserScan, self.scan_callback)
        self.subscribe_to_topic("/sonar_torso", Range,
                                self.sonar_torso_callback)
        self.subscribe_to_topic("/tf", TFMessage, self.tf_callback)
        self.subscribe_to_topic("/tf_static", TFMessage,
                                self.tf_static_callback)

    def subscribe_to_topic(self, topic_name, msg_type_placeholder, callback):
        self.last_times[topic_name] = time()
        self.create_subscription(msg_type_placeholder,
                                 topic_name, callback, 10)
        self.num_topics += 1

    def calculate_rate(self, topic):
        current_time = time()
        elapsed_time = current_time - self.last_times[topic]
        self.last_times[topic] = current_time
        rate = 1.0 / elapsed_time
        return rate

    def update_total_rate(self, topic):
        rate = self.calculate_rate(topic)
        self.total_rate += rate

    def print_average_rate(self):
        if self.num_topics > 0:
            average_rate = self.total_rate / self.num_topics
            self.get_logger().info(f"Average Rate: {average_rate:.2f} Hz")

    # Define callback functions for each topic (replace msg_type_placeholder)
    def cmd_vel_callback(self, msg):
        self.update_total_rate("/cmd_vel")
        self.print_average_rate()

    def imu_callback(self, msg):
        self.update_total_rate("/imu")
        self.print_average_rate()

    def rosout_callback(self, msg):
        self.update_total_rate("/rosout")
        self.print_average_rate()

    def rgb_image_raw_callback(self, msg):
        self.update_total_rate("/xtion/rgb/image_raw")
        self.print_average_rate()

    def joint_states_callback(self, msg):
        self.update_total_rate("/joint_states")
        self.print_average_rate()

    def camera_info_callback(self, msg):
        self.update_total_rate("/xtion/rgb/camera_info")
        self.print_average_rate()

    def odom_callback(self, msg):
        self.update_total_rate("/mobile_base_controller/odom")
        self.print_average_rate()

    def sonar_base_callback(self, msg):
        self.update_total_rate("/sonar_base")
        self.print_average_rate()

    def depth_image_raw_callback(self, msg):
        self.update_total_rate("/xtion/depth/image_raw")
        self.print_average_rate()

    def scan_callback(self, msg):
        self.update_total_rate("/scan")
        self.print_average_rate()

    def sonar_torso_callback(self, msg):
        self.update_total_rate("/sonar_torso")
        self.print_average_rate()

    def tf_callback(self, msg):
        self.update_total_rate("/tf")
        self.print_average_rate()

    def tf_static_callback(self, msg):
        self.update_total_rate("/tf_static")
        self.print_average_rate()

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    node = FrequencyNode()
    node.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
