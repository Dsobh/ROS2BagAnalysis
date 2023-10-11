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
        self.topic_rates = {}
        self.topic_stats = {}

        print("al menos he inicializado")
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
        self.topic_rates[topic_name] = 0.0
        self.topic_stats[topic_name] = {"count": 0, "start_time": time()}
        self.last_times[topic_name] = time()
        self.create_subscription(msg_type_placeholder,
                                 topic_name, callback, 10)
        self.num_topics += 1

    def update_total_rate(self, topic):
        if topic in self.topic_stats:
            self.topic_stats[topic]["count"] += 1

#Calculate and print publication rate by topic
    def print_average_rate(self, topic):
        if topic in self.topic_stats:
            stats = self.topic_stats[topic]
            current_time = time()
            elapsed_time = current_time - stats['start_time']
            average_rate = stats['count'] / elapsed_time    
            self.get_logger().info(f"Average Rate for {topic}: {average_rate:.2f} Hz")

    # Define callback functions for each topic (replace msg_type_placeholder)
    def cmd_vel_callback(self, msg):
        self.update_total_rate("/cmd_vel")
        self.print_average_rate("/cmd_vel")

    def imu_callback(self, msg):
        self.update_total_rate("/imu")
        self.print_average_rate("/imu")

    def rosout_callback(self, msg):
        self.update_total_rate("/rosout")
        self.print_average_rate("/rosout")

    def rgb_image_raw_callback(self, msg):
        self.update_total_rate("/xtion/rgb/image_raw")
        self.print_average_rate("/xtion/rgb/image_raw")

    def joint_states_callback(self, msg):
        self.update_total_rate("/joint_states")
        self.print_average_rate("/joint_states")

    def camera_info_callback(self, msg):
        self.update_total_rate("/xtion/rgb/camera_info")
        self.print_average_rate("xtion/rgb/camera_info")

    def odom_callback(self, msg):
        self.update_total_rate("/mobile_base_controller/odom")
        self.print_average_rate("/mobile_base_controller/odom")

    def sonar_base_callback(self, msg):
        self.update_total_rate("/sonar_base")
        self.print_average_rate("/sonar_base")

    def depth_image_raw_callback(self, msg):
        self.update_total_rate("/xtion/depth/image_raw")
        self.print_average_rate("/xtion/depth/image_raw")

    def scan_callback(self, msg):
        self.update_total_rate("/scan")
        self.print_average_rate("/scan")

    def sonar_torso_callback(self, msg):
        self.update_total_rate("/sonar_torso")
        self.print_average_rate("/sonar_torso")

    def tf_callback(self, msg):
        self.update_total_rate("/tf")
        self.print_average_rate("/tf")

    def tf_static_callback(self, msg):
        self.update_total_rate("/tf_static")
        self.print_average_rate("/tf_static")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    frequency_node = FrequencyNode()

    rclpy.spin(frequency_node)
    frequency_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
