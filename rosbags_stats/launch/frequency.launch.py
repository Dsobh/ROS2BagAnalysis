from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_file', description='Path to the ROS2 bag file'),
        DeclareLaunchArgument(
            'topic_name', description='Name of the topic to subscribe to'),
        DeclareLaunchArgument(
            'msg_type', description='Message type for the topic (e.g., std_msgs/msg/String)'),
        Node(
            package='rosbags_stats',  # Replace with the name of your ROS 2 package
            executable='frequency_node',  # Replace with the name of your node executable
            name='rosbags_stats',
            output='screen',
            remappings=[
                ('/your_remapped_topic', LaunchConfiguration('topic_name'))],
            parameters=[{'bag_file': LaunchConfiguration('bag_file')}],
            arguments=['--msg-type', LaunchConfiguration('msg_type')],
        ),
        LogInfo(
            condition=IfCondition(True),
            text=[
                'Starting bag reader node with bag file:',
                LaunchConfiguration('bag_file'),
                'topic name:',
                LaunchConfiguration('topic_name'),
                'message type:',
                LaunchConfiguration('msg_type'),
            ],
        ),
    ])
