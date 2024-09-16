
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("file_config", default_value="config/template.yaml", description="path to the configuration file"),
        Node(
            package="mess2_logger_py",
            executable="log_topics_to_csvs",
            name="log_topics_to_csvs",
            output="screen",
            parameters=[LaunchConfiguration("file_config")]
        ),
    ])
