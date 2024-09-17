
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mess2_logger_py_share = FindPackageShare(package="mess2_logger_py").find("mess2_logger_py")
    mess2_logger_py_parameters = mess2_logger_py_share + "/config/template.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "file_config", 
            default_value=mess2_logger_py_parameters, 
            description="path to the configuration file"),
        Node(
            package="mess2_logger_py",
            executable="log_topics_to_csvs",
            name="log_topics_to_csvs",
            output="screen",
            parameters=[LaunchConfiguration("file_config")]
        ),
    ])
