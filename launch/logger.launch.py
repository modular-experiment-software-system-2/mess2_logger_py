
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            "parameters",
            default_value=FindPackageShare(package="mess2_logger_py").find("mess2_logger_py")+"/config/logger_py.yaml",
            description="path to the parameters file"
        ),

        DeclareLaunchArgument(
            "namespace",
            default_value="mess2",
            description="namespace of the executable"
        ),

        Node(
            package="mess2_logger_py",
            executable="logger_py",
            name=PythonExpression(["'", LaunchConfiguration('namespace'), "_logger_py'"]),
            parameters=[LaunchConfiguration("parameters")]
        )
    ])
