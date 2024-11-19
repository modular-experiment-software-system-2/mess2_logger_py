#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import *
from builtin_interfaces.msg import *
from diagnostic_msgs.msg import *
from example_interfaces.msg import *
from flir_camera_msgs.msg import *
from geometry_msgs.msg import *
from geographic_msgs.msg import *
from lifecycle_msgs.msg import *
from map_msgs.msg import *
from mavros_msgs.msg import *
from nav_msgs.msg import *
from pcl_msgs.msg import *
from pendulum_msgs.msg import *
from rosbag2_interfaces.msg import *
from rcl_interfaces.msg import *
from rmw_dds_common.msg import *
from sensor_msgs.msg import *
from shape_msgs.msg import *
from stereo_msgs.msg import *
from statistics_msgs.msg import *
from std_msgs.msg import *
from tf2_msgs.msg import *
from trajectory_msgs.msg import *
from turtlesim.msg import *
from unique_identifier_msgs.msg import *
from visualization_msgs.msg import *

from ament_index_python.packages import get_package_share_directory
import csv
import os
import re
import shutil
import subprocess

CHECK_ARRAY = r"(.+?)\[(\d*)\]"
CHECK_PRIMATIVE = {"int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "bool", "string", "time", "duration", "byte"}


def get_msg_path(pkg_msg: str, type_msg: str):
    """
    Get the aboslute path to the .msg file of the msg type.

    @param pkg_msg: The package where the .msg file is located.
    @param type_msg: The msg type.

    @return:  The absolute path to the .msg file.
    """
    # get path to .msg file
    path_msg = os.path.join(get_package_share_directory(pkg_msg), "msg", f"{type_msg}.msg")

    # return the path
    return path_msg


def get_msg_map(path_msg: str, struct_msg: str, output: list):
    """
    Get a map of a ROS2 msg type's field types and names as a list of str.

    @param path_msg: The absolute path to the .msg file.
    @param struct_msg: The structure of the current msg.
    @param output: A list of str representing the complete structure of the msg.

    @return: An appended list of str representing the complete structure of the msg.
    """
    # open .msg file and read lines
    with open(path_msg, "r") as file_msg:
        lines = file_msg.readlines()
    
    # remove empty and commented lines
    lines = [line.strip() for line in lines if line.strip() and not line.strip().startswith("#")]
    
    # for each line retrieve field name and field type if any
    for line in lines:
        # retrieve field types and names
        segments = line.split()
        type_field = segments[0]
        name_field = segments[1]
        
        # if type_field is array extract true field type
        _is_array = is_array(type_field=type_field)
        if (_is_array):
            type_array = _is_array.group(1)
            size_array = _is_array.group(2)
            size_array = int(size_array) if size_array else "unspecified"
            type_field = type_array
        
        # if type_field is primative append output
        _is_primative = is_primative(type_field=type_field)
        if (_is_primative):
            output.append(f"{struct_msg}.{name_field}")
        # if type_field is not primative assume message type
        else:
            # if message type contains "/" then different pkg
            if (type_field.__contains__("/")):
                type_field = type_field.split("/")
                pkg_msg = type_field[0]
                type_msg = type_field[1]
                get_msg_map(get_msg_path(pkg_msg=pkg_msg, type_msg=type_msg), struct_msg=f"{struct_msg}.{name_field}", output=output)
            # if message type does not contain "/" then same pkg
            else:
                pkg_msg = path_msg.split("/")[-3]
                type_msg = type_field
                get_msg_map(get_msg_path(pkg_msg=pkg_msg, type_msg=type_msg), struct_msg=f"{struct_msg}.{name_field}", output=output)
    
    # return the complete structure
    return output


def is_array(type_field: str):
    """
    Check if a field type is an array.

    @param type_field: The type of the field.

    @return: The match object if the field type is an array, None otherwise.
    """
    # return the match object
    return re.match(CHECK_ARRAY, type_field)


def is_primative(type_field: str):
    """
    Check if a field type is a primative data type

    @param type_field: The type of the field.

    @return: True if the field type is a primative data type, False otherwise.
    """
    # return the bool
    return type_field in CHECK_PRIMATIVE


def get_topic_type(topic_msg: str):
        """
        Get the msg type of an advertised topic.

        @param topic_msg: The topic of which the msg type is to be retrieved.

        @return: The message type of the topic.
        """
        # run subcommand to determine topic type
        command = ["ros2", "topic", "type", topic_msg]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        output = result.stdout.strip().split("/")

        # return command output
        return output


class LogTopicsToCSVs(Node):
    """
    A class to log multiple ROS2 topic msgs to log .csv files.

    This class dynamically initializes logger instances for specific ROS2 topics. It supports dynamic message handling, subclass instance creation, and subscription creation.
    """
    def __init__(self):
        """
        
        """
        # initialize node
        super().__init__("log_topics_to_csvs")

        # declare parameters
        self.declare_parameter("dir_logs", "Desktop/logs")
        self.declare_parameter("dirs_sub", ["actor", "actor1"])
        self.declare_parameter("topics", ["/topic1", "/topic2"])
        self.declare_parameter("period_timer", 5.0)

        # retrieve parameters
        dir_logs = os.path.join(os.path.expanduser("~"), self.get_parameter("dir_logs").get_parameter_value().string_value)
        dirs_sub = self.get_parameter("dirs_sub").get_parameter_value().string_array_value
        topics = self.get_parameter("topics").get_parameter_value().string_array_value
        period_timer = self.get_parameter("period_timer").get_parameter_value().double_value

        # create log directories if they do not exist
        self.path_log = os.path.join(dir_logs, *dirs_sub)
        if not os.path.exists(self.path_log):
            os.makedirs(self.path_log)

        # empty log directories if they contain files
        for file_log in os.listdir(self.path_log):
            path_file = os.path.join(self.path_log, file_log)
            if os.path.isfile(path_file) or os.path.islink(path_file):
                os.unlink(path_file)
            elif os.path.isdir(path_file):
                shutil.rmtree(path_file)
        
        # sort advertised and unadvertised topics
        self.topics_advertised = []
        self.topics_unadvertised = []
        self.loggers = []
        for topic in topics:
            try:
                # get the message type
                _ = get_topic_type(topic)

                # add to advertised topics
                self.add_advertised_topic(topic_msg=topic)

            except Exception as e:
                # add to unadvertised topics
                self.add_unadvertised_topic(topic_msg=topic)
        
        # create timer function to periodically check topics and add them to advertised if advertised
        self.timer = self.create_timer(period_timer, self.remove_unavertised_topics)


    class LogTopicToCSV():
        """
        A class to log a ROS2 topic's msgs to a log .csv file.

        This class initializes a logger for a specific ROS2 topic, stores information about the topic and its corresponding msg type. It supports dynamic message handling and writing callback data to a log .csv file.
        """
        def __init__(self, topic, path_log):
            """
            Initialize the logger instance by setting the information attributes.

            @param topic: The topic of which the instance is created for.
            @param path_log: The absolute path to the directory where the log .csv file is to be written.
            """
            self.topic_msg = topic
            self.set_logger_info(path_log=path_log)


        def set_logger_info(self, path_log: str):
            """
            Set a logger instance's information including the msg type and map.

            @param path_log: The absolute path to the log .csv file for the topic.
            """
            # determine the topic type
            type_topic = get_topic_type(self.topic_msg)

            # store and map message type
            self.type_msg = type_topic[2]
            path_msg = get_msg_path(pkg_msg=type_topic[0], type_msg=self.type_msg)
            self.map_msg = get_msg_map(path_msg=path_msg, struct_msg="msg", output=[])
            
            # store path for log .csv file
            self.path_log = os.path.join(path_log, f"{self.topic_msg.replace('/', '_')}.csv")

            # append msg map to log .csv file
            with open(self.path_log, "a", newline="") as file_log:
                writer = csv.writer(file_log)
                writer.writerow(self.map_msg)

        def callback(self, msg):
            """
            The default callback function for a logger instance.

            @param msg: The msg passed to the callback once the subscription has been created for the logger instance.
            """
            list = []
            for struct_field in self.map_msg:
                # evaluate msg for each structure element
                try:
                    data_field = str(eval(struct_field))
                    if data_field.startswith("(") and data_field.endswith(")"):
                        data_field = data_field.replace(",", "")
                    list.append(data_field)
                # append empty string if cannot evaluate structure element
                except Exception as e:
                    list.append("")
            
            # append current msg to log .csv file
            with open(self.path_log, "a", newline="") as file_log:
                writer = csv.writer(file_log, delimiter="\t")
                writer.writerow(list)
            list.clear()
    

    def set_logger_subscription(self, logger: LogTopicToCSV):
        """
        Create a subscription for a logger instance.

        @param logger: The logger instance of which a subscription is to be created.
        """
        # create subscription attribute in logger instance
        logger.subscription = self.create_subscription(
            eval(logger.type_msg),
            logger.topic_msg,
            logger.callback,
            10
        )


    def print_logger_info(self, logger: LogTopicToCSV):
        """
        Print a logger instance's named topic, msg type, and msg map.

        @param logger: The logger instance whose info is to be printed.
        """
        self.get_logger().info(logger.topic_msg)
        self.get_logger().info(logger.type_msg)
        self.get_logger().info(f"{logger.map_msg}")
    

    def add_advertised_topic(self, topic_msg: str):
        """
        Add a topic to the list attribute of advertised topics, create a logger instance for that topic, and set its subscription.

        @param topic_msg: The advertised topic.
        """
        # create logger instance for topic
        logger = self.LogTopicToCSV(topic=topic_msg, path_log=self.path_log)

        # append logger to loggers and topic_msg to advertised_topics
        self.loggers.append(logger)
        self.topics_advertised.append(topic_msg)

        # set logger subscription
        self.set_logger_subscription(logger=logger)
        self.get_logger().info(f"added {topic_msg} to advertised")


    def add_unadvertised_topic(self, topic_msg: str):
        """
        Add a topic to the list attribute of unadvertised topics.

        @param topic_msg: The unadvertised topic.
        """
        # append topic_msg to unadvertised_topics
        self.topics_unadvertised.append(topic_msg)
        self.get_logger().info(f"added {topic_msg} to unadvertised")


    def remove_unavertised_topics(self):
        """
        Timer callback for removing unadvertised topics if they become advertised.
        """
        self.get_logger().info("checking unadvertised topics")
        for topic in self.topics_unadvertised:
            try:
                # get the message type
                type_topic = get_topic_type(topic)

                # add to advertised topics
                self.add_advertised_topic(topic_msg=topic)

                # remove topic from unadvertised topics
                self.topics_unadvertised.remove(topic)

            except Exception as e:
                pass

        # if all topics are advertised destroy timer
        if (len(self.topics_unadvertised) == 0):
            self.get_logger().info("all topics advertised")
            self.timer.destroy()


def main(args=None):
    """
    The main function responsible for creating an instance of the node class and starting the threaded executor.

    @param args: Any arguments for initializing the ROS2 client library for Python3.
    """
    try:
        rclpy.init(args=args)
        node = LogTopicsToCSVs()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
