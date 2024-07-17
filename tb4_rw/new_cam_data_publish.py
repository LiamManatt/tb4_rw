#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
import argparse
from std_msgs.msg import String
import os
parser = argparse.ArgumentParser(description="Process a directory and a file name.")

parser.add_argument(
"-d", "--directory",
required=True,
help="Directory to be processed."
)

parser.add_argument(
"-f", "--file",
type=str,
required=True,
help="File name to be processed."
)

args = parser.parse_args()

working_dir = args.directory
working_file = working_dir + '/' + args.file

publish_data = []
with open(working_file, "w") as file:
    file.write("")  # Overwrite the file with an empty string

class new_cam_data_publish(Node):

    def __init__(self):                  # this is constructor 

        super().__init__("new_cam_data_publish")   # node name, super() give access to 
                                         # methods and properties of a parent or sibling class.
        self.publisher_ = self.create_publisher(String, 'camera_data',10)
        timer_period = 0.3  # seconds
        self.get_logger().info("Obstacle coordinates now being published...")
        self.create_timer(timer_period, self.timer_callback)  # create a timer callback every 0.3 sec

    def timer_callback(self):
        global publish_data
        msg = String()                   # we publish "msg" to the topic
        msg.data = str(publish_data)
        self.publisher_.publish(msg)


def start_camera(args=None):
    import sys
    sys.path.append(working_dir)
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from . import yolo8nopreview
    # from . import yolonav2


def publish(args=None):
    global publish_data
    time.sleep(3)  # wait for the camera to be ready
    while rclpy.ok():

        with open(working_file, "r") as file:
            first_char = file.read(1)
            if first_char:    # check if the file is empty
                lines = file.readlines()
            else:
                continue

        time.sleep(0.3)  # seconds

        if len(lines) >= 1:
            print(lines[len(lines)-1])
            publish_data = lines[len(lines)-1]


def main(args=None):
    rclpy.init(args=args) # initialize rclpy
    node = new_cam_data_publish()     # create node

    publish_thread = threading.Thread(target = publish)
    camera_thread = threading.Thread(target = start_camera)
    publish_thread.start()
    camera_thread.start()

    rclpy.spin(node)      # node will be kept alive until ^C
    node.destroy_node()
    rclpy.shutdown()


################################################################

# if __name__ == '__main__':
#     main()
