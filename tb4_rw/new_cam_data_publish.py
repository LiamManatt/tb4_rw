#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time

from std_msgs.msg import String

working_dir = "/home/ubuntu/ros2_ws/src/tb4_rw/tb4_rw"
working_file = "/home/ubuntu/ros2_ws/src/tb4_rw/tb4_rw/data2.txt"

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
    import yolo8nopreview


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
