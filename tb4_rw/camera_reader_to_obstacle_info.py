#!/usr/bin/env python3
import rclpy
import re
import time
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

pos_arr = [[0., 0.], [0., 0.], [0., 0.]]
x_velo, z_velo = 0., 0.
last_callback_time = None

class camera_data_reader(Node):

    def __init__(self):
        super().__init__('camera_reader_to_obstacle_info')

        # subscriber for camera data
        self.subscription = self.create_subscription(String, '/camera_data', self.reader_callback, 10)
        self.subscription  # prevent unused variable warning

        # publisher for obstacle info
        self.publisher_ = self.create_publisher(Float64MultiArray, '/obstacle_info', 10)
        timer_period = 0.3 # sec
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def reader_callback(self, msg):
        global pos_arr, last_callback_time, x_velo, z_velo
        elapsed_time = 0
        callback_start_time = time.time()
        # Calculate elapsed time from the previous callback
        if last_callback_time is not None:
            elapsed_time = callback_start_time - last_callback_time
            print('Elapsed time since last callback:', elapsed_time)
        last_callback_time = callback_start_time

        # refresh data
        for i in range(3):
            pos_arr[i][1] = pos_arr[i][0]

        # self.get_logger().info('123 "%s"' % msg.data)
        msg.data = msg.data.strip("[")
        numbers = msg.data.split(", ")
        numbers[2] = re.sub(r'\]$', '', numbers[2])
        # store data
        for i in range(3):
            if numbers[i] == 'None' or numbers[i] == 'None\n':
                pos_arr[i][0] = None
            else:
                print(numbers)
                pos_arr[i][0] = float(numbers[i])

        # x-component (left-right)
        if (pos_arr[0][0] != None and pos_arr[0][1] != None
            and (pos_arr[0][0] - pos_arr[0][1] > 0)):
            print("obstacle moving right...")
        elif (pos_arr[0][0] != None and pos_arr[0][1] != None
            and (pos_arr[0][0] - pos_arr[0][1] < 0)):
            print("obstacle moving left...")
        elif (pos_arr[0][0] != None and pos_arr[0][1] != None
            and (pos_arr[0][0] - pos_arr[0][1] == 0)):
            print("obstacle horizontally stops...")

        # z-component (front-back)
        if (pos_arr[2][0] != None and pos_arr[2][1] != None
            and (pos_arr[2][0] - pos_arr[2][1] > 0)):
            print("obstacle moving further...")
        elif (pos_arr[2][0] != None and pos_arr[2][1] != None
            and (pos_arr[2][0] - pos_arr[2][1] < 0)):
            print("obstacle moving closer...")
        elif (pos_arr[2][0] != None and pos_arr[2][1] != None
            and (pos_arr[2][0] - pos_arr[2][1] == 0)):
            print("obstacle radially stops...")
        else:
            print("waiting for obstacle...")

        if pos_arr[0][0] != None and pos_arr[0][1] != None and last_callback_time is not None and elapsed_time:
            x_velo, z_velo = self.get_velo(elapsed_time)

            print('z velocity:', z_velo)
        else:
            print('velocity not available...')

    def publisher_callback(self):
        global pos_arr, x_velo, z_velo
        msg = Float64MultiArray()  # data to be published

        ###### need some modification #####
        if (pos_arr[0][0] is not None
            and pos_arr[0][1] is not None
            and pos_arr[2][0] is not None
            and pos_arr[2][1] is not None):
            msg.data = [pos_arr[0][0], pos_arr[1][0], pos_arr[2][0]]
            # self.get_logger().info('Publishing: "%f"' % msg.data)
        else:
            msg.data = [1e6, 1e6, 1e6]   # publish large number to indicate "None"
        self.publisher_.publish(msg)

    def get_velo(self, elapsed_time):
        x_velo = (pos_arr[0][0] - pos_arr[0][1])/elapsed_time
        z_velo = (pos_arr[2][0] - pos_arr[2][1])/elapsed_time
        return x_velo, z_velo


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = camera_data_reader()

    rclpy.spin(camera_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


