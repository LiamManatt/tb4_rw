#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import transforms3d.euler as euler

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_system_default


goal_pose_arr = np.zeros((1, 3))
goal_pose_arr_rel = np.zeros((1, 3))
state_arr = np.zeros((2, 3))
diff_arr = np.zeros((1, 3))
velocity = 0.0
omega = 0.0
iteration = 0 #record the index        

class TurtleBotTrajectories(Node):
    
    def __init__(self):
        super().__init__("go_to_goal_og")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/arches/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Odometry, "/arches/odom", self.controller_calc, qos_profile_system_default)
        self.get_logger().info("TurtleBot 4 should start moving toward goal pose/orientation.")
        self.timer_ = self.create_timer(0.5, self.drive_turtle)

    #def send_moveforward_command(self):
     #   msg = Twist()
      #  msg.angular.z = 0.0
       # msg.linear.x = 0.1
        #self.cmd_vel_pub_.publish(msg)

    #def pose_callback(self, msg: Odometry):
     #   self.get_logger().info("(" +str(msg.pose.pose.position.x) + ", " +str(msg.pose.pose.position.y) +")")

    def controller_calc(self, state):
        global iteration, state_arr, diff_arr, goal_pose_arr, velocity, omega, goal_pose_arr_rel

        state_arr[0][0] = state.pose.pose.position.x
        state_arr[0][1] = state.pose.pose.position.y
        quaternion = (state.pose.pose.orientation.x,
                      state.pose.pose.orientation.y,
                      state.pose.pose.orientation.z,
                      state.pose.pose.orientation.w
                     )
        euler_angles = euler.quat2euler(quaternion, 'sxyz')
        state_arr[0][2] = euler_angles[0]  # get the "yaw" data

        # store relative goal pose and initial state
        if iteration == 0:
            for i in range(3):
                state_arr[1][i] = state_arr[0][i]
                goal_pose_arr_rel[0][i] = goal_pose_arr[0][i] + state_arr[1][i]
            iteration += 1

        # calculate the difference
        for j in range(3):
            diff_arr[0][j] = state_arr[0][j] - goal_pose_arr_rel[0][j]
        
        angle_to_goal = np.arctan2(diff_arr[0][1],diff_arr[0][0])

        # turn robot until it's pointing at goal coordinates
        if abs(angle_to_goal - state_arr[0][2]) > 0.05:
            scale = 10
            dir = (angle_to_goal - state_arr[0][2])/(abs(angle_to_goal - state_arr[0][2]))
            angSpeed = min(0.05, abs((angle_to_goal - state_arr[0][2])/scale))
            omega = dir*angSpeed
            velocity = 0
        
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) >= 0.05 and abs(angle_to_goal - state_arr[0][2]) < 0.05:
            velocity = 0.05
        
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) < 0.05 and abs(angle_to_goal - state_arr[0][2]) <= 0.05:
            print('target position reached!?')
          # stop the robot when the target is reached
            velocity = 0
            omega = 0
            quit()


        print("state: ", state_arr[0] - state_arr[1])
        # print(" velocity: ", velocity , "omega: ", omega)
        print()


    def drive_turtle(self):
        # print("now sending driving command")
        global velocity, omega
        msg = Twist()

        msg.linear.x = float(velocity)
        msg.angular.z = float(omega)
        # print("published: ", msg.linear.x, msg.angular.z)
        self.cmd_vel_pub_.publish(msg)
        
    
    #def stop_turtle_command(self):
     #   msg = Twist()
      #  msg.angular.z = 0.0
       # msg.linear.x = 0.0
        #self.cmd_vel_pub_.publish(msg)

            

                


def main(args=None):
    global goal_pose_arr, state_arr, diff_arr, goal_pose_arr_rel

    #user input
    goal_pose_arr[0][0] = float(input("Assume start position is (0,0) and facing +y \n goal x: "))
    goal_pose_arr[0][1] = float(input(" goal y: "))
    goal_pose_arr[0][2] = float(input(" goal theta: "))

    rclpy.init(args=args)
    node = TurtleBotTrajectories()
    rclpy.spin(node)
    rclpy.shutdown()




