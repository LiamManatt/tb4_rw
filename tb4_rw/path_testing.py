#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import transforms3d.euler as euler
import sys

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.append("/home/ubuntu/ros2_ws/src/tb4_rw/tb4_rw")
import pid_controller as ctr

goal_pose_arr = np.zeros((1, 3))
goal_pose_arr_rel = np.zeros((1, 3))
state_arr = np.zeros((2, 3))
state_arr_rel = np.zeros((1, 3))
diff_arr = np.zeros((1, 3))
angle_to_goal = 0
velocity = 0.0
omega = 0.0
waypoint_number = 0
new_start = 0
iteration = 0 #record the index   

last_error = {'x': 0, 'y': 0, 'theta': 0}
integral = {'x': 0, 'y': 0, 'theta': 0}
dt = 0.3

class TurtleBotTrajectories(Node):
    
    def __init__(self):
        super().__init__("goal_waypoints")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/arches/cmd_vel", 10)

        qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.pose_subscriber_ = self.create_subscription(Odometry, "/arches/odom", self.controller_calc, qos_profile)
        
        self.get_logger().info("TurtleBot 4 should start moving toward goal pose/orientation through waypoints.")
        self.timer_ = self.create_timer(0.3, self.drive_turtle)
        self.timer2_ = self.create_timer(1.5, self.display_states)


    #def send_moveforward_command(self):
     #   msg = Twist()
      #  msg.angular.z = 0.0
       # msg.linear.x = 0.1
        #self.cmd_vel_pub_.publish(msg)

    #def pose_callback(self, msg: Odometry):
     #   self.get_logger().info("(" +str(msg.pose.pose.position.x) + ", " +str(msg.pose.pose.position.y) +")")


    def controller_calc(self, state):
        global iteration, state_arr, diff_arr, goal_pose_arr, velocity, omega, goal_pose_arr_rel, state_arr_rel, waypoint_number, goal_pose_arr_new, new_start, last_error, dt, integral

        state_arr[0][0] = state.pose.pose.position.x
        state_arr[0][1] = state.pose.pose.position.y
        quaternion = (state.pose.pose.orientation.x,
                      state.pose.pose.orientation.y,
                      state.pose.pose.orientation.z,
                      state.pose.pose.orientation.w
                     )
        euler_angles = euler.quat2euler(quaternion, 'sxyz')
        state_arr[0][2] = euler_angles[0]  # get the "yaw" data

        # store initial state
        if iteration == 0:
            for i in range(3):
                state_arr[1][i] = state_arr[0][i]
            iteration += 1
        
        # store new goal waypoint pose 
        if new_start == 0:
            for i in range(3):
                goal_pose_arr_rel[0][i] = goal_pose_arr_new[waypoint_number][i] + state_arr[1][i]
            if (waypoint_number + 1) < goal_pose_arr_new.shape[0]:
                print(f"Now going to waypoint #{waypoint_number + 1} \n ")
            elif (waypoint_number + 1) == goal_pose_arr_new.shape[0]:
                print("Now going to end goal destination \n ")
            new_start += 1

        # calculate the difference
        for j in range(3):
            diff_arr[0][j] = goal_pose_arr_rel[0][j] - state_arr[0][j]
            state_arr_rel[0][j] = state_arr[0][j] - state_arr[1][j]
        
        angle_to_goal = np.arctan2(diff_arr[0][1], diff_arr[0][0])
        diff_arr[0,2] = angle_to_goal - state_arr[0,2]

        # if at final goal coordinates and oriented correct, DONE
        if np.sqrt((state_arr[0][0]-goal_pose_arr_rel[0][0])**2 + (state_arr[0][1]-goal_pose_arr_rel[0][1])**2) < 0.1 and (waypoint_number + 1) == goal_pose_arr_new.shape[0]: #and abs(diff_arr[0][2]) <= 0.1:    
            print("Goal destination reached!? \n")
            # stop the robot when the goal destination is reached (after going through waypoints)
            velocity = 0
            omega = 0
            quit()
        elif np.sqrt((state_arr[0,0]-goal_pose_arr_rel[0,0])**2 + (state_arr[0,1]-goal_pose_arr_rel[0,1])**2) < 0.1 and (waypoint_number + 1) < goal_pose_arr_new.shape[0]:
            print(f"waypoint #{waypoint_number + 1} reached! \n")
            new_start = 0
            waypoint_number += 1
        else:
            velocity, omega, last_error, integral = ctr.pid_controller(error_x = diff_arr[0,0], error_y = diff_arr[0,1], error_theta = diff_arr[0,2], last_error = last_error, integral = integral, dt = dt)

        

    def display_states(self):
        global state_arr_rel, waypoint_number, goal_pose_arr, velocity, omega, diff_arr, goal_pose_arr_new, angle_to_goal
        print("state: ", state_arr_rel[0])
        if (waypoint_number + 1) < goal_pose_arr_new.shape[0]:
            print(f" adjustment required to waypoint #{waypoint_number + 1}: {diff_arr[0][0]} {diff_arr[0][1]}")
        else:
            print(" adjustment required to final destination: ", diff_arr[0])
        print(" velocity: ", velocity , " omega: ", omega)
        print(" angle to goal: ", angle_to_goal)
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
    global goal_pose_arr, state_arr, diff_arr, goal_pose_arr_rel, state_arr_rel, goal_pose_arr_new 

    #user input for goal
    goal_pose_arr[0][0] = float(input("Assume start position is (0,0) and going +y is forwards \n goal x: "))
    goal_pose_arr[0][1] = float(input(" goal y: "))
    goal_pose_arr[0][2] = float(input(" goal theta: "))

    #user inputs for waypoints
    wp_n = int(input("How many waypoints would you like to add? "))
    waypoints_arr = np.zeros((wp_n, 3))
    for i in range(wp_n):
        waypoints_arr[i][0] = float(input(f"Enter desired (x,y) for waypoint #{i+1} \n desired x: "))
        waypoints_arr[i][1] = float(input(" desired y: "))
    
    # shift "goal" to end of array so that robot goes to waypoints first
    goal_pose_arr_new = np.concatenate((waypoints_arr, goal_pose_arr), axis=0)


    rclpy.init(args=args)
    node = TurtleBotTrajectories()
    rclpy.spin(node)
    rclpy.shutdown()



