#!/usr/bin/env python3

import rclpy
import time
import threading
import numpy as np
import transforms3d.euler as euler
from rclpy.node import Node
import sys

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.append("/home/ubuntu/ros2_ws/src/tb4_rw/tb4_rw")
# import other function file
import pid_controller as ctr
import create_BezierCurve as bcurve
#import calculate_slope
import RRT_star as rrts
#import move_next_position

np.set_printoptions(suppress=True, floatmode='fixed')

# obstacle information
obstacle_state_arr = [[1e6, 1e6], [1e6, 1e6]]  # in odom frame
#(new)
initial_obstacle_position = [1e6, 1e6, 1e6]
current_obstacle_position = initial_obstacle_position
#obstacle_old_position = [None, None]
#obstacle_velocity = 0.
obstacle_radius = 0.1    # (R) meter
#obstacle_x_velo = obstacle_velocity* np.cos(current_obstacle_position[2])
#obstacle_y_velo = obstacle_velocity* np.sin(current_obstacle_position[2])
obstacle_temp_list = [[1e6, 1e6, 1e6], [1e6, 1e6, 1e6]]  # list for getting the velo. of obstacle

# robot information
robot_radius = 0.17    # (r) meter
# initial_robot_position = [249., 250., 1.25*np.pi]    # [x, y, theta]
# current_robot_position = initial_robot_position
robot_velocity = 0.0
robot_omega = 0.0
robot_x_velo = 0.0
robot_y_velo = 0.0
#see_obstacle = 0

integral = {'x': 0, 'y': 0, 'theta': 0}
last_error = {'x': 0, 'y': 0, 'theta': 0}
dt = 0.3

temp_target = [0., 0., 0.]
target = [0., 0., 0.]
state_arr = np.zeros((2, 3))  # robot state
diff_arr = [0., 0., 0.]
angle_to_goal = 0
controller_index = 0

safety_margin = 5*robot_radius

dodge = 0
#nearby = 0

iteration = 1                        # record the index
iteration_since_detect = 0              # number of iteration since robot detect obstacle

elapsed_time = 0

security_width = obstacle_radius + 2.5*robot_radius # the additional width around the obstacle for safety 

distance_proceed = 0.

last_callback_time = None
doneBezier = False

# Bezier curve parameters
smooth_data = []
smooth_theta = 0.
smooth_data_index = 0


class odom_data_subscriber(Node):

    def __init__(self):
        super().__init__("odom_data_subscriber")

        # publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/'+turtlebotname+'/cmd_vel', 10)
        self.get_logger().info("start^^))!")

        qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # subscriber for odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/'+turtlebotname+'/odom',
            self.odom_callback,
            qos_profile
        )

        # subscriber for odom_obstacle_info
        self.subscription_obstacle_info = self.create_subscription(
            Float64MultiArray,
            "/odom_obstacle_info",
            self.odom_obstacle_info_callback,
            10
        )




        self.timer2_ = self.create_timer(2, self.display_states)

    def start_obstacle_callback_thread(self):
        obstacle_thread = threading.Thread(target=self.odom_obstacle_info_callback)
        obstacle_thread.start()

    def odom_obstacle_info_callback(self, msg):
        global security_width, obstacle_temp_list, last_callback_time, dodge
        global iteration_since_detect, current_obstacle_position

        # store the old data
        for i in range(3):
            obstacle_temp_list[1][i] = obstacle_temp_list[0][i]
            #  old one                    #  new one
        print(msg.data)
        # recording data from /odom_obstacle_info as new data
       
        if msg.data != [0.,0.,0.] or (np.abs(msg.data[0]) < 1e3 and np.abs(msg.data[1]) < 1e3 and np.abs(msg.data[2]) < 1e3):
            for j in range(3):
            # print('storing obstacle data...')
                obstacle_temp_list[0][j] = msg.data[j]
        else:
            for j in range(3):
                obstacle_temp_list[0][j] = 1e6

        if obstacle_temp_list[0][0]-obstacle_temp_list[1][0] != 0:
            obstacle_slope = (obstacle_temp_list[0][1]-obstacle_temp_list[1][1])/(obstacle_temp_list[0][0]-obstacle_temp_list[1][0])
            current_obstacle_position = [obstacle_temp_list[0][0], obstacle_temp_list[0][1], np.arctan(obstacle_slope)]
        else:
            current_obstacle_position = [obstacle_temp_list[0][0], obstacle_temp_list[0][1], 0.]
        
        #print(current_obstacle_position)
        #print()

        # if camera see "None", don't dodge
        if np.abs(msg.data[0]) > 1e4 and np.abs(msg.data[1]) > 1e4 and np.abs(msg.data[2]) > 1e4:
            dodge = 0

    def odom_callback(self, msg):
        global iteration, state_arr, diff_arr, target, robot_velocity, robot_omega, doneBezier, integral, last_error, dt, angle_to_goal
        global obstacle_state_arr, controller_index, dodge, last_callback_time #nearby, see_obstacle
        global iteration_since_detect, current_obstacle_position, target, distance_proceed
        global smooth_data, smooth_theta, smooth_data_index

        elapsed_time = 0                    # for getting time duration between each callback
        callback_start_time = time.time()
        if last_callback_time is not None:
            elapsed_time = callback_start_time - last_callback_time
            # print('odom_obstacle_callback elapse time:', elapsed_time)
        last_callback_time = callback_start_time

        # store the old data
        for i in range(3):
            state_arr[1][i] = state_arr[0][i]
            #  old one         #  new one

        # recording data from /odom as new data (the unit in /odom is SI units)
        state_arr[0][0] = msg.pose.pose.position.x
        state_arr[0][1] = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                    )
        euler_angles = euler.quat2euler(quaternion, 'sxyz')
        state_arr[0][2] = euler_angles[0]  # get the "yaw" data

        # set the size of the map of RRT* #
        # print('robot orientation=', state_arr[0][2]/np.pi, 'pi')
        #if np.cos(state_arr[0][2]) > np.sin(state_arr[0][2]:
        rrts.RRTStar.x_limit = (state_arr[0][0]+2*distance_proceed, state_arr[0][0]-2*distance_proceed)
        rrts.RRTStar.y_limit = (state_arr[0][1]+2*distance_proceed, state_arr[0][1]-2*distance_proceed)
        #else:
         #   rrts.RRTStar.x_limit = (state_arr[0][0]+2*distance_proceed*np.sin(state_arr[0][2]), state_arr[0][0])
         #   rrts.RRTStar.y_limit = (state_arr[0][1]+2*distance_proceed*np.sin(state_arr[0][2]), state_arr[0][1])
        # set the size of the map of RRT* #

        # set the target and start driving
        if iteration == 1:
            target[0] = state_arr[0][0] + distance_proceed*np.cos(state_arr[0][2])
            target[1] = state_arr[0][1] + distance_proceed*np.sin(state_arr[0][2])
            target[2] = -1*np.pi   # here I set the constant target orientation
            print('target:', target)

            # robot can only drive after the target is calculated
            driving_thread = threading.Thread(target = self.drive)
            driving_thread.start()
            print()
            print('start driving...')
            print()

        # print the position of obstacle every 50 iterations
        # if iteration%50 == 1:
        #     print('current_obstacle_position=', 
        #           current_obstacle_position[0], ',', 
        #           current_obstacle_position[1], ',', 
        #           current_obstacle_position[2]/np.pi, 'pi')






        if np.abs(current_obstacle_position[0]) < 10000:
            distance_to_obstacle = np.sqrt((current_obstacle_position[0]-state_arr[0][0])**2+(current_obstacle_position[1]-state_arr[0][1])**2)
            # print('current_obstacle_position=', current_obstacle_position[0], ',', current_obstacle_position[1])
            if iteration%50 == 0:
                print('distance to obstacle:', distance_to_obstacle)
                pass
            # print('min. dist. need to dodge the obstacle:', obstacle_radius + robot_radius + safety_margin)
            # print('dodge:', dodge)
            # print('iteration:', iteration)
        else:
            distance_to_obstacle = 1000000   # large number to indicate there is no obstacle in front
            if iteration%50 == 0:

                print('No obstacle in front...')
                pass
            # print('iteration:', iteration)

        # check if robot near obstacle, if true:
        if ((distance_to_obstacle <= obstacle_radius + robot_radius + safety_margin) and (dodge == 0)):
            print("iteration:", iteration, "obstacle close to robot!!")
            #nearby = 1
            #see_obstacle = 1



            controller_index = 0
            dodge = 1
            robot_velocity = 0
            robot_omega = 0
        else:
            # The control to final destination
            # calculate the difference to final destination
            # print('now control to the target...')
            for j in range(2):
                diff_arr[j] = target[j] - state_arr[0,j]
            angle_to_goal = np.arctan2(diff_arr[1],diff_arr[0])
            diff_arr[2] = angle_to_goal - state_arr[0,2]

            # print('diff_arr to target:', diff_arr)
            #### new controller
            #robot_velocity, robot_omega = ctr.controller(error_x=diff_arr[0], error_y=diff_arr[1], error_theta=diff_arr[2],
            #                                                iteration=iteration, initial_velocity=robot_velocity, initial_omega=robot_omega,
            #                                                dodge=dodge, last_error = last_error, integral = integral, dt = dt)
            robot_velocity, robot_omega, last_error, integral = ctr.pid_controller(error_x=diff_arr[0], error_y=diff_arr[1], error_theta=diff_arr[2], last_error = last_error, integral=integral, dt=dt)
            #print('rbt velo, rbt omega = ', robot_velocity, ',', robot_omega)
            # escape the callback function
            # print("state: ", state_arr[0])
            # print('target:', target)
            # print()

            # count iteration only if the robot starts moving
            if np.abs(state_arr
[1][0] - state_arr[0][0]) > 0.00001 or np.abs(state_arr[1][1] - state_arr[0][1]) > 0.00001:
                iteration += 1

            # return

        # predict obstacle trajectory only if robot starts dodging and at particular # of iteration
        if dodge:
            if iteration_since_detect%30 == 0:
                doneBezier = False
                print('now appending obstacle list...')
                rrts.RRTStar.obstacle_list = []   # clear the list every time come in this function
                # current_obstacle_position = [msg.data[0], msg.data[1], 0.]            # the last element should be orientation !!!
                # print('data from odom_obstacle_info', current_obstacle_position)
                rrts.RRTStar.obstacle_list.append([current_obstacle_position[0],
                                        current_obstacle_position[1],
                                        security_width])
                # print('append list:\n', current_obstacle_position[0], ',', current_obstacle_position[1])
                # print('data in obstacle_list:', rrts.RRTStar.obstacle_list)

                # get obstacle velo
                if elapsed_time != 0:
                    obstacle_x_velo = (obstacle_temp_list[0][0]-obstacle_temp_list[1][0])/elapsed_time
                    obstacle_y_velo = (obstacle_temp_list[0][1]-obstacle_temp_list[1][1])/elapsed_time
                    print('obstacle x velo:', obstacle_x_velo, 'obstacle y velo:', obstacle_y_velo)




                    # print('now robot postion:', state_arr[0][0], ',',state_arr[0][1])
                    # do obstacle trajectory prediction
                    predicted_length = 10
                    print('obstacle list:')
                    for i in range(predicted_length):
                        obstacle_list_x = current_obstacle_position[0]+obstacle_x_velo*i*elapsed_time
                        obstacle_list_y = current_obstacle_position[1]+obstacle_x_velo*i*elapsed_time
                        rrts.RRTStar.obstacle_list.append([obstacle_list_x, obstacle_list_y, security_width*1])   # (1-(i/predicted_length))
                        print('(' ,obstacle_list_x, ',', obstacle_list_y, ',', security_width*(1-(i/predicted_length)), '),')
                        if np.sqrt((target[0]-obstacle_list_x)**2 + (target[1]-obstacle_list_y)**2) < security_width*(1-(i/predicted_length)):
                            print('target is in the prohibited zone!!!')

                print()
                print('RRT* map size:')
                print('x_limit=', rrts.RRTStar.x_limit)
                print('y_limit=', rrts.RRTStar.y_limit)
                # set RRT* starting point and goal
                rrts.RRTStar.start = (state_arr[0][0], state_arr[0][1])
                rrts.RRTStar.goal = (target[0], target[1])
                print('start=', rrts.RRTStar.start)
                print('goal=', rrts.RRTStar.goal)
                # RRT*
                print('RRT* Path Planning...')
                print()

                # when doing RRT*, stop the robot
                robot_velocity = 0
                robot_omega = 0

                path = rrts.RRTStar.do_RRTstar(start=rrts.RRTStar.start,
                                            goal=rrts.RRTStar.goal,
                                            obstacle_list=rrts.RRTStar.obstacle_list,    # obstacle list need the data from camera
                                            x_limit=rrts.RRTStar.x_limit,
                                            y_limit=rrts.RRTStar.y_limit,
                                            step_size=rrts.RRTStar.step_size,
                                            max_iterations=rrts.RRTStar.max_iterations,
                                            current_robot_position=state_arr
                                        )   

                # Create Bezier Curve
                smooth_data, smooth_theta, x_smooth_reversed, y_smooth_reversed = bcurve.create_BezierCurve(path)
                controller_index = 0
                smooth_data_index = 200      # record which point is temporary target
                smooth_data_increase = smooth_data_index
                doneBezier = True
                print('\nDone Bezier Curve...\n')

            if doneBezier:
                # get the difference
                for j in range(2):
                    diff_arr[j] = smooth_data[smooth_data_index][j] - state_arr[0,j]
                diff_arr[2] = smooth_theta[smooth_data_index] - state_arr[0,2]

                # orientation adjustment if needed
                if np.abs(diff_arr[2]) > np.pi/6:
                    self.turn(turning_index=2)

                #### new controller
                #robot_velocity, robot_omega = ctr.pid_controller(error_x=diff_arr[0], error_y=diff_arr[1], error_theta=diff_arr[2],
                 #                                           iteration=iteration, initial_velocity=robot_velocity, initial_omega=robot_omega,
                 #                                           dodge=dodge, last_error = last_error, integral=integral, dt=dt)
                robot_velocity, robot_omega, last_error, integral = ctr.pid_controller(error_x=diff_arr[0], error_y=diff_arr[1], error_theta=diff_arr[2], last_error = last_error, integral=integral, dt=dt)
                if iteration % 30 == 0:
                    print('\nRRT* path control...\n')
                # print('iteration since detect:', iteration_since_detect, 'dodge:', dodge)
                iteration_since_detect += 1    ######################

        # if the robot is close enough to destination
        if np.sqrt((state_arr[0][0]-target[0])**2 + (state_arr[0][1]-target[1])**2) < .2:
            dodge = 0
            for j in range(2):
                diff_arr[j] = target[j] - state_arr[0,j]
            angle_to_goal = np.arctan2(diff_arr[1],diff_arr[0])
            diff_arr[2] = angle_to_goal - state_arr[0,2]
            #### new controller
            #robot_velocity, robot_omega = ctr.pid_controller(error_x=diff_arr[0], error_y=diff_arr[1], error_theta=diff_arr[2],
             #                                               iteration=iteration, initial_velocity=robot_velocity, initial_omega=robot_omega,
             #                                               dodge=dodge, last_error=last_error, integral=integral, dt=dt)
            robot_velocity, robot_omega, last_error, integral = ctr.pid_controller(error_x=diff_arr[0], error_y=diff_arr[1], error_theta=diff_arr[2], last_error=last_error, integral=integral, dt=dt)
            # print("past state: ", state_arr[1])
            if iteration%50 == 0:
                print("state (ready to go to dist.): ", state_arr[0])
                # print('target:', target)
            if np.sqrt((state_arr[0][0]-target[0])**2 + (state_arr[0][1]-target[1])**2) < 0.1:
                print('target reached!')
                # stop the robot when the target is reached
                robot_velocity = 0
                robot_omega = 0
                quit()

        # count iteration only if the robot starts moving
        if np.abs(state_arr[1][0] - state_arr[0][0]) > 0.0001 or np.abs(state_arr[1][1] - state_arr[0][1]) > 0.0001:
            # print('iteration adding...')
            iteration += 1

    def display_states(self):
    #    print(dodge)
    #    print(obstacle_state_arr)
    #    print(current_obstacle_position)
        print("current difference array: ", diff_arr)
        #print("elapsed time: ", elapsed_time)
        
    def drive(self):
        # keep publishing the data to cmd_vel to drive the robot
        while rclpy.ok():
            global robot_velocity, robot_omega, diff_arr
            msg = Twist()

            if robot_velocity >= 0:
                msg.linear.x = float(robot_velocity)
                msg.angular.z = float(robot_omega)
                self.publisher_.publish(msg)
                time.sleep(0.3)
            else:
                # if velo < 0 => turn around
                self.turn(msg, turning_index=1)

    def turn(self, msg=Twist(), turning_index=1):
        # publish "turn around" to cmd_velocity, "turning_index" indicate the type of turn
        # turning_index 1 => turn around
        #               2 => orientation adjustment
        global state_arr, diff_arr, dodge, robot_velocity
        old_angle = state_arr[0][2]
        # print("old angle: ", old_angle)
        if turning_index == 1:
            # print('start turning around...')
            while np.abs(state_arr[0][2] - old_angle) < np.pi-1 or np.abs(state_arr[0][2] - old_angle) > np.pi+1:
                # print("diff.: ", np.abs(state_arr[0][2] - old_angle))
                msg.linear.x = float(0)
                msg.angular.z = float(1)
                self.publisher_.publish(msg)
                time.sleep(0.3)
                # if dodge:
                #     break
            # print('stop turning...')

        elif turning_index == 2:
            if diff_arr[2] > np.pi/6:
                while (np.abs(state_arr[0][2] - old_angle) > np.pi/6):
                    msg.linear.x = float(0)
                    msg.angular.z = float(-0.2)   # clockwise
                    self.publisher_.publish(msg)
                    time.sleep(0.3)
            elif diff_arr[2] < np.pi/6:
                while (np.abs(state_arr[0][2] - old_angle) > np.pi/6):
                    msg.linear.x = float(0)
                    msg.angular.z = float(0.2)    # counter-clockwise
                    self.publisher_.publish(msg)
                    time.sleep(0.3)


def main(args=None):
    global target, state_arr, diff_arr, distance_proceed, turtlebotname
    # user input
    turtlebotname = str(input("turtlebot in use: "))
    distance_proceed = float(input("desired proceed distance (m): "))

    rclpy.init(args=args)
    Odom_data_subscriber = odom_data_subscriber()

    # Start the obstacle callback thread
    Odom_data_subscriber.start_obstacle_callback_thread()

    # driving_thread = threading.Thread(target = Odom_data_subscriber.drive)
    # driving_thread.start()

    rclpy.spin(Odom_data_subscriber)

    Odom_data_subscriber.destroy_node()
    rclpy.shutdown()
