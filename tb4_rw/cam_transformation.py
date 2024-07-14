import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import tf2_py
import tf2_msgs.msg
import asyncio
import time
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy
import transforms3d.euler as euler

np.set_printoptions(suppress=True, floatmode='fixed')

callback_counter = 1
transform_matrix = [[], [], [], [], [], []]
full_matrix = np.array([])
odom_matrix = np.array([])

class TFListenerNode(Node):

    # static transform data
    base_link__shell_link = [[0., 0., 0.0942], [0., 0., 0., 1.]]
    shell_link__oakd_camera_bracket = [[-0.118, 0., 0.05257], [0., 0., 0., 1.]]
    oakd_camera_bracket__oakd_link = [[0.0584, 0., 0.9676], [0., 0., 0., 1.]]
    oakd_link__oakd_rgb_camera_frame = [[0., 0., 0.], [0., 0., 0., 1.]]
    oakd_rgb_camera_frame__oakd_rgb_camera_optical_frame = [[0., 0., 0.], [0.5, -0.5, 0.5, -0.5]]

    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("start!#@###!")
        
        qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to the /odom topic to get transformation related to odom 
        self.tf_subscriber = self.create_subscription(
            Odometry,
            '/arches/odom',
            self.do_odom_transform_callback,
            qos_profile)

        # Subscribe to the /obstacle_info topic to get coord of obstacle in camera frame
        self.tf_subscriber = self.create_subscription(
            Float64MultiArray,
            '/obstacle_info',
            self.get_obstacle_info_callback,
            10)

        # publisher for odom_obstacle_info
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/odom_obstacle_info',
            10)
        timer_period = 0.3 # sec
        self.timer = self.create_timer(timer_period, self.odom_publisher_callback)

    def odom_publisher_callback(self):
        global odom_matrix
        msg = Float64MultiArray()
        # if np.size(odom_matrix) != 0:
        #     print('odom_matrix[0]=', odom_matrix[0][0])
        if np.size(odom_matrix) != 0:
            msg.data = [odom_matrix[0][0], odom_matrix[1][0], odom_matrix[2][0]]
            self.publisher_.publish(msg)
        # else:
        #     msg.data = [1e6, 1e6, 1e6]   # use large numbner to indicate "None"
        #     self.publisher_.publish(msg)

    def get_obstacle_info_callback(self, msg):
        global full_matrix, odom_matrix
        camera_matrix = np.array([[msg.data[0]],
                                  [msg.data[1]],
                                  [msg.data[2]],
                                  [1]])
        # print('data get from obstacle_info topic:', camera_matrix)
        # print('full matrix:', full_matrix)

        # wait for full_matrix be assigned some values
        if np.size(full_matrix) != 0:
            odom_matrix = np.matmul(full_matrix, camera_matrix) # if "None", camera matrix will be very large
            print('obstacle coord in odom:\n', odom_matrix)
            print()

    def do_odom_transform_callback(self, msg):
        global callback_counter, transform_matrix, full_matrix
        # Process the TF data here
        # You can access the transformations using msg.transforms

        # print('callbackcounter:', callback_counter)
        if callback_counter == 1:
            # get all static transform matrices (only do it at first callback)
            transform_matrix[4], transform_matrix[3], transform_matrix[2], transform_matrix[1], transform_matrix[0] = self.do_static_transform()

        # # Wait for 2 seconds to give tf_buffer some time to fill up
        # self.get_logger().info("Received TF data. Waiting for 2 seconds to process.")
        # time.sleep(2.0)

        # print('source: odom')
        # print('child: base_link')

        # 5
        translation = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        transform_matrix[5] = self.odom_get_transform_matrix(translation, rotation)
        # print(transform_matrix[5])
        # print()

        # get full transform matrix here
        full_matrix = self.get_full_transform_matrix(transform_matrix[5], transform_matrix[4],
                                                     transform_matrix[3], transform_matrix[2],
                                                     transform_matrix[1], transform_matrix[0])

        # print('full matrix:')
        # print(full_matrix)
        # print()

        # get the odom data here for clarifying initial position
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                    )
        euler_angles = euler.quat2euler(quaternion, 'sxyz')
        robot_theta = euler_angles[0]  # get the "yaw" data
       # print('robot coord in odom:\n', robot_x)
       # print(robot_y)
       # print(robot_theta/np.pi, 'pi')
       # print()

        callback_counter += 1

    def do_static_transform(self):
        # print('now do static transform...')

        # 4
        translation = self.base_link__shell_link[0]
        rotation = self.base_link__shell_link[1]
        transform_matrix_4 = self.static_get_transform_matrix(translation, rotation)
        # print('base_link => shell_link')
        # print(transform_matrix_4)

        # 3
        translation = self.shell_link__oakd_camera_bracket[0]
        rotation = self.shell_link__oakd_camera_bracket[1]
        transform_matrix_3 = self.static_get_transform_matrix(translation, rotation)
        # print('shell_link => oakd_camera_bracket')
        # print(transform_matrix_3)

        # 2
        translation = self.oakd_camera_bracket__oakd_link[0]
        rotation = self.oakd_camera_bracket__oakd_link[1]
        transform_matrix_2 = self.static_get_transform_matrix(translation, rotation)
        # print('oakd_camera_bracket => oakd_link')
        # print(transform_matrix_2)

        # 1
        translation = self.oakd_link__oakd_rgb_camera_frame[0]
        rotation = self.oakd_link__oakd_rgb_camera_frame[1]
        transform_matrix_1 = self.static_get_transform_matrix(translation, rotation)
        # print('oakd_link => oakd_rgb_camera_frame')
        # print(transform_matrix_1)

        # 0
        translation = self.oakd_rgb_camera_frame__oakd_rgb_camera_optical_frame[0]
        rotation = self.oakd_rgb_camera_frame__oakd_rgb_camera_optical_frame[1]
        transform_matrix_0 = self.static_get_transform_matrix(translation, rotation)
        # print('oakd_rgb_camera_frame => oakd_rgb_camera_optical_frame')
        # print(transform_matrix_0)

        print()
        return transform_matrix_4, transform_matrix_3, transform_matrix_2, transform_matrix_1, transform_matrix_0

    def odom_get_transform_matrix(self, translation ,rotation):
        # print('translation x:', translation.x)
        # print('translation y:', translation.y)
        # print('translation z:', translation.z)

        # print('rotation x:', rotation.x)
        # print('rotation y:', rotation.y)
        # print('rotation z:', rotation.z)
        # print('rotation w:', rotation.w)

        # convert rotation data into matrix
        r = R.from_quat([rotation.x, rotation.y,
                         rotation.z, rotation.w])
        rotation_matrix = r.as_matrix()

        # create new ones column
        zero_row = np.zeros((1, 3))
        transform_matrix = np.vstack((rotation_matrix, zero_row))

        # create new zero row
        ones_column = np.ones((4, 1))
        transform_matrix = np.hstack((transform_matrix, ones_column))

        transform_matrix[0][3] = translation.x
        transform_matrix[1][3] = translation.y
        transform_matrix[2][3] = translation.z

        return transform_matrix

    def static_get_transform_matrix(self, translation, rotation):
        # print('translation x:', translation[0])
        # print('translation y:', translation[1])
        # print('translation z:', translation[2])

        # print('rotation x:', rotation[0])
        # print('rotation y:', rotation[1])
        # print('rotation z:', rotation[2])
        # print('rotation w:', rotation[3])

        # convert rotation data into matrix
        r = R.from_quat([rotation[0], rotation[1],
                         rotation[2], rotation[3]])
        rotation_matrix = r.as_matrix()

        # create new ones column
        zero_row = np.zeros((1, 3))
        transform_matrix = np.vstack((rotation_matrix, zero_row))

        # create new zero row
        ones_column = np.ones((4, 1))
        transform_matrix = np.hstack((transform_matrix, ones_column))

        transform_matrix[0][3] = translation[0]
        transform_matrix[1][3] = translation[1]
        transform_matrix[2][3] = translation[2]

        return transform_matrix

    def get_full_transform_matrix(self, transform_matrix_5, transform_matrix_4, transform_matrix_3,
                                  transform_matrix_2, transform_matrix_1, transform_matrix_0):
        _543_matrix = np.matmul(np.matmul(transform_matrix_5,transform_matrix_4), transform_matrix_3)
        _210_matrix = np.matmul(np.matmul(transform_matrix_2,transform_matrix_1), transform_matrix_0)
        # _210_matrix = np.matmul(transform_matrix_2,transform_matrix_1)
        full_matrix = np.matmul(_543_matrix, _210_matrix)
        return full_matrix

def main(args=None):
    rclpy.init(args=args)

    tf_listener_node = TFListenerNode()
    rclpy.spin(tf_listener_node)
    tf_listener_node.destroy_node()
    rclpy.shutdown()


