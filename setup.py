from setuptools import setup

package_name = 'tb4_rw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "go_to_goal_og = tb4_rw.go_to_goal_og:main",
            "goal_waypoints = tb4_rw.goal_waypoints:main",
            "camera_data_publisher = tb4_rw.camera_data_publisher:main",
            "camera_reader_to_obstacle_info = tb4_rw.camera_reader_to_obstacle_info:main",
            "cam_transformation = tb4_rw.cam_transformation:main",
            "turtle_dodger_edit = tb4_rw.turtle_dodger_edit:main",
            "rw_turtle_dodger = tb4_rw.rw_turtle_dodger:main",
            "odom_data_subscriber_dodge = tb4_rw.odom_data_subscriber_dodge:main",
            "path_testing = tb4_rw.path_testing:main",
            "new_cam_data_publish = tb4_rw.new_cam_data_publish:main"
        ],
    },
)
