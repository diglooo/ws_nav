import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    
    robot_params_dir=os.path.join(get_package_share_directory('minolo'),'params')
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    
    #Load URDF into memory
    urdf_file = os.path.join(get_package_share_directory('minolo'), 'urdf', "robot.urdf")   
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    launch_nav2=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
            launch_arguments={'params_file': os.path.join(robot_params_dir, "nav2_params.yaml")}.items(),
            )
    launch_amcl=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/localization_launch.py']),
            launch_arguments={'params_file': os.path.join(robot_params_dir, "nav2_params.yaml"),'map':'/home/diglo/ws_nav/src/maps/map_1737995541.yaml'}.items(),
            )

    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
        launch_arguments={'slam_params_file': os.path.join(robot_params_dir, "slam_params.yaml")}.items(),
        
        )

    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(robot_params_dir, 'ydlidar-X4.yaml')],
        namespace='/')
        
    robot_state_publisher_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'rate': 20, }])
    
    radio_teleop_receiver = Node(
         package='teleop_receiver',
         executable='receiver',
         name='radio_teleop_receiver',
         output='screen',
         parameters=[{
                    'serial_port' : '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_709a94ae5324ed11926c94e8f9a97352-if00-port0',
                    'baudrate' : 115200,
                    'velocity_topic' : '/cmd_vel',
                    }])

    imu_receiver = Node(
         package='imu_receiver',
         executable='receiver',
         name='imu_receiver',
         output='screen',
         parameters=[{
                    'serial_port' : '/dev/ttyACM0',
                    'baudrate' : 115200,
                    'output_topic' : '/imu_data',
                    'accel_sensitivity': 2.0,
                    'gyro_sensitivity':500.0
                    }])

    motor_control_node=Node(
        package='minolo',
        executable='motor_controller',
        name='minolo_motor_controller',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(robot_params_dir, 'motor_controller_params.yaml')])
    
    motor_interface_node=Node(
        package='minolo',
        executable='motor_interface',
        name='minolo_motor_interface',
        output='screen',
        parameters=[
            os.path.join(robot_params_dir, 'motor_interface_params.yaml')])
    
    lidar_odometry=Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='laser_odometry_node',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_lidar',
            'publish_tf' : False,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 30.0}],)
    
    localization_node=Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_params_dir, 'state_estimation_params.yaml')],       
    )

    topic_mux = Node(
        package='twist_mux',
        executable="twist_mux",
        name="twist_mux",
        parameters=[os.path.join(robot_params_dir,'twist_mux.yaml')],
        remappings={
            ('/cmd_vel_out', '/cmd_vel_muxed'),
        },
        output='screen',)
    
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')
    
    return LaunchDescription([
        motor_control_node,
        motor_interface_node,
        robot_state_publisher_node,
        radio_teleop_receiver,
        imu_receiver,
        lidar_node,
        launch_nav2,
        launch_amcl,
        #launch_slam,
        localization_node
   ])
