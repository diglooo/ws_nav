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
         executable='teleop_receiver',
         name='radio_teleop_receiver',
         output='screen',
         parameters=[{
                    'serial_port' : '/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1:1.0-port0',
                    'baudrate' : 115200,
                    'velocity_topic' : '/cmd_vel',
                    }])

    imu_receiver = Node(
         package='imu_receiver',
         executable='imu_receiver',
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
        parameters=[
            os.path.join(robot_params_dir, 'motor_controller_params.yaml')])
    
    motor_interface_node=Node(
        package='minolo',
        executable='motor_interface',
        name='minolo_motor_interface',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.3:1.0-port0',
            'baudrate': 115200,
            'autozero_vel_timeout_seconds': 0.1
        }])
    
    localization_node=Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_params_dir, 'state_estimation_params.yaml')],       
    )
    
    rl_params_file = os.path.join(get_package_share_directory('minolo'), 'params', "navsat_robot_localization.yaml")   
    navsat_transform=Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[rl_params_file],
        remappings=[
            ("imu_data", "imu_data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    global_localization=Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[rl_params_file],
        remappings=[
            ("odometry/filtered", "odometry/global"),
        ],
    )
    
    local_localization=Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[rl_params_file],
        remappings=[
            ("odometry/filtered", "odometry/local"),
        ],
    )
    
    gps_node=Node(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.4:1.0-port0',
            'baudrate': 115200,
            'caster_host': "euref-ip.asi.it",
            'caster_port': 2101,
            'mountpoint': "GENO00ITA0",
            'username': "ddigloria",
            'password': "cogo-2023",
        }])
    
    initialize_origin=Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        remappings=[
            ("fix", "gps/fix"),
        ])

    swri_transform=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf2_ros",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )

    map_odom_transform=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf2_ros",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
    
    return LaunchDescription([
        #motor_control_node,
        #motor_interface_node,
        robot_state_publisher_node,
        #radio_teleop_receiver,
        imu_receiver,
        #lidar_node,
        #launch_nav2,
        #launch_amcl,
        #launch_slam,
        #localization_node,
        gps_node,
        #navsat_transform,
        #initialize_origin,
        global_localization,
        #local_localization,
        #map_odom_transform,
        #global_localization
   ])
