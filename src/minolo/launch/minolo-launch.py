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
    
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']))

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
        parameters=[{'robot_description': robot_description}])
    
    # motor_control_mode=Node(
    #     package='minolo',
    #     executable='minolo_bringup',
    #     name='minolo_motor_interface',
    #     output='screen')
    motor_control_node=Node(
        package='minolo',
        executable='motor_controller',
        name='minolo_motor_controller',
        output='screen',
        parameters=[os.path.join(robot_params_dir, 'motor_controller_params.yaml')])
    
    motor_interface_node=Node(
        package='minolo',
        executable='motor_interface',
        name='minolo_motor_interface',
        output='screen',
        parameters=[os.path.join(robot_params_dir, 'motor_interface_params.yaml')])
    
    lidar_odometry=Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='laser_odometry_node',
        output='screen',
        parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 5.0}],)
    
    localization_node=Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_params_dir, 'state_estimation_params.yaml')],       
    )
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    return LaunchDescription([
        motor_control_node,
        motor_interface_node,
        #lidar_node,
        #robot_state_publisher_node,
        #launch_nav2,
        #launch_slam,
        #lidar_odometry,
        #localization_node
   ])
