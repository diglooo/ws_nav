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
    
    diglobot2_param_dir=os.path.join(get_package_share_directory('minolo'),'params')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_toolbox_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    urdf_file = os.path.join(get_package_share_directory('minolo'), 'urdf', "robot.urdf")   


    #Load URDF into memory
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    launch_nav2=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
        launch_arguments={'params_file': os.path.join(diglobot2_param_dir, "nav2_params.yaml")}.items())
    
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_launch_file_dir, '/online_async_launch.py']))

    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(diglobot2_param_dir, 'ydlidar-X4.yaml')],
        namespace='/')
        
    robot_state_publisher_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}])
    
    motor_control_mode=Node(
        package='motor_interface',
        executable='velocity_writer',
        name='motor_interface',
        output='screen')
    
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    return LaunchDescription([
        motor_control_mode,
        lidar_node,
        robot_state_publisher_node,
        launch_nav2,
        launch_slam,
   ])
