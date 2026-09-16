from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    driver_share = get_package_share_directory('gl5_driver')
    detection_share = get_package_share_directory('gl5_detection')
    bringup_share = get_package_share_directory('gl5_bringup')
    driver = Node(package='gl5_driver', executable='gl5_node', name='gl5_node',
                  parameters=[LaunchConfiguration('params_file')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(driver_share, 'config', 'gl5.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('obstacles', default_value='true'),
        DeclareLaunchArgument('obstacle_params_file', default_value=os.environ.get(
            'GL5_OBSTACLE_PARAMS_FILE', os.path.join(detection_share, 'config', 'obstacles.yaml'))),
        DeclareLaunchArgument('region_file', default_value=os.environ.get(
            'GL5_REGION_FILE', os.path.join(os.getcwd(), 'gl5_region.json'))),
        driver,
        Node(package='gl5_detection', executable='gl5_obstacle_node',
             name='gl5_obstacle_detector', output='screen',
             parameters=[LaunchConfiguration('obstacle_params_file'),
                         {'region_file': LaunchConfiguration('region_file')}],
             condition=IfCondition(LaunchConfiguration('obstacles'))),
        RegisterEventHandler(OnProcessExit(target_action=driver,
            on_exit=[EmitEvent(event=Shutdown(reason='GL5 driver exited'))])),
        Node(package='rviz2', executable='rviz2', name='gl5_rviz', output='screen',
             arguments=['-d', os.path.join(bringup_share, 'rviz', 'gl5.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
