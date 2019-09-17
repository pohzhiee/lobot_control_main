
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    main_package_name = 'lobot_control_main'
    main_exec_name = 'lobot_control_exec'

    main_exec = Node(package=main_package_name,
                     node_executable=main_exec_name, output='screen')
    return LaunchDescription([main_exec])
