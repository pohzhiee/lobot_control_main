import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkgShareDir  = get_package_share_directory('lobot_control_main')
    configDir = os.path.join(pkgShareDir, 'config', 'params.yaml')
    node1 = launch_ros.actions.Node(
        package='my_parameter_server', node_executable='param_server_exec', arguments=[configDir], output='screen')
    return launch.LaunchDescription([node1])
