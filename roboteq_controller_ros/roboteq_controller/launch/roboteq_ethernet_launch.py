import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lds = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('roboteq_controller'),
        'config',
        'query_ethernet.yaml'
        )
        
    enode=Node(
        package = 'roboteq_controller',
        name = 'roboteq_ethernet_controller_node',
        executable = 'roboteq_ethernet_controller_node',
        parameters = [config]
    )

    lds.add_action(enode)

    return lds