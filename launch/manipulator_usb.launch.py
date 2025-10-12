import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import xacro


def generate_launch_description():
    
    package_name='manipulator_controller' #<--- CHANGE ME


    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))

    control = Node(
            package=package_name,
            executable='manipulator_controller',
            name='manipulator_controller',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'manipulator_controller_usb.yaml')],
    )

    # Launch!
    return LaunchDescription([
        control

    ])