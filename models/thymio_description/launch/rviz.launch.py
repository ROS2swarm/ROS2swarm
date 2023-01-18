import os
from typing import List

import launch.actions

from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def rviz(context: LaunchContext) -> List[Node]:
    name = perform_substitutions(context, [launch.substitutions.LaunchConfiguration('name')])
    source_path = os.path.join(
        get_package_share_directory('thymio_description'), 'launch', 'template.rviz')
    with open(source_path, 'r') as f:
        source = f.read()
    if not name:
        dest = source.replace('/thymio', '')
    else:
        dest = source
    dest = dest.replace('thymio', name)
    dest_path = os.path.join(
        get_package_share_directory('thymio_description'), 'launch', 'thymio.rviz')
    with open(dest_path, 'w') as f:
        f.write(dest)
    node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', dest_path], output='screen')
    return [node]


def generate_launch_description() -> None:
    arguments = [
        launch.actions.DeclareLaunchArgument(
            'name', default_value='', description='The thymio name'),
        launch.actions.OpaqueFunction(function=rviz)
    ]
    return LaunchDescription(arguments)
