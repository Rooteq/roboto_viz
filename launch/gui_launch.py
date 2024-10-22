import os
import xacro

#TODO: fix use sim time as a settable parameter

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    gui_node = Node(
        package='roboto_viz',
        executable='test',
        name='gui_app',
        output='both'
    )

    return LaunchDescription([
        gui_node
    ])