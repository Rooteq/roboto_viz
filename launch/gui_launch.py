import os
import xacro

#TODO: maybe look into sim_time (youre launch both as a sim app or real app)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    gui_node = Node(
        package='roboto_viz',
        executable='test',
        # name='gui_app',
        output='both',
        emulate_tty=True
    )

    return LaunchDescription([
        gui_node
    ])