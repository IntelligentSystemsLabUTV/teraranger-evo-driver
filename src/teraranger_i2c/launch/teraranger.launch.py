"""
Teraranger app launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

June 11, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Builds a LaunchDescription for the Teraranger app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(get_package_share_directory('teraranger_i2c'), 'config/teraranger.yaml')

    # Create node launch description
    node = Node(
        package='teraranger_i2c',
        executable='teraranger_app',
        exec_name='teraranger_app',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config]
    )

    ld.add_action(node)

    return ld
