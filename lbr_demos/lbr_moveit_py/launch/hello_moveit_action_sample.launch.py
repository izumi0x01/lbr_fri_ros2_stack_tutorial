#!/usr/bin/env python3
# pilz_moveit_client.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Pilz MoveGroup Client の起動用 Launch ファイル
    """
    
    return LaunchDescription([
        # ===== Launch Arguments =====
        DeclareLaunchArgument(
            'robot_name',
            default_value='lbr',
            description='Robot namespace'
        ),
        
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # ===== Node起動 =====
        Node(
            package='lbr_moveit_py',  # パッケージ名を入れる
            executable='hello_moveit_a',  # setup.py で定義した実行ファイル名
            name='pilz_moveit_client',
            namespace='',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ],
        )
    ])
    
    return ld