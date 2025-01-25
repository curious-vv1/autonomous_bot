import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    package_name = 'auto_bot'

    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(
        package_name), 'launch', 'rsp.launch.py')]), launch_arguments={'use_sim_time': 'true'}.items())

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),)

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=[
                        '-topic', 'robot_description', '-entity', 'robot'], output='screen')

    # diff_drive_spawner = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=["diff_cont"],
    #         )
    #     ]

    # )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )


    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # joint_broad_spawner = TimerAction(
    #     period=7.0,
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=["joint_broad"],
    #         )
    #     ]

    # )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        # delayed_diff_drive_spawner,
        # joint_broad_spawner
    ])
