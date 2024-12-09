from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'my_robot_package'  # 패키지 이름
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'ur_urdf.xacro'
    )
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )
    
    # Gazebo launch 파일 포함
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # 로봇 State Publisher 노드
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=['--ros-args', '-p', 'use_sim_time:=true'],
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    # Gazebo에 로봇을 스폰하는 노드
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'ur5_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
