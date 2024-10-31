from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for the quadcopter."""
    pkg_project_bringup = get_package_share_directory("drone_acharya_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    sdf_file  =  os.path.join(pkg_project_bringup, 'models', 'ign_drone_v1', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo
    world_file = PathJoinSubstitution([pkg_project_bringup, 'worlds', 'ign_drone_v1_world.sdf'])
    ign_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v4', world_file],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "drone_v1_rviz.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    ros2_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge_drone_v1.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    bridge = Node(
        name='parameter_bridge_clock',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ign.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            ign_sim,
            robot_state_publisher,   
            ros2_gz_bridge,
            bridge,
            # rviz,
        ]
    )
