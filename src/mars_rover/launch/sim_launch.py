from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    pkg_mars_rover = FindPackageShare('mars_rover')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # File paths
    world_path = '/home/ankith/mars_ws/src/mars_rover/worlds/mars.world'
    urdf_path = PathJoinSubstitution([pkg_mars_rover, 'urdf', 'robot.urdf.xacro'])
    rviz_config = '/home/ankith/mars_ws/src/mars_rover/rviz/mars_rover.rviz'
    slam_params = '/home/ankith/mars_ws/src/mars_rover/config/mapper_params_online_async.yaml'

    # Launch Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_gazebo_ros, '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                urdf_path
            ]),
            'use_sim_time': True
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mars_rover', '-topic', 'robot_description', '-z', '0.2'],
        output='screen'
    )

    # Keyboard teleop
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Relay laser scan topic
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='laser_relay',
        arguments=['/gazebo_ros_laser_controller/out', '/scan'],
        output='screen'
    )

    # SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        relay_node,
        slam_toolbox_launch,
        TimerAction(period=5.0, actions=[teleop]),
        TimerAction(period=3.0, actions=[rviz])
    ])