from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='1.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                    ])   
                )
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch',
                    'rviz2.launch.py'
                    ])
                )
            ),

        Node(
            package="dwa_motion_planner",
            executable="dwa_action_server",
            name='dwa_action_server',
            output='screen'
            ),
        
        TimerAction(
            period=5.0,
            actions=[
            Node(
                package="dwa_motion_planner",
                executable="dwa_action_client",
                name='dwa_action_client',
                output='screen',
                parameters=[{
                    'goal_x': LaunchConfiguration('goal_x'),
                    'goal_y': LaunchConfiguration('goal_y')
                    }]
                )
            ]
        )


    ])
