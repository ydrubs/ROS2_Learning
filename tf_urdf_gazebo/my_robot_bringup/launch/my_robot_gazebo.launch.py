from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot_description')
    pkg_gzb = get_package_share_directory('ros_gz_sim')


    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    rviz_path = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    gz_launch_path = os.path.join(pkg_gzb, 'launch', 'gz_sim.launch.py')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    gazebo_include = IncludeLaunchDescription(
                     PythonLaunchDescriptionSource(gz_launch_path),
                     launch_arguments={'gz_args' : 'empty.sdf -r'}.items()
                    )   


    """
        Applies the CLI command:
            ros2 run robot_state_publisher robot_state_publisher \
            --ros-args -p robot_description:="$(xacro my_robot.urdf.xacro)" 
    """
    robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{'robot_description' : robot_description}],
    output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',rviz_path]
    )

    robot_instance_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        output="screen"
    )

    return LaunchDescription([
        gazebo_include,
        robot_state_publisher_node,
        robot_instance_node, 
        rviz2_node

    ])

    
