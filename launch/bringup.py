import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

def generate_launch_description():
    pkg_share = get_package_share_directory('sdnova_simulation')
    model_path = os.path.join(pkg_share, 'urdf/sdnova.urdf')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_path])}]
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros',
    	executable='spawn_entity.py',
        arguments=['-entity', 'sdnova', '-topic', 'robot_description', '-z', '0.6'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-u', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        robot_state_publisher_node,
        spawn_entity
    ])
