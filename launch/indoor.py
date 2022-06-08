import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_this = get_package_share_directory('openspot_sim')
    world_path = os.path.join(pkg_this, 'world/indoor.sdf')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/robot.py']),
            launch_arguments={'world': world_path, 'posx': '-0.9', 'posy': '0.5'}.items()
        ),
    ])
