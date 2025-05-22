import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    orion_chat_share = get_package_share_directory('orion_chat')
    orion_gz_share = get_package_share_directory('orion_gz')
    
    orion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(orion_chat_share, 'launch', 'orion_launch.py'))
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(orion_gz_share, 'launch', 'gz_ros2_control.launch.py')),
        launch_arguments={
            'simplified': 'true',
        }.items()
    )
    
    
    return LaunchDescription([
        orion_launch,
        gz_launch,
    ])

if __name__ == '__main__':
    generate_launch_description()
