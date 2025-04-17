import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    orion_chat_share = get_package_share_directory('orion_chat')
    
    orion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(orion_chat_share, 'launch', 'orion_launch.py'))
    )
    
    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=['/model/my_vehicle/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        remappings=[('/model/my_vehicle/cmd_vel', '/cmd_vel')],
        output='screen'
    )
    
    world_file = os.path.join(orion_chat_share, 'worlds', 'car_world.sdf')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"{world_file} -r"}.items()
    )
    
    ros_gz_sim_demos_share = get_package_share_directory('ros_gz_sim_demos')
    model_file = os.path.join(ros_gz_sim_demos_share, 'models', 'vehicle', 'model.sdf')
    spawn_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')),
        launch_arguments={
            'world': 'car_world',
            'file': model_file,
            'entity_name': 'my_vehicle',
            'x': '5.0',
            'y': '5.0',
            'z': '0.5'
        }.items()
    )
    
    return LaunchDescription([
        orion_launch,
        parameter_bridge_node,
        gz_sim_launch,
        TimerAction(period=3.0, actions=[spawn_model_launch])
    ])

if __name__ == '__main__':
    generate_launch_description()
