import launch
from launch.actions import RegisterEventHandler
import launch_ros
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart



def generate_launch_description():
    
    orion_tts = Node(
            package="orion_chat",
            executable="orion_tts",
            name="orion_tts",
            output="screen"
        )
    delayed_tts = launch.actions.TimerAction(
        period=5.0,
        actions=[orion_tts]
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="orion_chat",
            executable="orion_stt",
            name="orion_stt",
            output="screen"
        ),
        launch_ros.actions.Node(
            package="orion_chat",
            executable="orion_chat",
            name="orion_chat",
            output="screen"
        ),
        delayed_tts

    ])

