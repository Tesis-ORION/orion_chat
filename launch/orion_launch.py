import launch
import launch_ros.actions

def generate_launch_description():
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
        launch_ros.actions.Node(
            package="orion_chat",
            executable="orion_tts",
            name="orion_tts",
            output="screen"
        )
    ])
