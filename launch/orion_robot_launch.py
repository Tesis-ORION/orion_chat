import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    tts_arg = DeclareLaunchArgument(
        "tts",
        default_value="yes",
        description="Si es 'no', no se lanzará el nodo TTS"
    )
    audio_recorder_arg = DeclareLaunchArgument(
        "audio_recorder",
        default_value="yes",
        description="Si es 'no', no se lanzará el nodo Audio Recorder"
    )
    tts_node = Node(
        package="orion_chat",
        executable="orion_tts",
        name="orion_tts",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("tts"), "' == 'yes'"
            ])
        )
    )
    audio_recorder_node = Node(
        package="orion_chat",
        executable="audio_recorder",
        name="audio_recorder",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("audio_recorder"), "' == 'yes'"
            ])
        )
    )

    return LaunchDescription([
        tts_arg,
        audio_recorder_arg,
        tts_node,
        audio_recorder_node,
    ])
