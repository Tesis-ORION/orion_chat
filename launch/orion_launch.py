import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 1) Declaramos el argumento “raspberry” (default: "no")
    raspberry_arg = DeclareLaunchArgument(
        "raspberry",
        default_value="no",
        description="Si es 'yes', no se lanzará el nodo TTS (está en la Raspberry)"
    )

    # 2) Nodo STT y Chat siempre levantan
    stt_node = Node(
        package="orion_chat",
        executable="orion_stt",
        name="orion_stt",
        output="screen"
    )
    chat_node = Node(
        package="orion_chat",
        executable="orion_chat",
        name="orion_chat",
        output="screen"
    )

    # 3) Nodo TTS envuelto en Timer + condición
    tts_node = Node(
        package="orion_chat",
        executable="orion_tts",
        name="orion_tts",
        output="screen"
    )
    
    delayed_tts = TimerAction(
        period=5.0,
        actions=[tts_node],
        # Solo lanzar si raspberry != "yes"
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("raspberry"), "' != 'yes'"
            ])
        )
    )

    return LaunchDescription([
        # registrar el argumento para que esté disponible en this launch
        raspberry_arg,
        # nodos básicos
        stt_node,
        chat_node,
        # TTS (condicional y retrasado)
        delayed_tts,
    ])

