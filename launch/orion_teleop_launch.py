import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Argumentos
    raspberry_arg = DeclareLaunchArgument(
        "raspberry",
        default_value="no",
        description="Si es 'yes', no se lanzará el nodo TTS (está en la Raspberry)"
    )

    # Nodo STT - para reconocimiento de voz
    stt_node = Node(
        package="orion_chat",
        executable="orion_stt",
        name="orion_stt",
        output="screen"
    )

    # Nodo Chat - para procesamiento de conversación
    chat_node = Node(
        package="orion_chat",
        executable="orion_chat",
        name="orion_chat",
        output="screen"
    )

    # Nodo de control de brazos independiente
    arms_control_node = Node(
        package="orion_chat",
        executable="orion_arms_control",
        name="orion_arms_control",
        output="screen"
    )

    # Nodo TTS con control de base DESHABILITADO para teleoperación
    tts_node = Node(
        package="orion_chat",
        executable="orion_tts",
        name="orion_tts",
        output="screen",
        parameters=[{
            'enable_base_control': False  # CLAVE: Deshabilitamos control de base
        }]
    )

    # TTS retrasado y condicional (solo si no está en Raspberry)
    delayed_tts = TimerAction(
        period=5.0,
        actions=[tts_node],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("raspberry"), "' != 'yes'"
            ])
        )
    )

    return LaunchDescription([
        raspberry_arg,
        stt_node,
        chat_node,
        arms_control_node,  # Control independiente de brazos
        delayed_tts,       # TTS sin control de base
    ])
