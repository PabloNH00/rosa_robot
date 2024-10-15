from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_common',  # Paquete del primer nodo
            executable='tts_node',  # Nodo a ejecutar
            name='tts_node',
            output='screen'
        ),
        Node(
            package='audio_common',  # Paquete del segundo nodo
            executable='audio_player_node',  # Nodo a ejecutar
            name='audio_player_node',
            output='screen'
        ),
        Node(
            package='voice_transcriber_pkg',  # Paquete del tercer nodo
            executable='voice_transcriber',  # Nodo a ejecutar
            name='voice_transcriber_node',
            output='screen'
        )
    ])
