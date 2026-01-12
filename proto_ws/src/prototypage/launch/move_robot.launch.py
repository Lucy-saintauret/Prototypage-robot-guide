from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lancement du client de service client_mode.py
        Node(
            package='prototypage',
            executable='mode_choice',
            name='mode_choice',
            output='screen'
        ),
        
        Node(
            package='prototypage',
            executable='suivi_personne_green',
            name='suivi_personne',
            output='screen'
        ),
        
        Node(
            package='prototypage',
            executable='control_roues',
            name='control_roues',
            output='screen'
        )
    ])
