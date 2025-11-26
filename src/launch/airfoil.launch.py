from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Controller Node
        Node(
            package='morphing_airfoil',
            executable='controller',
            name='airfoil_controller',
            output='screen',
            parameters=[
                {'kp': 2.5},
                {'ki': 0.1},
                {'kd': 0.5},
                {'target_lift': 6.0}
            ]
        ),
        
        # 2. Sensor Simulation (Replace with real driver later)
        Node(
            package='morphing_airfoil',
            executable='sensor_sim',
            name='sensor_driver',
            output='screen'
        ),

        # 3. Actuator Simulation (Replace with real driver later)
        Node(
            package='morphing_airfoil',
            executable='actuator_sim',
            name='actuator_driver',
            output='screen'
        )
    ])