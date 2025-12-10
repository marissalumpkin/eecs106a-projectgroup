from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='True',
        description='Use simulation (True) or real hardware (False)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino (e.g., /dev/ttyUSB0 or /dev/tty.usbmodem...)'
    )

    return LaunchDescription([
        use_sim_arg,
        serial_port_arg,

        # 1. Controller Node
        Node(
            package='morphing_airfoil',
            executable='controller',
            name='airfoil_controller',
            output='screen',
            parameters=[
                {'reactive_gain': 0.5} # Tune this! (Degrees per Gram)
            ]
        ),
        
        # 2. Sensor Node (Sim or Real)
        Node(
            package='morphing_airfoil',
            executable='sensor_sim',
            name='sensor_driver',
            output='screen',
            parameters=[
                {'use_sim': LaunchConfiguration('use_sim')},
                {'serial_port': LaunchConfiguration('serial_port')},
                {'baud_rate': 115200}
            ]
        ),

        # 3. Actuator Simulation (REMOVED: Handled by sensor_sim)
        # Node(
        #     package='morphing_airfoil',
        #     executable='actuator_sim',
        #     name='actuator_driver',
        #     output='screen'
        # )
    ])