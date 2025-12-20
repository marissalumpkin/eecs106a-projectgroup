from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

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
    
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='reactive',
        description='Controller type: "reactive" or "pid"'
    )

    return LaunchDescription([
        use_sim_arg,
        serial_port_arg,
        controller_type_arg,

        # 1. Reactive Controller Node
        Node(
            package='morphing_airfoil',
            executable='controller',
            name='airfoil_controller',
            output='screen',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'reactive'"])),
            parameters=[
                {'reactive_gain': 0.5},  # Tune this! (Degrees per Newton)
                {'calibration_duration': 5.0}  # Calibration time in seconds
            ]
        ),
        
        # 1b. PID Controller Node
        Node(
            package='morphing_airfoil',
            executable='pid_controller',
            name='airfoil_controller',
            output='screen',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'pid'"])),
            parameters=[
                {'kp': 2.5},      # Proportional gain - tune this!
                {'ki': 0.1},      # Integral gain - tune this!
                {'kd': 0.5},      # Derivative gain - tune this!
                {'target_lift': 6.0},  # Target lift in Newtons
                {'calibration_duration': 5.0}  # Calibration time in seconds
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