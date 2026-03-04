from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters
    initial_percentage = LaunchConfiguration('initial_percentage')
    nominal_voltage = LaunchConfiguration('nominal_voltage')
    capacity_ah = LaunchConfiguration('capacity_ah')
    idle_current_amps = LaunchConfiguration('idle_current_amps')
    discharge_rate_per_meter = LaunchConfiguration('discharge_rate_per_meter')
    odom_topic = LaunchConfiguration('odom_topic')
    battery_topic = LaunchConfiguration('battery_topic')

    declare_initial_percentage = DeclareLaunchArgument(
        'initial_percentage', default_value='1.0')
    declare_nominal_voltage = DeclareLaunchArgument(
        'nominal_voltage', default_value='24.0')
    declare_capacity_ah = DeclareLaunchArgument(
        'capacity_ah', default_value='20.0')
    declare_idle_current_amps = DeclareLaunchArgument(
        'idle_current_amps', default_value='1.25')
    declare_discharge_rate_per_meter = DeclareLaunchArgument(
        'discharge_rate_per_meter', default_value='0.0001')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/odometry')
    declare_battery_topic = DeclareLaunchArgument(
        'battery_topic', default_value='/battery_state')

    battery_simulator_node = Node(
        package='clearpath_gz',
        executable='battery_simulator',
        name='battery_simulator',
        output='screen',
        parameters=[{
            'initial_percentage': initial_percentage,
            'nominal_voltage': nominal_voltage,
            'capacity_ah': capacity_ah,
            'idle_current_amps': idle_current_amps,
            'discharge_rate_per_meter': discharge_rate_per_meter,
            'odom_topic': odom_topic,
            'battery_topic': battery_topic
        }]
    )

    return LaunchDescription([
        declare_initial_percentage,
        declare_nominal_voltage,
        declare_capacity_ah,
        declare_idle_current_amps,
        declare_discharge_rate_per_meter,
        declare_odom_topic,
        declare_battery_topic,
        battery_simulator_node
    ])
