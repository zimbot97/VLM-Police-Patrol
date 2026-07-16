"""Launch the oled_status HUD node with configurable I2C + network params."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('i2c_bus', default_value='5'),
        DeclareLaunchArgument('i2c_address', default_value='60'),   # 0x3C
        DeclareLaunchArgument('ip_interface', default_value='wlan0'),
        DeclareLaunchArgument('fps', default_value='15.0'),
        DeclareLaunchArgument('enable_display', default_value='true'),
        DeclareLaunchArgument('vlm_timeout_sec', default_value='900.0'),
    ]

    def pv(name, ptype):
        return ParameterValue(LaunchConfiguration(name), value_type=ptype)

    node = Node(
        package='oled_status',
        executable='oled_status',
        name='oled_status',
        output='screen',
        parameters=[{
            'i2c_bus': pv('i2c_bus', int),
            'i2c_address': pv('i2c_address', int),
            'ip_interface': pv('ip_interface', str),
            'fps': pv('fps', float),
            'enable_display': pv('enable_display', bool),
            'vlm_timeout_sec': pv('vlm_timeout_sec', float),
        }],
    )

    return LaunchDescription(args + [node])
