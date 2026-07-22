from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    machine_file = LaunchConfiguration("machine_file")
    inventory_file = LaunchConfiguration("inventory_file")
    legacy_file = LaunchConfiguration("legacy_file")
    web_preview = LaunchConfiguration("web_preview")
    web_port = LaunchConfiguration("web_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "machine_file", default_value="/etc/vixel/machine.yaml",
            description="Host/provider configuration YAML (copy from machine.example.yaml)",
        ),
        DeclareLaunchArgument(
            "inventory_file", default_value="/var/lib/vixel/inventory.yaml",
            description="Writable enrolled-sensor inventory YAML",
        ),
        DeclareLaunchArgument(
            "legacy_file", default_value="",
            description="Optional one-time legacy inventory migration input",
        ),
        DeclareLaunchArgument(
            "web_preview", default_value="false",
            description="Start the loopback dashboard and put sensors in preview mode",
        ),
        DeclareLaunchArgument(
            "web_port", default_value="8080",
            description="Single loopback dashboard/API/image port",
        ),
        Node(
            package="vixel_manager",
            executable="inventory_manager",
            name="inventory_manager",
            namespace="vixel",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "machine_file": machine_file,
                "inventory_file": inventory_file,
                "legacy_file": legacy_file,
                "camera_backend": "genicam",
                "start_preview": ParameterValue(web_preview, value_type=bool),
            }],
            on_exit=Shutdown(reason="Vixel inventory manager exited"),
        ),
        Node(
            package="vixel_genicam",
            executable="genicam_provider",
            name="genicam_provider",
            namespace="vixel/providers/genicam",
            output="screen",
            emulate_tty=True,
            parameters=[{"machine_file": machine_file}],
            on_exit=Shutdown(reason="Vixel GenICam provider exited"),
        ),
        Node(
            package="vixel_web",
            executable="web_gateway",
            name="web_gateway",
            namespace="vixel",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(web_preview),
            parameters=[{
                "address": "127.0.0.1",
                "port": ParameterValue(web_port, value_type=int),
                "machine_file": machine_file,
                "inventory_file": inventory_file,
            }],
            on_exit=Shutdown(reason="Vixel web gateway exited"),
        ),
    ])
