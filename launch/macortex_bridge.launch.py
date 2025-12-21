import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _make_node(context, *args, **kwargs):
    config_value = LaunchConfiguration("config").perform(context)

    # Accept:
    # - relative path (from cwd): config:=config/default.yaml
    # - relative path in install: config:=config/default.yaml
    package_share = get_package_share_directory("macortex_bridge")

    if os.path.isabs(config_value):
        raise RuntimeError(
            "Launch argument 'config' must be a relative path (e.g. config/default.yaml); "
            f"got absolute path: {config_value}"
        )

    if os.path.exists(config_value):
        # Relative path from the current working directory.
        config_path = os.path.abspath(config_value)
    else:
        # Relative path inside the installed package share.
        candidate = os.path.join(package_share, config_value)
        if not os.path.exists(candidate):
            raise RuntimeError(
                "Could not find params YAML. Tried: "
                f"'{config_value}' (relative to CWD) and '{candidate}' (relative to package share)."
            )
        config_path = candidate

    node = Node(
        package="macortex_bridge",
        executable="macortex_bridge",
        name="macortex_bridge",
        namespace=LaunchConfiguration("ns"),
        output="screen",
        parameters=[config_path],
    )

    return [node]


def generate_launch_description() -> LaunchDescription:
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="config/default.yaml",
        description=(
            "Params YAML relative path (e.g. config/default.yaml)"
        ),
    )

    ns_arg = DeclareLaunchArgument(
        "ns",
        default_value="",
        description="Namespace for the node (empty for root)",
    )

    return LaunchDescription([config_arg, ns_arg, OpaqueFunction(function=_make_node)])
