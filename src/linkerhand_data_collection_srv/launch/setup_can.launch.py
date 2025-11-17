from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def _can_setup_actions(prefix: str):
    enable = LaunchConfiguration(f"{prefix}_enable")
    iface = LaunchConfiguration(f"{prefix}_if")
    name = LaunchConfiguration(f"{prefix}_name")
    bitrate = LaunchConfiguration(f"{prefix}_bitrate")

    # Bash command to: ensure modules, bring down iface, rename, set bitrate, and bring up
    cmd = (
        "bash -c '"
        "set -e; "
        "if command -v modprobe >/dev/null 2>&1; then "
        "sudo -n modprobe can || true; "
        "sudo -n modprobe can_raw || true; "
        "sudo -n modprobe slcan || true; "
        "fi; "
        # try to bring IFACE down (if present)
        "sudo -n ip link set ${IFACE} down || true; "
        # rename IFACE->NAME if different and IFACE exists
        "if ip link show ${IFACE} >/dev/null 2>&1 && [ \"${IFACE}\" != \"${NAME}\" ]; then "
        "  sudo -n ip link set dev ${IFACE} name ${NAME} || true; "
        "fi; "
        # ensure NAME exists, operate on NAME
        "sudo -n ip link set ${NAME} down || true; "
        "sudo -n ip link set ${NAME} type can bitrate ${BITRATE} || true; "
        "sudo -n ip link set ${NAME} up || true; "
        "ip -details -statistics link show ${NAME} | cat'"
    )

    return ExecuteProcess(
        cmd=[
            cmd
        ],
        additional_env={
            "IFACE": iface,
            "NAME": name,
            "BITRATE": bitrate,
        },
        shell=True,
        condition=IfCondition(enable),
    )


def generate_launch_description():
    # Define arguments for up to four CAN devices
    args = []
    for prefix, default_if, default_name in [
        ("hand_left", "can0", "hand_left"),
        ("hand_right", "can1", "hand_right"),
        ("arm_left", "can2", "arm_left"),
        ("arm_right", "can3", "arm_right"),
    ]:
        args += [
            DeclareLaunchArgument(f"{prefix}_enable", default_value="false"),
            DeclareLaunchArgument(f"{prefix}_if", default_value=default_if),
            DeclareLaunchArgument(f"{prefix}_name", default_value=default_name),
            DeclareLaunchArgument(f"{prefix}_bitrate", default_value="1000000"),
        ]

    actions = [*args]

    # For each device, add a conditional ExecuteProcess (enabled via *_enable:=true)
    # Launch conditions are not directly used here; use enable flags when invoking
    # This launch file expects to be run with sudo-nopasswd configured for modprobe/ip.

    # hand_left
    actions.append(_can_setup_actions("hand_left"))
    # hand_right
    actions.append(_can_setup_actions("hand_right"))
    # arm_left
    actions.append(_can_setup_actions("arm_left"))
    # arm_right
    actions.append(_can_setup_actions("arm_right"))

    return LaunchDescription(actions)


