# launch/test_stop_turn.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    cfg_arg = DeclareLaunchArgument(
        "in_place_cfg",
        default_value=PathJoinSubstitution([
            get_package_share_directory("motion_primitives"),
            "config",
            "config.yaml",   # 存在するYAMLに合わせてください
        ]),
        description="Parameter YAML for in_place_turn_node (angular_speed, etc.)"
    )

    in_place = LifecycleNode(
        package="motion_primitives",
        executable="in_place_turn",
        name="in_place_turn_node",
        namespace="",                # ← これが必須（空でもOK）
        output="screen",
        parameters=[LaunchConfiguration("in_place_cfg")]
    )

    stop = LifecycleNode(
        package="motion_primitives",
        executable="stop_motion",
        name="stop_motion_node",
        namespace="",                # ← これが必須
        output="screen"
    )

    emit_cfg_in_place = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == in_place,
        transition_id=Transition.TRANSITION_CONFIGURE
    ))

    emit_cfg_stop = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: n == stop,
        transition_id=Transition.TRANSITION_CONFIGURE
    ))

    return LaunchDescription([
        cfg_arg,
        in_place,
        stop,
        emit_cfg_in_place,
        emit_cfg_stop,
    ])
