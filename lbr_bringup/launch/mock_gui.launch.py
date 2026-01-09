from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())   
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    # static transform world -> <robot_name>_floating_link
    ld.add_action(
        LBRDescriptionMixin.node_static_tf(
            tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            parent="world",
            child=PythonExpression(
                ["'", LaunchConfiguration("robot_name"), "' + '_floating_link'"]
            ),
        )
    )

    # robot description
    robot_description = LBRDescriptionMixin.param_robot_description(mode="mock")

    # robot_state_publisher (/lbr/joint_states)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        namespace="lbr",
        remappings=[
            ("/joint_states", "/lbr/joint_states")
        ]
    )
    ld.add_action(robot_state_publisher)

    # joint_state_publisher_gui (/lbr/joint_states)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        namespace="lbr",
        remappings=[
            ("/joint_states", "/lbr/joint_states")
        ]
    )
    ld.add_action(joint_state_publisher_gui)

    return ld
