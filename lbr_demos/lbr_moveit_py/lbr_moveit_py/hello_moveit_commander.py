import rclpy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander, moveit_commander
from geometry_msgs.msg import Pose


def main():
    rclpy.init()
    node = rclpy.create_node("hello_moveit_commander")

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("arm")

    initial_joint_positions = {
        "lbr_joint_1": 0.0,
        "lbr_joint_2": -0.5,
        "lbr_joint_3": 0.0,
        "lbr_joint_4": 1.5,
        "lbr_joint_5": 0.0,
        "lbr_joint_6": 1.0,
        "lbr_joint_7": 0.0,
    }
    group.set_joint_value_target(initial_joint_positions)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    ###
    start_pose = group.get_current_pose().pose
    waypoints = []

    target_pose = Pose()
    target_pose.position.x = start_pose.position.x
    target_pose.position.y = start_pose.position.y
    target_pose.position.z = start_pose.position.z - 0.1
    target_pose.orientation = start_pose.orientation
    waypoints.append(target_pose)

    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
        jump_threshold=0.0
    )

    if fraction > 0.95:
        group.execute(plan, wait=True)
    else:
        print(f"Cartesian path planning incomplete: {fraction*100:.1f}% success")
    group.stop()
    group.clear_pose_targets()

    ###
    start_pose = group.get_current_pose().pose
    waypoints = []

    target_pose = Pose()
    target_pose.position.x = start_pose.position.x
    target_pose.position.y = start_pose.position.y
    target_pose.position.z = start_pose.position.z - 0.1
    target_pose.orientation = start_pose.orientation
    waypoints.append(target_pose)

    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
        jump_threshold=0.0
    )

    if fraction > 0.95:
        group.execute(plan, wait=True)
    else:
        print(f"Cartesian path planning incomplete: {fraction*100:.1f}% success")
    group.stop()
    group.clear_pose_targets()

    ###
    start_pose = group.get_current_pose().pose
    waypoints = []

    target_pose = Pose()
    target_pose.position.x = start_pose.position.x
    target_pose.position.y = start_pose.position.y
    target_pose.position.z = start_pose.position.z - 0.1
    target_pose.orientation = start_pose.orientation
    waypoints.append(target_pose)

    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
        jump_threshold=0.0
    )

    if fraction > 0.95:
        group.execute(plan, wait=True)
    else:
        print(f"Cartesian path planning incomplete: {fraction*100:.1f}% success")
    group.stop()
    group.clear_pose_targets()

    ###
    start_pose = group.get_current_pose().pose
    waypoints = []

    target_pose = Pose()
    target_pose.position.x = start_pose.position.x
    target_pose.position.y = start_pose.position.y
    target_pose.position.z = start_pose.position.z - 0.1
    target_pose.orientation = start_pose.orientation
    waypoints.append(target_pose)

    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.01,
        jump_threshold=0.0
    )

    if fraction > 0.95:
        group.execute(plan, wait=True)
    else:
        print(f"Cartesian path planning incomplete: {fraction*100:.1f}% success")
    group.stop()
    group.clear_pose_targets()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
