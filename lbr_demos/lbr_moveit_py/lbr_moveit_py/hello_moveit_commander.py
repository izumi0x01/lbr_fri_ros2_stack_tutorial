#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from std_msgs.msg import Header
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import threading


class CartesianPathExecutor(Node):
    def __init__(self):
        super().__init__('cartesian_path_executor')

        self.group_name = 'arm'
        self.base_frame = 'lbr_link_0'
        self.ee_link = 'lbr_link_ee'
        self.joint_state_topic = '/lbr/joint_states'
        self.cartesian_srv_ns = '/lbr/compute_cartesian_path'
        self.action_ns = '/lbr/joint_trajectory_controller/follow_joint_trajectory'

        self.current_joint_state = None
        self.joint_state_lock = threading.Lock()

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.cartesian_client = self.create_client(GetCartesianPath, self.cartesian_srv_ns)
        self.get_logger().info("Waiting for /lbr/compute_cartesian_path...")
        self.cartesian_client.wait_for_service()
        self.get_logger().info("Service is available.")

        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_ns,
        )
        self.get_logger().info("Waiting for FollowJointTrajectory action server...")
        self.trajectory_action_client.wait_for_server()
        self.get_logger().info("Action server available.")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.run_sequence()

    def get_current_pose(self) -> Pose:
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                rclpy.time.Time())
            pose = Pose()
            pose = Pose()
            pose.position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z,
            )
            pose.orientation = transform.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def joint_state_callback(self, msg: JointState):
        with self.joint_state_lock:
            self.current_joint_state = msg

    def wait_for_joint_state(self) -> JointState:
        self.get_logger().info("Waiting for /lbr/joint_states...")
        while rclpy.ok():
            with self.joint_state_lock:
                if self.current_joint_state:
                    return self.current_joint_state
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_trajectory(self, positions: list, duration_sec: int = 5):
        if len(positions) != 7:
            self.get_logger().error("Expected 7 joint values.")
            return

        goal = FollowJointTrajectory.Goal()
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start = Duration(sec=duration_sec)

        goal.trajectory.joint_names = [
            "lbr_A1",
            "lbr_A2",
            "lbr_A3",
            "lbr_A4",
            "lbr_A5",
            "lbr_A6",
            "lbr_A7"
        ]
        goal.trajectory.points.append(point)
        goal.goal_time_tolerance.sec = 1

        self.get_logger().info("Sending trajectory via FollowJointTrajectory action...")
        future = self.trajectory_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return

        self.get_logger().info("Trajectory accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error("Trajectory execution failed.")
        else:
            self.get_logger().info("Trajectory execution succeeded.")

    def plan_and_execute_cartesian_path(self, relx, rely, relz):
        # Get current joint state and end-effector pose
        joint_state = self.wait_for_joint_state()
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error("Failed to get current pose.")
            return

        # Prepare Cartesian planning request
        request = GetCartesianPath.Request()
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.header = Header(frame_id=self.base_frame)
        request.max_step = 0.02
        request.jump_threshold = 10.0
        request.avoid_collisions = False
        request.start_state = RobotState(joint_state=joint_state, is_diff=True)

        # Move downward 3cm from current pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + relx
        target_pose.position.y = current_pose.position.y + rely
        target_pose.position.z = current_pose.position.z + relz
        target_pose.orientation = current_pose.orientation
        request.waypoints.append(target_pose)

        # Call MoveIt service
        self.get_logger().info("Calling compute_cartesian_path...")
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        trajectory = result.solution.joint_trajectory
        if not trajectory.points:
            self.get_logger().error("Empty trajectory. Aborting.")
            return

        # Assign safe timing to all points
        duration_per_point = 0.5
        joint_count = len(trajectory.joint_names)
        for i, pt in enumerate(trajectory.points):
            total_sec = duration_per_point * (i + 1)
            pt.time_from_start = Duration(sec=int(total_sec), nanosec=int((total_sec % 1) * 1e9))
            pt.velocities = [0.0] * joint_count
            pt.accelerations = [0.0] * joint_count

        # Execute via FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = trajectory.joint_names
        goal.trajectory.points = trajectory.points
        goal.goal_time_tolerance.sec = 1

        self.get_logger().info("Sending Cartesian trajectory...")
        send_future = self.trajectory_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if not handle.accepted:
            self.get_logger().error("Trajectory goal was rejected.")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Cartesian trajectory execution completed.")

    def run_sequence(self):
        self.get_logger().info("Step 1: Moving to initial position...")
        self.send_trajectory([
            -0.087,
            -0.366,
            -0.524,
            1.239,
             0.192,
             -1.571,
             1.745
        ], duration_sec=8)

        self.get_logger().info("Step 2: Executing Cartesian path...")
        self.plan_and_execute_cartesian_path(relx=0.0, rely=0.0, relz=-0.03)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPathExecutor()
    rclpy.spin_once(node, timeout_sec=10.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
