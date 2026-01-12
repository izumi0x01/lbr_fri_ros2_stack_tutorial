#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class PilzMoveGroupClient(Node):
    def __init__(self):
        super().__init__('pilz_moveit_client')
        
        self.client = ActionClient(self, MoveGroup, '/lbr/move_action')
        self.client.wait_for_server()
    
    def move_linear(self, target_pose):
        """
        Pilz LIN プランナーで直線移動
        """
        goal = MoveGroup.Goal()
        
        # ===== プランニングパラメータ =====
        goal.request.group_name = "arm"
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # ===== Pilz固有: プランナーIDの指定 =====
        goal.request.planner_id = "LIN"  # ★ 直線移動プランナー
        
        # ===== 制約設定 =====
        constraint = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header = Header(frame_id="lbr_link_0")
        pos_constraint.link_name = "lbr_link_ee"
        pos_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=2, dimensions=[0.001])
        )
        pos_constraint.constraint_region.primitive_poses.append(
            Pose(position=target_pose.position)
        )
        pos_constraint.weight = 1.0
        constraint.position_constraints.append(pos_constraint)
        
        goal.request.goal_constraints.append(constraint)
        
        # ===== Goal送信 =====
        self.get_logger().info("Sending LIN motion goal...")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False
        
        # ===== 結果待機 =====
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        return result.error_code.val == 1  # SUCCESS


def main():
    rclpy.init()
    client = PilzMoveGroupClient()
    
    # ===== 直線移動のテスト =====
    waypoints = [
        Pose(position=Point(x=0.4, y=0.0, z=0.6), orientation=Quaternion(w=1.0)),
        Pose(position=Point(x=0.4, y=0.2, z=0.6), orientation=Quaternion(w=1.0)),
        Pose(position=Point(x=0.4, y=0.2, z=0.4), orientation=Quaternion(w=1.0)),
    ]
    
    for i, pose in enumerate(waypoints):
        client.get_logger().info(f"\n=== Waypoint {i+1} ===")
        success = client.move_linear(pose)
        if not success:
            client.get_logger().error("Motion failed, stopping.")
            break
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()