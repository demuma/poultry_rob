#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose


class FakeNavServer(Node):

    def __init__(self):
        super().__init__("fake_nav2_server")

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_callback
        )

        self.get_logger().info("Fake NavigateToPose server ready.")

    async def execute_callback(self, goal_handle):
        pose = goal_handle.request.pose.pose.position

        self.get_logger().info(
            f"Goal received: x={pose.x:.2f}, y={pose.y:.2f}"
        )

        goal_handle.succeed()

        result = NavigateToPose.Result()

        self.get_logger().info("Returning result")

        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeNavServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
