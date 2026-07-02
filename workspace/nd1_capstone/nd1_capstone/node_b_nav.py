#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


VALID_TARGETS = {
    "Shelf_1": (1.5, 0.5, 0.0),
    "Shelf_2": (2.5, -1.0, 0.0),
    "Shelf_3": (0.5, 2.0, 0.0),
    "Worker": (0.0, 0.0, 0.0),
    "Workbench": (1.0, 0.0, 0.0),
}


class NodeBNav(Node):
    def __init__(self):
        super().__init__("node_b_nav")

        self.declare_parameter("sim_mode", True)
        self.declare_parameter("dock", True)

        self.sim_mode = bool(self.get_parameter("sim_mode").value)
        self.dock = bool(self.get_parameter("dock").value)

        self.pub_result = self.create_publisher(Bool, "/nav_result", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/nav_request", self._on_nav_request, 10)

        self._status(f"Node B 시작 (sim_mode={self.sim_mode}, dock={'ON' if self.dock else 'OFF'})")

    def _on_nav_request(self, msg: String):
        try:
            req = json.loads(msg.data)
        except Exception as e:
            self._status(f"nav_request JSON 오류: {e}")
            self._publish_result(False)
            return

        target = req.get("target")

        if target not in VALID_TARGETS:
            self._status(f"알 수 없는 이동 target: {target}")
            self._publish_result(False)
            return

        x, y, yaw = VALID_TARGETS[target]
        self._status(f"이동 요청 수신: target={target}, pose=({x}, {y}, {yaw})")

        if self.sim_mode:
            self._status(f"sim 이동 성공: {target}")
            self._publish_result(True)
            return

        self._status("real navigation not implemented")
        self._publish_result(False)

    def _publish_result(self, ok: bool):
        self.pub_result.publish(Bool(data=ok))
        self._status("이동 결과: " + ("성공" if ok else "실패"))

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[B] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = NodeBNav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
