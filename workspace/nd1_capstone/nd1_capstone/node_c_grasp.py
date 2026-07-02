#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


VALID_ITEMS = ["parts_box", "tool_box", "sensor_box"]
VALID_OPS = ["grasp", "place"]


class NodeCGrasp(Node):
    def __init__(self):
        super().__init__("node_c_grasp")

        self.declare_parameter("sim_mode", True)
        self.declare_parameter("ik", True)

        self.sim_mode = bool(self.get_parameter("sim_mode").value)
        self.ik = bool(self.get_parameter("ik").value)

        self.pub_result = self.create_publisher(Bool, "/grasp_result", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/grasp_request", self._on_grasp_request, 10)

        self._status(f"Node C 시작 (sim_mode={self.sim_mode}, IK={'ON' if self.ik else 'OFF'})")

    def _on_grasp_request(self, msg: String):
        try:
            req = json.loads(msg.data)
        except Exception as e:
            self._status(f"grasp_request JSON 오류: {e}")
            self._publish_result(False)
            return

        op = req.get("op")
        item = req.get("item")

        if op not in VALID_OPS:
            self._status(f"알 수 없는 작업 op: {op}")
            self._publish_result(False)
            return

        if item not in VALID_ITEMS:
            self._status(f"알 수 없는 물품 item: {item}")
            self._publish_result(False)
            return

        self._status(f"파지 요청 수신: op={op}, item={item}")

        if self.sim_mode:
            self._status(f"sim {op} 성공: {item}")
            self._publish_result(True)
            return

        self._status("real grasp/place not implemented")
        self._publish_result(False)

    def _publish_result(self, ok: bool):
        self.pub_result.publish(Bool(data=ok))
        self._status("파지 결과: " + ("성공" if ok else "실패"))

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[C] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = NodeCGrasp()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
