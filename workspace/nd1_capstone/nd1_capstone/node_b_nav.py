#!/usr/bin/env python3
# Node B — 이동/Nav2 래퍼
# 역할: /nav_request(String JSON) → /nav_result(Bool)
#
# 구현 원칙
# - Coordinator와의 공통 계약은 {"target": "Shelf_1"} 형태로 유지한다.
# - sim_mode:=true에서는 Nav2 없이 즉시 성공을 반환해 통합 스모크 테스트가 가능하다.
# - sim_mode:=false에서는 Nav2 NavigateToPose 액션을 사용한다.

import json
import math
import re
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


# 기획서 ZONE_MAP 반영:
# A구역=(1.5, 0.5), B구역=(2.5, -1.0)
# Coordinator 계약 유지를 위해 A구역은 Shelf_1, B구역은 Workbench로도 접근 가능하게 둔다.
VALID_TARGETS: Dict[str, Tuple[float, float, float]] = {
    "Shelf_1": (1.5, 0.5, 0.0),
    "Shelf_2": (2.5, -1.0, 0.0),
    "Shelf_3": (0.5, 2.0, 0.0),
    "Worker": (0.0, 0.0, 0.0),
    "Workbench": (1.0, 0.0, 0.0),
}

TARGET_ALIASES = {
    "shelf1": "Shelf_1",
    "shelf_1": "Shelf_1",
    "1번선반": "Shelf_1",
    "1번": "Shelf_1",
    "선반1": "Shelf_1",
    "a구역": "Shelf_1",
    "a존": "Shelf_1",
    "azone": "Shelf_1",
    "aarea": "Shelf_1",

    "shelf2": "Shelf_2",
    "shelf_2": "Shelf_2",
    "2번선반": "Shelf_2",
    "2번": "Shelf_2",
    "선반2": "Shelf_2",

    "shelf3": "Shelf_3",
    "shelf_3": "Shelf_3",
    "3번선반": "Shelf_3",
    "3번": "Shelf_3",
    "선반3": "Shelf_3",
    "c구역": "Shelf_3",
    "c존": "Shelf_3",

    "worker": "Worker",
    "작업자": "Worker",
    "워커": "Worker",

    "workbench": "Workbench",
    "작업대": "Workbench",
    "워크벤치": "Workbench",
    "b구역": "Workbench",
    "b존": "Workbench",
    "bzone": "Workbench",
    "barea": "Workbench",
    "도착지": "Workbench",
    "배치구역": "Workbench",
}


class NodeBNav(Node):
    def __init__(self):
        super().__init__("node_b_nav")

        self.declare_parameter("sim_mode", True)
        self.declare_parameter("dock", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("nav_server_timeout", 3.0)

        self.sim_mode = bool(self.get_parameter("sim_mode").value)
        self.dock = bool(self.get_parameter("dock").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.nav_server_timeout = float(self.get_parameter("nav_server_timeout").value)

        self.pub_result = self.create_publisher(Bool, "/nav_result", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/nav_request", self._on_nav_request, 10)

        self._nav_client = None
        self._navigate_to_pose_cls = None
        self._busy = False
        self._active_target = None

        if not self.sim_mode:
            self._init_nav2_client()

        self._status(
            f"Node B 시작 (sim_mode={self.sim_mode}, dock={'ON' if self.dock else 'OFF'}, "
            f"frame_id={self.frame_id})"
        )

    def _on_nav_request(self, msg: String):
        if self._busy:
            self._status(f"이미 이동 중: active_target={self._active_target}, 새 요청 거부")
            self._publish_result(False)
            return

        try:
            req = json.loads(msg.data)
            if not isinstance(req, dict):
                raise ValueError("nav_request는 JSON object여야 함")
        except Exception as e:
            self._status(f"nav_request JSON 오류: {e}")
            self._publish_result(False)
            return

        parsed = self._parse_request(req)
        if parsed is None:
            self._publish_result(False)
            return

        target, x, y, yaw = parsed
        self._status(f"이동 요청 수신: target={target}, pose=({x:.2f}, {y:.2f}, {yaw:.2f})")

        if self.sim_mode:
            self._status(f"sim 이동 성공: {target}")
            self._publish_result(True)
            return

        self._send_nav_goal(target, x, y, yaw)

    def _parse_request(self, req: Dict[str, Any]) -> Optional[Tuple[str, float, float, float]]:
        target = self._normalize_target(req.get("target"))

        if target is None and "x" in req and "y" in req:
            try:
                x = float(req["x"])
                y = float(req["y"])
                yaw = float(req.get("yaw", 0.0))
                return "custom", x, y, yaw
            except (TypeError, ValueError) as e:
                self._status(f"좌표 기반 nav_request 파싱 실패: {e}")
                return None

        if target not in VALID_TARGETS:
            self._status(f"알 수 없는 이동 target: {req.get('target')}")
            return None

        x, y, yaw = VALID_TARGETS[target]
        return target, x, y, yaw

    def _normalize_target(self, value: Any) -> Optional[str]:
        if value is None:
            return None

        raw = str(value).strip()
        if raw in VALID_TARGETS:
            return raw

        norm_value = self._norm(raw)
        for alias, internal in sorted(TARGET_ALIASES.items(), key=lambda kv: len(self._norm(kv[0])), reverse=True):
            if self._norm(alias) == norm_value or self._norm(alias) in norm_value:
                return internal

        return None

    @staticmethod
    def _norm(text: str) -> str:
        return re.sub(r"[\s_\-:/,.;()\[\]{}]+", "", str(text)).lower()

    def _init_nav2_client(self) -> bool:
        if self._nav_client is not None:
            return True

        try:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
        except Exception as e:
            self._status(f"Nav2 모듈 로드 실패: {e}")
            return False

        self._navigate_to_pose_cls = NavigateToPose
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._status("Nav2 NavigateToPose action client 준비")
        return True

    def _send_nav_goal(self, target: str, x: float, y: float, yaw: float):
        if not self._init_nav2_client():
            self._publish_result(False)
            return

        if not self._nav_client.wait_for_server(timeout_sec=self.nav_server_timeout):
            self._status("Nav2 action server 대기 시간 초과")
            self._publish_result(False)
            return

        goal_msg = self._navigate_to_pose_cls.Goal()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = self._yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._busy = True
        self._active_target = target
        self._status(f"Nav2 goal 전송: target={target}")

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda future: self._on_goal_response(future, target))

    def _on_goal_response(self, future, target: str):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._busy = False
            self._active_target = None
            self._status(f"Nav2 goal 응답 예외: {e}")
            self._publish_result(False)
            return

        if not goal_handle.accepted:
            self._busy = False
            self._active_target = None
            self._status(f"Nav2 goal 거부: target={target}")
            self._publish_result(False)
            return

        self._status(f"Nav2 goal 수락: target={target}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_done(f, target))

    def _on_nav_done(self, future, target: str):
        ok = False
        status = None

        try:
            result = future.result()
            status = int(result.status)
            try:
                from action_msgs.msg import GoalStatus
                ok = status == GoalStatus.STATUS_SUCCEEDED
            except Exception:
                ok = status == 4  # STATUS_SUCCEEDED
        except Exception as e:
            self._status(f"Nav2 결과 예외: {e}")
            ok = False

        self._busy = False
        self._active_target = None
        self._status(f"Nav2 결과 수신: target={target}, status={status}, ok={ok}")
        self._publish_result(ok)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
        half = float(yaw) * 0.5
        return math.sin(half), math.cos(half)

    def _publish_result(self, ok: bool):
        self.pub_result.publish(Bool(data=bool(ok)))
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
