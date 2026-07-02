cat > workspace/nd1_capstone/nd1_capstone/node_c_grasp.py <<'PY'
#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
#  Node C — 파지/배치 노드 (IK) [학생 구현]
#  역할: /grasp_request → nd1_m7_ik IK 계산 → 텔레포트 파지/배치 → /grasp_result
#
#  제공(인프라): pub/sub, 파라미터, 팔 초기화, main
#  구현 완료:  ① _solve_ik (nd1_m7_ik 호출)  ② _teleport (ign service)
#
#  토픽 계약(고정):
#    In  /grasp_request {op:"grasp"|"place", x, y}
#    Out /grasp_result(Bool) / /robot_status
#  ★ 표준안 제약: 팔 로컬 파지 타깃 y-offset ≥ 0.20 (y=0 특이점 → IK 발산)
#  ★ 통합 호환: sim_mode에서는 {op,item}만 와도 기본 좌표로 처리
# ════════════════════════════════════════════════════════════════
import json
import os
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

Y_OFFSET_MIN = 0.20  # 특이점 회피 최소 y (표준안) — 변경 금지


class NodeCGrasp(Node):
    def __init__(self):
        super().__init__("node_c_grasp")
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("arm_links", [0.20, 0.18, 0.12])
        self.declare_parameter("grasp_x", 0.35)
        self.declare_parameter("grasp_y", 0.25)
        self.declare_parameter("world_name", "warehouse")
        self.declare_parameter("box_model", "box1")
        self.declare_parameter("box_sdf_path", "")
        self.sim_mode = self.get_parameter("sim_mode").value
        self.links = list(self.get_parameter("arm_links").value)

        self.pub_result = self.create_publisher(Bool, "/grasp_result", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/grasp_request", self._on_request, 10)
        self._robot = self._init_arm()
        self._status(f"Node C 시작 (sim_mode={self.sim_mode}, IK={'ON' if self._robot else 'OFF'})")

    def _on_request(self, msg: String):
        try:
            d = json.loads(msg.data)
            op = d.get("op", "grasp")

            if op not in ("grasp", "place"):
                self._status(f"⚠️ 알 수 없는 op: {op}")
                self._result(False)
                return

            # 팀원 Coordinator가 parts_box로 보내는 경우도
            # 우리 시나리오의 electronic_parts_box로 해석
            raw_item = d.get("item", "electronic_parts_box")
            if raw_item == "parts_box":
                item = "electronic_parts_box"
            else:
                item = raw_item

            # 전자부품 상자는 기본적으로 fragile=True
            fragile = bool(d.get("fragile", item == "electronic_parts_box"))

            # 기존 공식 계약: op + x + y
            # 팀원 통합 테스트 계약: op + item
            # x/y가 없으면 sim_mode에서만 기본 좌표를 넣어 통합 흐름이 멈추지 않게 함
            if "x" in d and "y" in d:
                wx, wy = float(d["x"]), float(d["y"])
            elif self.sim_mode:
                if op == "grasp":
                    wx, wy = 1.5, 0.5
                else:
                    wx, wy = 2.5, -1.0

                self._status(
                    f"⚠️ x/y 없는 grasp_request 수신 → sim_mode 기본 좌표 사용: "
                    f"op={op}, item={item}, target=({wx:.2f},{wy:.2f})"
                )
            else:
                self._status("⚠️ grasp_request에 x/y 좌표가 없음")
                self._result(False)
                return

        except (json.JSONDecodeError, ValueError) as e:
            self._status(f"⚠️ grasp_request 파싱 실패: {e}")
            self._result(False)
            return

        # 작업 가능 범위 제한
        if abs(wx) > 5.0 or abs(wy) > 5.0:
            self._status(
                f"⚠️ 작업 범위 초과: ({wx:.2f}, {wy:.2f}) — Node C 처리 불가"
            )
            self._result(False)
            return

        tx = float(self.get_parameter("grasp_x").value)
        ty = float(self.get_parameter("grasp_y").value)

        # ★ 특이점 회피: y-offset 강제
        if abs(ty) < Y_OFFSET_MIN:
            self._status(f"⚠️ y={ty:.2f} < {Y_OFFSET_MIN} → 클램프")
            ty = Y_OFFSET_MIN

        q = self._solve_ik(tx, ty)
        if q is None:
            self._status("⚠️ IK 수렴 실패")
            self._result(False)
            return

        if fragile:
            self._status(
                f"{op} 대상={item}, fragile=True → 충격 최소화 저속 파지 모드"
            )
        else:
            self._status(
                f"{op} 대상={item}, fragile=False → 일반 파지 모드"
            )

        self._status(
            f"{op} IK 해 q={[round(float(v), 3) for v in q]} "
            f"(target=({tx:.2f},{ty:.2f}))"
        )

        ok = self._teleport(op, wx, wy)
        self._result(ok)

# ── ① 구현 완료: IK 계산 (nd1_m7_ik) ──────────────────────────────
    def _solve_ik(self, x, y):
        """팔 로컬 타깃 (x,y)의 관절각을 구한다. 실패 시 None."""
        if self._robot is None:
            if self.sim_mode:
                self._status("IK 로봇 모델 없음 → sim_mode 임시 관절각 사용")
                return [0.0, 0.0, 0.0]
            return None

        try:
            from nd1_m7_ik import numerical_ik
            q = numerical_ik(self._robot, (x, y))
            return [float(v) for v in q]
        except Exception as e:
            self._status(f"⚠️ IK 계산 예외: {e}")
            if self.sim_mode:
                self._status("sim_mode이므로 임시 관절각 사용")
                return [0.0, 0.0, 0.0]
            return None

    def _init_arm(self):
        try:
            from nd1_m7_ik import RobotArm3DOF
            return RobotArm3DOF(links=self.links)
        except Exception as e:
            self.get_logger().warn(f"nd1_m7_ik 로드 실패(sim 전용 가능): {e}")
            return None

# ── ② 구현 완료: 텔레포트 파지/배치 (ign service) ──────────────────
    def _teleport(self, op: str, x: float, y: float) -> bool:
        """op=grasp → 박스 제거 / op=place → (x,y)에 박스 재생성. 성공 bool."""
        op = str(op).lower().strip()

        if self.sim_mode:
            self._status(f"[sim] {op} 텔레포트 가정 — 성공")
            return True

        world = str(self.get_parameter("world_name").value)
        box = str(self.get_parameter("box_model").value)
        sdf_path = str(self.get_parameter("box_sdf_path").value)

        if not sdf_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                sdf_path = os.path.join(
                    get_package_share_directory("nd1_capstone"),
                    "models",
                    f"{box}.sdf",
                )
            except Exception:
                sdf_path = f"/home/ubuntu/ros2_ws/src/nd1_capstone/models/{box}.sdf"

        try:
            if op == "grasp":
                cmd = [
                    "ign", "service",
                    "-s", f"/world/{world}/remove",
                    "--reqtype", "ignition.msgs.Entity",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "5000",
                    "--req", f'name: "{box}" type: MODEL',
                ]
                self._status(f"Gazebo remove 요청: {box}")

            elif op == "place":
                req = (
                    f'sdf_filename: "{sdf_path}" '
                    f'name: "{box}" '
                    f'pose: {{ position: {{ x: {float(x)} y: {float(y)} z: 0.1 }} }}'
                )
                cmd = [
                    "ign", "service",
                    "-s", f"/world/{world}/create",
                    "--reqtype", "ignition.msgs.EntityFactory",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "5000",
                    "--req", req,
                ]
                self._status(f"Gazebo create 요청: {box} at ({x:.2f}, {y:.2f})")

            else:
                self._status(f"⚠️ 알 수 없는 op: {op}")
                return False

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5,
            )

            output = (result.stdout + result.stderr).lower()
            ok = ("true" in output) and (result.returncode == 0)

            if not ok:
                self._status(f"⚠️ Gazebo service 실패: {output.strip()}")

            return ok

        except subprocess.TimeoutExpired:
            self._status("⚠️ Gazebo service timeout")
            return False
        except FileNotFoundError:
            self._status("⚠️ ign 명령을 찾지 못함")
            return False
        except Exception as e:
            self._status(f"⚠️ 텔레포트 예외: {e}")
            return False

    def _result(self, ok: bool):
        self.pub_result.publish(Bool(data=ok))
        self._status(f"결과: {'성공' if ok else '실패'}")

    def _status(self, text):
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
PY