#!/usr/bin/env python3
# Node A — 자연어 명령 해석
# 역할: /llm_command(String) → /mission(String, JSON)

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 팀 공통 내부 이름
LOCATION_ALIASES = {
    "1번선반": "Shelf_1",
    "1번": "Shelf_1",
    "선반1": "Shelf_1",
    "Shelf_1": "Shelf_1",
    "SHELF_1": "Shelf_1",

    "2번선반": "Shelf_2",
    "2번": "Shelf_2",
    "선반2": "Shelf_2",
    "Shelf_2": "Shelf_2",
    "SHELF_2": "Shelf_2",

    "3번선반": "Shelf_3",
    "3번": "Shelf_3",
    "선반3": "Shelf_3",
    "Shelf_3": "Shelf_3",
    "SHELF_3": "Shelf_3",

    "작업자": "Worker",
    "워커": "Worker",
    "Worker": "Worker",
    "WORKER": "Worker",

    "작업대": "Workbench",
    "워크벤치": "Workbench",
    "Workbench": "Workbench",
    "WORKBENCH": "Workbench",
}

ITEM_ALIASES = {
    "부품박스": "parts_box",
    "부품": "parts_box",
    "partsbox": "parts_box",
    "parts_box": "parts_box",

    "공구박스": "tool_box",
    "공구": "tool_box",
    "toolbox": "tool_box",
    "tool_box": "tool_box",

    "센서박스": "sensor_box",
    "센서": "sensor_box",
    "sensorbox": "sensor_box",
    "sensor_box": "sensor_box",
}


SYSTEM_PROMPT = """
너는 창고 피킹 보조 로봇의 명령 파서다.
한국어 명령을 아래 JSON 형식으로만 변환한다.

단일 미션 형식:
{
  "task": "pick_and_deliver",
  "item": "parts_box",
  "source": "Shelf_1",
  "target": "Worker"
}

위치 이름:
- 1번 선반: Shelf_1
- 2번 선반: Shelf_2
- 3번 선반: Shelf_3
- 작업자: Worker
- 작업대: Workbench

물품 이름:
- 부품 박스: parts_box
- 공구 박스: tool_box
- 센서 박스: sensor_box

설명 없이 JSON 객체 하나만 출력한다.
"""


class NodeALLM(Node):
    def __init__(self):
        super().__init__("node_a_llm")

        self.pub = self.create_publisher(String, "/mission", 10)
        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(String, "/llm_command", self._on_command, 10)

        self._llm = self._init_groq()
        self._model = os.environ.get("GROQ_MODEL", "openai/gpt-oss-20b")

        self._status(f"Node A 시작 — LLM={'ON' if self._llm else 'OFF(폴백)'}")

    def _on_command(self, msg: String):
        text = msg.data.strip()
        self._status(f"명령 수신: '{text}'")

        mission = self._parse_with_llm(text)
        if mission is None:
            mission = self._parse_fallback(text)

        if mission is None:
            self._status(f"명령 해석 실패: '{text}'")
            return

        mission_json = json.dumps(mission, ensure_ascii=False)
        self.pub.publish(String(data=mission_json))

        self._status(f"미션 발행: {mission_json}")

    def _parse_with_llm(self, text: str):
        if self._llm is None:
            return None

        try:
            response = self._llm.chat.completions.create(
                model=self._model,
                temperature=0,
                response_format={"type": "json_object"},
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": text},
                ],
            )

            content = response.choices[0].message.content
            mission = json.loads(content)

            if self._valid_mission(mission):
                return mission

            self._status(f"LLM 응답 형식 오류: {mission}")
            return None

        except Exception as e:
            self._status(f"LLM 파싱 실패, 폴백 사용: {e}")
            return None

    def _parse_fallback(self, text: str):
        compact = text.replace(" ", "")
        upper = compact.upper()

        # 정지 명령
        stop_keywords = ["정지", "멈춰", "멈추", "스톱", "STOP"]
        if any(k in upper for k in stop_keywords):
            return {
                "task": "stop"
            }

        source = self._find_source(compact, upper)
        target = self._find_target(compact, upper)
        item = self._find_item(compact, upper)

        if source and target and item:
            return {
                "task": "pick_and_deliver",
                "item": item,
                "source": source,
                "target": target,
            }

        self._status(
            f"폴백 해석 실패: source={source}, target={target}, item={item}"
        )
        return None

    def _find_source(self, compact: str, upper: str):
        # source는 선반만 허용
        if "1번선반" in compact or "1번" in compact or "선반1" in compact or "SHELF_1" in upper:
            return "Shelf_1"
        if "2번선반" in compact or "2번" in compact or "선반2" in compact or "SHELF_2" in upper:
            return "Shelf_2"
        if "3번선반" in compact or "3번" in compact or "선반3" in compact or "SHELF_3" in upper:
            return "Shelf_3"
        return None

    def _find_target(self, compact: str, upper: str):
        if "작업자" in compact or "워커" in compact or "WORKER" in upper:
            return "Worker"
        if "작업대" in compact or "워크벤치" in compact or "WORKBENCH" in upper:
            return "Workbench"
        return None

    def _find_item(self, compact: str, upper: str):
        if "부품박스" in compact or "부품" in compact or "PARTS_BOX" in upper or "PARTSBOX" in upper:
            return "parts_box"
        if "공구박스" in compact or "공구" in compact or "TOOL_BOX" in upper or "TOOLBOX" in upper:
            return "tool_box"
        if "센서박스" in compact or "센서" in compact or "SENSOR_BOX" in upper or "SENSORBOX" in upper:
            return "sensor_box"
        return None

    @staticmethod
    def _valid_mission(mission):
        if not isinstance(mission, dict):
            return False

        task = mission.get("task")

        if task == "stop":
            return True

        if task != "pick_and_deliver":
            return False

        return (
            mission.get("item") in ["parts_box", "tool_box", "sensor_box"]
            and mission.get("source") in ["Shelf_1", "Shelf_2", "Shelf_3"]
            and mission.get("target") in ["Worker", "Workbench"]
        )

    def _init_groq(self):
        key = os.environ.get("GROQ_API_KEY", "").strip()
        if not key or key.startswith("your_"):
            return None

        try:
            from groq import Groq
            return Groq(api_key=key)
        except Exception:
            return None

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[A] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = NodeALLM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
