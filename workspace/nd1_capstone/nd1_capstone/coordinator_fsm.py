#!/usr/bin/env python3

import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


VALID_ITEMS = ["parts_box", "tool_box", "sensor_box"]
VALID_SOURCES = ["Shelf_1", "Shelf_2", "Shelf_3"]
VALID_TARGETS = ["Worker", "Workbench"]


class State(Enum):
    IDLE = auto()
    PLANNING = auto()
    NAV_TO_SHELF = auto()
    PICKING = auto()
    NAV_TO_WORKER = auto()
    PLACING = auto()
    DONE = auto()
    ERROR = auto()


class CoordinatorFSM(Node):
    def __init__(self):
        super().__init__("coordinator_fsm")

        self.declare_parameter("sim_mode", True)
        self.declare_parameter("tick_delay", 0.7)

        self.sim_mode = bool(self.get_parameter("sim_mode").value)
        self.tick_delay = float(self.get_parameter("tick_delay").value)

        self.pub_status = self.create_publisher(String, "/robot_status", 10)
        self.pub_nav = self.create_publisher(String, "/nav_request", 10)
        self.pub_grasp = self.create_publisher(String, "/grasp_request", 10)

        self.create_subscription(String, "/mission", self._on_mission, 10)
        self.create_subscription(Bool, "/nav_result", self._on_nav_result, 10)
        self.create_subscription(Bool, "/grasp_result", self._on_grasp_result, 10)

        self.state = State.IDLE
        self.mission = None
        self.current_item = None
        self.source = None
        self.target = None

        self._sim_timers = []

        self._status("Coordinator started")

    def _on_mission(self, msg: String):
        if self.state not in (State.IDLE, State.DONE, State.ERROR):
            self._status("busy: ignore new mission")
            return

        try:
            mission = json.loads(msg.data)
        except Exception as e:
            self._set_error(f"mission json error: {e}")
            return

        if not self._valid_mission(mission):
            self._set_error(f"invalid mission: {mission}")
            return

        task = mission.get("task")

        if task == "stop":
            self.state = State.DONE
            self._status("stop -> DONE")
            return

        self.mission = mission
        self.current_item = mission["item"]
        self.source = mission["source"]
        self.target = mission["target"]

        self.state = State.PLANNING
        self._status(
            f"mission received: task={task}, item={self.current_item}, "
            f"source={self.source}, target={self.target}"
        )

        self._enter_nav_to_shelf()

    def _valid_mission(self, mission):
        if not isinstance(mission, dict):
            return False

        task = mission.get("task")

        if task == "stop":
            return True

        if task != "pick_and_deliver":
            return False

        return (
            mission.get("item") in VALID_ITEMS
            and mission.get("source") in VALID_SOURCES
            and mission.get("target") in VALID_TARGETS
        )

    def _enter_nav_to_shelf(self):
        self.state = State.NAV_TO_SHELF
        self._status(f"NAV_TO_SHELF: target={self.source}")

        payload = {
            "target": self.source
        }
        self.pub_nav.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _enter_picking(self):
        self.state = State.PICKING
        self._status(f"PICKING: item={self.current_item}")

        payload = {
            "op": "grasp",
            "item": self.current_item
        }
        self.pub_grasp.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _enter_nav_to_worker(self):
        self.state = State.NAV_TO_WORKER
        self._status(f"NAV_TO_WORKER: target={self.target}")

        payload = {
            "target": self.target
        }
        self.pub_nav.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _enter_placing(self):
        self.state = State.PLACING
        self._status(f"PLACING: item={self.current_item}")

        payload = {
            "op": "place",
            "item": self.current_item
        }
        self.pub_grasp.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _on_nav_result(self, msg: Bool):
        ok = bool(msg.data)

        if self.state == State.NAV_TO_SHELF:
            if not ok:
                self._set_error("NAV_TO_SHELF failed")
                return

            self._status("NAV_TO_SHELF result: success")
            self._enter_picking()
            return

        if self.state == State.NAV_TO_WORKER:
            if not ok:
                self._set_error("NAV_TO_WORKER failed")
                return

            self._status("NAV_TO_WORKER result: success")
            self._enter_placing()
            return

        self._status(f"nav_result ignored in state={self.state.name}")

    def _on_grasp_result(self, msg: Bool):
        ok = bool(msg.data)

        if self.state == State.PICKING:
            if not ok:
                self._set_error("PICKING failed")
                return

            self._status("PICKING result: success")
            self._enter_nav_to_worker()
            return

        if self.state == State.PLACING:
            if not ok:
                self._set_error("PLACING failed")
                return

            self._status("PLACING result: success")
            self.state = State.DONE
            self._status("DONE")
            return

        self._status(f"grasp_result ignored in state={self.state.name}")

    def _simulate_nav_result(self, ok: bool):
        def callback():
            self._on_nav_result(Bool(data=ok))

        self._oneshot_timer(callback)

    def _simulate_grasp_result(self, ok: bool):
        def callback():
            self._on_grasp_result(Bool(data=ok))

        self._oneshot_timer(callback)

    def _oneshot_timer(self, callback):
        timer_holder = {"timer": None}

        def fire():
            timer = timer_holder["timer"]
            if timer is not None:
                timer.cancel()
                try:
                    self.destroy_timer(timer)
                except Exception:
                    pass

            callback()

        timer_holder["timer"] = self.create_timer(self.tick_delay, fire)
        self._sim_timers.append(timer_holder["timer"])

    def _set_error(self, text: str):
        self.state = State.ERROR
        self._status("ERROR: " + text)

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=f"[FSM:{self.state.name}] {text}"))


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorFSM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
