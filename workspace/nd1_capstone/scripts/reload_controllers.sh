#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════
#  ND1 캡스톤 — controller_manager 컨트롤러 강제 재적재 (폴백 전용)
#
#  ⚠️ 이건 근본 수정이 아니라 1회성 복구 스크립트다.
#     정상적인 흐름은 launch_sim_patched.sh 로 터미널1을 기동해서
#     update_rate 불일치 자체를 없애는 것.
#
#  이 스크립트가 필요한 경우:
#    - update_rate 패치를 적용했는데도(fix_controller_update_rate.sh 실행 후)
#      CPU 과부하(load average 과다) 등 다른 이유로 spawner가 여전히
#      list_controllers/load_controller 타임아웃을 내는 경우.
#    - 즉, 매 기동마다 이 스크립트를 반복 실행해야 한다면 update_rate
#      패치가 실제로 적용됐는지, `top`으로 load average를 먼저 재점검할 것.
#
#  [사용 — Gazebo(터미널1)와 nd1_capstone bringup(터미널2)이 이미 뜬 상태에서]
#    ./reload_controllers.sh
# ════════════════════════════════════════════════════════════════
set -euo pipefail

CONTROL_YAML="/opt/ros/humble/share/irobot_create_control/config/control.yaml"

echo "[1/4] controller_manager 응답 확인 (최대 15초 대기)..."
if ! timeout 15 ros2 service call /controller_manager/list_controllers \
      controller_manager_msgs/srv/ListControllers > /tmp/nd1_lc_before.txt 2>&1; then
  echo "[ERROR] controller_manager가 응답하지 않음." >&2
  echo "        gz_ros2_control 플러그인 자체가 아직 안 떴을 수 있음 — Gazebo 콘솔 로그 확인 필요." >&2
  cat /tmp/nd1_lc_before.txt >&2
  exit 1
fi
cat /tmp/nd1_lc_before.txt

echo
echo "[2/4] diffdrive_controller unload (로드돼 있으면 제거, 없으면 무시)..."
ros2 service call /controller_manager/unload_controller \
  controller_manager_msgs/srv/UnloadController "{name: 'diffdrive_controller'}" || true

echo "[3/4] joint_state_broadcaster unload (로드돼 있으면 제거, 없으면 무시)..."
ros2 service call /controller_manager/unload_controller \
  controller_manager_msgs/srv/UnloadController "{name: 'joint_state_broadcaster'}" || true

echo
echo "[4/4] 넉넉한 타임아웃(60s)으로 순서대로 재적재..."
echo "      - joint_state_broadcaster 먼저"
ros2 run controller_manager spawner joint_state_broadcaster \
  -c controller_manager --controller-manager-timeout 60

echo "      - diffdrive_controller (control.yaml 파라미터 포함)"
ros2 run controller_manager spawner diffdrive_controller \
  -c controller_manager --param-file "${CONTROL_YAML}" \
  --controller-manager-timeout 60

echo
echo "[DONE] 최종 상태 (양쪽 다 'active'여야 정상):"
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers
