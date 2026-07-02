#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════
#  ND1 캡스톤 — 터미널1(Gazebo) 대체 기동 스크립트
#
#  기존 "실연동_검증_절차.md" 의 터미널1 명령을 그대로 실행하면 아래 두
#  문제가 재현된다(모두 실측/확인됨, 세부 원인은 문서 T9 참조):
#
#   1) world:=<이름> 으로 넘겨도 turtlebot4_ignition_bringup 내부의
#      ign_args → ros_gz_sim/ign_gazebo.launch.py 전달 체인이 끊겨서
#      world 인자가 증발하고, Gazebo가 항상 빈 'default' world로 뜬다.
#      → 라이다 /scan 이 전부 .inf, SLAM 맵이 항상 비어 map 프레임 부재.
#      → 해결: world SDF 절대경로를 gz_args 맨 앞에 직접 넣어서 우회.
#
#   2) irobot_create_control/config/control.yaml 의
#      controller_manager.update_rate(1000Hz)가 Gazebo 물리 스텝
#      (warehouse.sdf max_step_size=0.003s ≈333Hz)보다 빨라
#      joint_state_broadcaster/diffdrive_controller spawner가 타임아웃 후
#      중복 로드 에러로 죽는다.
#      → 해결: fix_controller_update_rate.sh 로 update_rate를 낮춘 뒤 기동.
#
#  ⚠️ 두 문제 모두 시스템 패키지 파일(/opt/ros/humble/...) 자체의 동작이라
#     컨테이너 재빌드 시 초기화된다. 매 컨테이너 기동 시 터미널1을
#     "그냥" ros2 launch로 실행하지 말고, 이 스크립트로 대체할 것.
#
#  [사용]
#    ./launch_sim_patched.sh                 # world=warehouse (기본)
#    ./launch_sim_patched.sh maze             # 다른 월드 (해당 .sdf가
#                                              # turtlebot4_ignition_bringup/worlds
#                                              # 에 있는 경우만)
#
#  [기동 후 검증]
#    ign topic -l | grep -m1 world           # /world/<world>/... 로 나와야 함
#                                             # (default면 world sdf 로드 실패)
#    ros2 topic hz /scan                      # 값이 찍혀야 정상
# ════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-warehouse}"
TARGET_HZ="${CONTROLLER_UPDATE_HZ:-300}"

TB4_IGN_SHARE="/opt/ros/humble/share/turtlebot4_ignition_bringup"
WORLD_SDF="${TB4_IGN_SHARE}/worlds/${WORLD}.sdf"

# ── ROS 환경 로드 ──────────────────────────────────────────────
# ⚠️ ROS의 setup.bash는 내부적으로 미설정 변수(AMENT_TRACE_SETUP_FILES 등)를
#    참조하는 방식으로 짜여 있어 `set -u`(nounset)와 호환되지 않는다.
#    source하는 동안만 -u를 잠시 해제한다.
set +u
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ros2_ws/install/setup.bash
fi
set -u

# ── world sdf 존재 확인 ────────────────────────────────────────
if [ ! -f "$WORLD_SDF" ]; then
  echo "[ERROR] world SDF 없음: $WORLD_SDF" >&2
  echo "        설치된 world 목록:" >&2
  find "${TB4_IGN_SHARE}/worlds" -maxdepth 1 -name "*.sdf" 2>/dev/null >&2
  exit 1
fi

# ── controller_manager update_rate 패치 ───────────────────────
"${SCRIPT_DIR}/fix_controller_update_rate.sh" "${TARGET_HZ}"

# ── Gazebo 기동 (world SDF 절대경로를 gz_args에 직접 지정) ─────
echo "[INFO] Gazebo 기동: world=${WORLD} (${WORLD_SDF})"
exec ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  model:=lite world:="${WORLD}" rviz:=false \
  gz_args:="${WORLD_SDF} -s -r --render-engine ogre --headless-rendering"
