#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════
#  ND1 캡스톤 — gz_ros2_control update_rate 불일치 패치
#
#  [증상]
#    joint_state_broadcaster / diffdrive_controller spawner가 반복
#    타임아웃(list_controllers / load_controller) 후 프로세스가 죽고,
#    이어서 "A controller named 'xxx' was already loaded inside the
#    controller manager" 에러가 나며 diffdrive_controller가 끝내 active
#    상태가 되지 않음.
#
#  [근본 원인 — Gazebo(ign gazebo) 콘솔 로그에서 실측 확인됨]
#    [gz_ros2_control]: Desired controller update period (0.001 s) is
#    faster than the gazebo simulation period (0.003 s).
#
#    irobot_create_control/config/control.yaml 의
#      controller_manager.ros__parameters.update_rate: 1000   (1ms 주기)
#    가 warehouse.sdf 의
#      <physics><max_step_size>0.003</max_step_size></physics>  (≈333Hz)
#    보다 빠르게 설정되어 있어, controller_manager 내부 update 루프와
#    서비스 응답(list_controllers/load_controller 등)이 불안정해짐.
#    이로 인해 spawner는 "실패"로 보고하지만 서버 내부적으로는 이미 로드가
#    되어 있는 상태 불일치가 발생 → 재시도 시 중복 로드 에러로 죽음.
#
#  [조치]
#    update_rate 를 Gazebo 물리 스텝(≈333Hz)보다 낮은 값(기본 300Hz)으로
#    낮춤. 대상 파일은 ROS 시스템 패키지 경로
#      /opt/ros/humble/share/irobot_create_control/config/control.yaml
#    이며, 이 값은 gz_ros2_control 플러그인이 로봇 URDF/xacro의
#    <ros2_control> 태그에 지정된 경로를 통해 직접 읽기 때문에, 이 파일을
#    직접 고치는 것이 유일한 실질적 방법이다(nd1_capstone 쪽 launch
#    파라미터 오버레이로는 이 값에 영향을 줄 수 없음 — controller_manager
#    자체가 이 파일을 gz_ros2_control 플러그인 초기화 시점에 읽어들임).
#
#    ⚠️ 시스템 패키지 파일을 직접 수정하므로, 컨테이너/이미지가 재빌드되면
#       이 패치는 초기화된다. 매 컨테이너 기동 시(터미널1 실행 전) 다시
#       실행해야 함. launch_sim_patched.sh 가 이 스크립트를 자동으로 먼저
#       호출한다.
#
#    ⚠️ /opt/ros/humble/... 경로는 보통 root 소유라 일반 사용자(ubuntu 등)로는
#       쓰기 권한이 없다. 이 스크립트는 대상 파일이 쓰기 불가능하면 자동으로
#       sudo를 붙여 재시도한다(비밀번호 없는 sudo 환경이면 그대로 진행되고,
#       비밀번호가 걸려 있으면 터미널에서 입력 프롬프트가 뜬다).
#
#  [사용]
#    ./fix_controller_update_rate.sh [목표_Hz(기본 300)]
#
#  [검증 방법 — 패치 후 재확인]
#    ros2 param get /controller_manager update_rate   (해당 서비스 노출 안 될 수 있음)
#    ign gazebo 콘솔 로그에서 "Desired controller update period" 경고가
#    다시 뜨는지 확인 (안 뜨면 정상)
# ════════════════════════════════════════════════════════════════
set -euo pipefail

TARGET_HZ="${1:-300}"
CONTROL_YAML="/opt/ros/humble/share/irobot_create_control/config/control.yaml"
BACKUP="${CONTROL_YAML}.orig"

# 이 파일/디렉터리에 쓰기 권한이 있으면 그냥 실행, 없으면 sudo를 앞에 붙여서 실행.
# (sudo가 없는 환경이면 명확한 에러로 즉시 알려줌 — 조용히 실패하지 않도록)
run_priv() {
  if [ -w "$(dirname "$CONTROL_YAML")" ] && { [ ! -e "$CONTROL_YAML" ] || [ -w "$CONTROL_YAML" ]; }; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    echo "[ERROR] $CONTROL_YAML 에 쓰기 권한이 없고 sudo도 없음." >&2
    echo "        root로 재실행하거나, 이 컨테이너에 sudo를 설치해야 함." >&2
    exit 1
  fi
}

if ! [[ "$TARGET_HZ" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] 목표 Hz는 양의 정수여야 함: '${TARGET_HZ}'" >&2
  exit 1
fi

if [ ! -f "$CONTROL_YAML" ]; then
  echo "[ERROR] 파일 없음: $CONTROL_YAML" >&2
  echo "        ROS 환경(source /opt/ros/humble/setup.bash)이 로드된 컨테이너 안에서 실행하세요." >&2
  exit 1
fi

# 최초 1회만 원본 백업 (재실행해도 원본이 덮어써지지 않도록)
if [ ! -f "$BACKUP" ]; then
  run_priv cp "$CONTROL_YAML" "$BACKUP"
  echo "[INFO] 원본 백업 생성: $BACKUP"
fi

CURRENT="$(grep -oP 'update_rate:\s*\K[0-9]+' "$CONTROL_YAML" | head -1 || true)"
if [ -z "$CURRENT" ]; then
  echo "[ERROR] update_rate 라인을 찾지 못함. 패키지 버전이 달라 파일 구조가" >&2
  echo "        바뀌었을 수 있음 — 수동 확인 필요: $CONTROL_YAML" >&2
  exit 1
fi

if [ "$CURRENT" = "$TARGET_HZ" ]; then
  echo "[SKIP] 이미 update_rate: ${TARGET_HZ} Hz 로 설정되어 있음. 변경 없음."
  exit 0
fi

# update_rate 뒤의 숫자만 치환 (주석 "# Hz" 등 나머지는 그대로 유지)
run_priv sed -i -E "s/(update_rate:[[:space:]]*)[0-9]+/\1${TARGET_HZ}/" "$CONTROL_YAML"

NEW="$(grep -oP 'update_rate:\s*\K[0-9]+' "$CONTROL_YAML" | head -1 || true)"
if [ "$NEW" != "$TARGET_HZ" ]; then
  echo "[ERROR] 패치 실패(치환 결과 불일치). 원본으로 복원함." >&2
  run_priv cp "$BACKUP" "$CONTROL_YAML"
  exit 1
fi

echo "[OK] update_rate: ${CURRENT} Hz → ${TARGET_HZ} Hz 로 패치 완료."
echo "     대상 파일: ${CONTROL_YAML}"
echo "     원복하려면: cp '${BACKUP}' '${CONTROL_YAML}'"
