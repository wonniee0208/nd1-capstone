# ND1 캡스톤 — 4노드 통합 (Node A·B·C + Coordinator FSM)

> 📘 **학생 배포본(반골격)** — 노드의 핵심 로직은 `# TODO`로 비어 있습니다.
> **먼저 `학생_시작가이드.md`를 읽으세요.** (구현 맵·개발 순서·자가검증)

## 1. 아키텍처 (대칭 핸드셰이크)

```
/llm_command ─▶ Node A(LLM+폴백) ─/mission─▶ Coordinator(FSM)
                                              │  /nav_request   ─▶ Node B(Nav2) ─/nav_result─┐
                                              │  /grasp_request ─▶ Node C(IK)   ─/grasp_result┘
                                              └─ /robot_status (상태 로그)
```
- Coordinator는 **std_msgs만 의존** → Nav2/IK와 완전 디커플링. 학생이 노드별 독립 개발·sim 가능.
- 토픽 계약: nav_request `{x,y,yaw}` / grasp_request `{op:"grasp"|"place",x,y}` (둘 다 String JSON), 결과는 Bool.

## 2. 폴더 구조
```
workspace/nd1_capstone/
├─ package.xml  setup.py  setup.cfg
├─ launch/
│  ├─ bringup.launch.py     # 4노드 + 선택적 SLAM/Nav2 + 박스 인자 패스스루
│  └─ spawn_boxes.launch.py # 기본 월드에 box1 스폰 (world:= 인자)
├─ models/box1.sdf          # 파지 대상 박스 (Fortress SDF)
└─ nd1_capstone/
   ├─ node_a_llm.py         # LLM 해석 + 폴백        [담당: LLM]
   ├─ node_b_nav.py         # Nav2 래퍼             [담당: ROS2]
   ├─ node_c_grasp.py       # IK 파지/배치+텔레포트  [담당: 기구학]
   ├─ coordinator_fsm.py    # 중 ★★☆ FSM 조정(권장) [담당: 통합]
   ├─ linear_orchestrator.py# 하 ★☆☆ 선형 순차
   └─ llm_planner.py        # 상 ★★★ 멀티미션 플래너
```
> 실연동(sim_mode:=false)·박스 스폰은 `기본월드_박스세팅.md`, SLAM/map 프레임은 `실연동_검증_절차.md` 참조.

## 3. 기동
```bash
cp .env.example .env            # GROQ_API_KEY 입력(없으면 폴백 동작)
docker compose up --build
# noVNC http://localhost:8080 (pw: nd1capstone)
```
컨테이너 내부:
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch nd1_capstone bringup.launch.py sim_mode:=true
```

## 4. ★ 스모크 테스트 (sim_mode — B/C/Gazebo 불필요)
```bash
# 위 launch 실행 상태에서, 새 터미널:
source ~/ros2_ws/install/setup.bash
ros2 topic echo /robot_status &
ros2 topic pub --once /llm_command std_msgs/String '{data: "A구역 박스를 B구역으로 옮겨줘"}'
```
→ NAVIGATING→GRASPING→TRANSPORTING→PLACING→DONE 출력되면 통과.

## 5. 실연동 (sim_mode:=false)
선행: TurtleBot4 시뮬을 터미널1에서 기동.
⚠️ Nav2만으로는 `map` 프레임이 없어 costmap이 `Invalid frame ID "map"` 으로 막힌다.
**SLAM(또는 AMCL)으로 map→odom 공급원을 반드시 함께 띄운다.** (상세: `실연동_검증_절차.md`)
```bash
# 터미널1: 시뮬
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
# 터미널2: SLAM + Nav2 + 4노드 통합 (패치된 bringup)
ros2 launch nd1_capstone bringup.launch.py sim_mode:=false slam:=true nav2:=true
# (사전 맵 사용 시) slam:=true 대신 localization:=true map:=/경로/맵.yaml
```
- Node B → 실제 Nav2 이동(status==SUCCEEDED만 성공 처리)
- Node C → nd1_m7_ik IK 계산 + 텔레포트(ign service) 파지/배치
  - world_name/box_model 파라미터를 실제 월드에 맞게 지정 필요.
- map 프레임 확인: `ros2 run tf2_ros tf2_echo map base_link` 변환 출력되면 정상.

## 6. 핵심 제약·기준 (사실)
- **파지 y-offset ≥ 0.20**: y=0 순수 x축은 특이점 → IK 발산. Node C가 자동 클램프.
- 표준 구역 좌표 A(1.5,0.5)/B(2.5,−1.0)/C(0.5,2.0) — 모두 제약 만족.
- groq 모델: 2026-06-17 llama-3.x deprecated → 기본 `openai/gpt-oss-20b`(GROQ_MODEL).

## 7. 검증 완료 (이 환경 기준)
- 4개 노드 + launch + setup `py_compile` 통과
- 3DOF DLS IK 수치검증: y=0 발산(200회 err 0.05) / y≥0.20 수렴(6~8회 err<1e-4)
- 폴백 파서 + Pydantic: 5개 명령 정상 분기
- FSM 전이: pick_and_place 4단계 순서 / navigate 단축 / 재시도 후 FAILED 확인

> ⚠️ ROS2 통신부(rclpy·nav2_msgs)·Gazebo 텔레포트는 컨테이너에서만 실행/검증 가능.
> 위 수치·로직 검증은 통신·시뮬 외 핵심 알고리즘에 한정됨.

## 8. Node C 구현 기록

### 담당 파일

`workspace/nd1_capstone/nd1_capstone/node_c_grasp.py`

### 구현 내용

Node C는 Coordinator로부터 `/grasp_request`를 수신하여 파지 또는 배치 동작을 처리하고, 처리 결과를 `/grasp_result`로 반환하는 노드이다.

이번 구현에서는 다음 기능을 완료하였다.

1. `_solve_ik()` 구현
   - `nd1_m7_ik`의 `numerical_ik()` 호출
   - 팔 로컬 목표 좌표 `(x, y)`에 대한 3DOF 관절각 계산
   - IK 계산 실패 또는 예외 발생 시 안전하게 실패 처리
   - `sim_mode=True`에서는 fallback 관절각을 반환하도록 처리

2. `_teleport()` 구현
   - `sim_mode=True`에서는 Gazebo 호출 없이 성공 로그 처리
   - `sim_mode=False`에서는 `ign service` 기반 박스 제거 및 생성 명령 구성
   - `grasp` 요청 시 `/world/<world>/remove` 호출
   - `place` 요청 시 `/world/<world>/create` 호출
   - timeout, `ign` 명령 없음, 기타 예외 상황 처리

3. 특이점 회피 처리 유지
   - `Y_OFFSET_MIN = 0.20`
   - y-offset이 너무 작을 경우 IK 발산 가능성을 줄이기 위해 최소 offset을 보장한다.

### 검증 결과

문법 검사 통과:

```bash
python3 -m py_compile src/nd1_capstone/nd1_capstone/node_c_grasp.py

- `python3 scripts/smoke_test.py` 실행 결과 `SMOKE TEST: PASS` 확인

### 프로젝트 시나리오 가정

본 프로젝트는 전자부품 물류창고를 가정한다.  
창고 내 상자는 `electronic_parts_box`로 정의하며, 상자 안에는 충격에 민감한 소형 전자부품이 들어 있다고 설정한다.

따라서 Node C는 `/grasp_request`에서 다음 속성을 받을 수 있도록 구현하였다.

```json
{
  "op": "grasp",
  "x": 1.5,
  "y": 0.5,
  "item": "electronic_parts_box",
  "fragile": true
}