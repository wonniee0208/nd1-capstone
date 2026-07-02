# ND1 캡스톤 — 4노드 통합 (Node A·B·C + Coordinator FSM)

> 📘 **학생 배포본 기반 통합 구현본** — Node A·B·C와 Coordinator FSM을 공통 JSON 토픽 계약에 맞춰 연결했습니다.
> **먼저 `학생_시작가이드.md`를 읽으세요.** (구현 맵·개발 순서·자가검증)

## 1. 아키텍처 (대칭 핸드셰이크)

```text
/llm_command ─▶ Node A(LLM+폴백) ─/mission─▶ Coordinator(FSM)
                                              │  /nav_request   ─▶ Node B(Nav2) ─/nav_result─┐
                                              │  /grasp_request ─▶ Node C(IK)   ─/grasp_result┘
                                              └─ /robot_status (상태 로그)
```

* Coordinator는 **std_msgs만 의존** → Nav2/IK와 완전 디커플링. 학생이 노드별 독립 개발·sim 가능.
* 이번 통합에서는 창고 피킹 보조 로봇 시나리오에 맞춰 토픽 계약을 아래 형식으로 통일하였다.
* 토픽 계약:

  * `/mission`: `{task,item,source,target}`
  * `/nav_request`: `{target}`
  * `/grasp_request`: `{op,item}`
  * `/nav_result`, `/grasp_result`: `std_msgs/Bool`

### 공통 JSON 예시

#### `/mission`

```json
{
  "task": "pick_and_deliver",
  "item": "parts_box",
  "source": "Shelf_1",
  "target": "Worker"
}
```

#### `/nav_request`

```json
{
  "target": "Shelf_1"
}
```

#### `/grasp_request`

```json
{
  "op": "grasp",
  "item": "parts_box"
}
```

```json
{
  "op": "place",
  "item": "parts_box"
}
```

---

## 2. 폴더 구조

```text
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

---

## 3. 기동

```bash
cp .env.example .env            # GROQ_API_KEY 입력(없으면 폴백 동작)
docker-compose up -d
# noVNC http://localhost:8080 (pw: nd1capstone)
```

컨테이너 내부:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch nd1_capstone bringup.launch.py sim_mode:=true
```

---

## 4. ★ 스모크 테스트 (sim_mode — B/C/Gazebo 불필요)

```bash
# 위 launch 실행 상태에서, 새 터미널:
docker exec -it nd1_capstone_dev bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros2_ws
source install/setup.bash

ros2 topic pub --once /llm_command std_msgs/msg/String "{data: '1번 선반에 있는 부품 박스를 작업자에게 가져다줘'}"
```

정상 동작 시 아래 흐름이 출력되면 통과이다.

```text
NAV_TO_SHELF → PICKING → NAV_TO_WORKER → PLACING → DONE
```

실제 확인된 로그 흐름:

```text
[node_a_llm] 명령 수신
[node_a_llm] 미션 발행: {"task": "pick_and_deliver", "item": "parts_box", "source": "Shelf_1", "target": "Worker"}

[coordinator_fsm] mission received
[coordinator_fsm] NAV_TO_SHELF

[node_b_nav] 이동 요청 수신: target=Shelf_1
[node_b_nav] 이동 결과: 성공

[coordinator_fsm] PICKING

[node_c_grasp] 파지 요청 수신: op=grasp, item=parts_box
[node_c_grasp] 파지 결과: 성공

[coordinator_fsm] NAV_TO_WORKER

[node_b_nav] 이동 요청 수신: target=Worker
[node_b_nav] 이동 결과: 성공

[coordinator_fsm] PLACING

[node_c_grasp] 파지 요청 수신: op=place, item=parts_box
[node_c_grasp] 파지 결과: 성공

[coordinator_fsm] DONE
```

---

## 5. 실연동 (sim_mode:=false)

선행: TurtleBot4 시뮬을 터미널1에서 기동.

⚠️ Nav2만으로는 `map` 프레임이 없어 costmap이 `Invalid frame ID "map"`으로 막힌다.
**SLAM(또는 AMCL)으로 map→odom 공급원을 반드시 함께 띄운다.**
상세 내용은 `실연동_검증_절차.md` 참조.

```bash
# 터미널1: 시뮬
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py

# 터미널2: SLAM + Nav2 + 4노드 통합
ros2 launch nd1_capstone bringup.launch.py sim_mode:=false slam:=true nav2:=true

# 사전 맵 사용 시
ros2 launch nd1_capstone bringup.launch.py sim_mode:=false localization:=true map:=/경로/맵.yaml
```

* Node B → 실제 Nav2 이동 구현 시 `/nav_request`의 `target`을 내부 좌표로 변환하여 이동한다.
* Node C → 실제 로봇팔/IK 구현 시 `/grasp_request`의 `op`, `item` 계약은 유지한다.
* map 프레임 확인:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

변환 출력이 나오면 정상이다.

---

## 6. 핵심 제약·기준 (사실)

* 토픽 이름은 변경하지 않는다.

  * `/llm_command`
  * `/mission`
  * `/nav_request`
  * `/nav_result`
  * `/grasp_request`
  * `/grasp_result`
  * `/robot_status`
* `/mission` JSON 형식은 `task`, `item`, `source`, `target`을 사용한다.
* `/nav_request` JSON 형식은 `target`을 사용한다.
* `/grasp_request` JSON 형식은 `op`, `item`을 사용한다.
* 결과 토픽 `/nav_result`, `/grasp_result`는 `std_msgs/Bool`을 사용한다.
* 내부 위치 이름:

  * `Shelf_1`
  * `Shelf_2`
  * `Shelf_3`
  * `Worker`
  * `Workbench`
* 내부 물품 이름:

  * `parts_box`
  * `tool_box`
  * `sensor_box`
* groq 모델: 2026-06-17 llama-3.x deprecated → 기본 `openai/gpt-oss-20b`(GROQ_MODEL).
* GROQ_API_KEY가 없으면 Node A는 폴백 파서로 동작한다.

---

## 7. 검증 완료 (이 환경 기준)

* 4개 노드 launch 실행 확인
* Node A 폴백 파서 동작 확인
* `/llm_command` → `/mission` JSON 발행 확인
* Coordinator FSM이 `task`, `item`, `source`, `target` 기반 미션 처리 확인
* Node B가 `/nav_request`의 `target`을 수신하여 이동 성공 결과 반환 확인
* Node C가 `/grasp_request`의 `op`, `item`을 수신하여 파지/배치 성공 결과 반환 확인
* `sim_mode:=true` 환경에서 전체 상태 전이 완료 확인

실제 확인된 FSM 흐름:

```text
PLANNING
→ NAV_TO_SHELF
→ PICKING
→ NAV_TO_WORKER
→ PLACING
→ DONE
```

> ⚠️ ROS2 통신부(rclpy·nav2_msgs)·Gazebo 텔레포트는 컨테이너에서만 실행/검증 가능.
> 현재 검증은 `sim_mode:=true` 기준의 4노드 통합 흐름 확인에 해당한다.

---

## 8. 통합 구현 기록

### 담당 파일

이번 통합 작업에서 수정한 파일은 다음과 같다.

```text
workspace/nd1_capstone/nd1_capstone/node_a_llm.py
workspace/nd1_capstone/nd1_capstone/coordinator_fsm.py
workspace/nd1_capstone/nd1_capstone/node_b_nav.py
workspace/nd1_capstone/nd1_capstone/node_c_grasp.py
```

### 구현 목적

기존 배포본은 `action`, `pick_x`, `pick_y`, `place_x`, `place_y` 중심의 좌표 기반 명령 구조를 사용하고 있었다.
이번 프로젝트에서는 창고 피킹 보조 로봇 시나리오에 맞게 노드 간 JSON 형식을 아래 기준으로 통일하였다.

```json
{
  "task": "pick_and_deliver",
  "item": "parts_box",
  "source": "Shelf_1",
  "target": "Worker"
}
```

이 형식을 기준으로 Node A, Coordinator, Node B, Node C가 서로 같은 토픽 계약을 사용하도록 수정하였다.

---

### 8.1 Node A 구현 내용

Node A는 `/llm_command`로 사용자 자연어 명령을 수신하고, 이를 `/mission` JSON으로 변환한다.

예시 명령:

```text
1번 선반에 있는 부품 박스를 작업자에게 가져다줘
```

발행되는 `/mission`:

```json
{
  "task": "pick_and_deliver",
  "item": "parts_box",
  "source": "Shelf_1",
  "target": "Worker"
}
```

구현 내용:

1. Groq API 사용 가능 시 LLM 기반 JSON 파싱 구조 유지
2. API 키가 없거나 LLM이 실패할 경우 폴백 파서 동작
3. 폴백 파서에서 자연어 표현을 내부 이름으로 변환

   * `1번 선반` → `Shelf_1`
   * `2번 선반` → `Shelf_2`
   * `3번 선반` → `Shelf_3`
   * `작업자` → `Worker`
   * `작업대` → `Workbench`
   * `부품 박스` → `parts_box`
   * `공구 박스` → `tool_box`
   * `센서 박스` → `sensor_box`
4. 해석 성공 시 `/mission`에 공통 JSON 형식으로 발행
5. 해석 실패 시 잘못된 미션을 발행하지 않고 로그 출력

---

### 8.2 Coordinator FSM 구현 내용

Coordinator는 `/mission`을 수신하고, 미션 내용을 검증한 뒤 Node B와 Node C에 순차적으로 요청을 보낸다.

수신하는 `/mission` 형식:

```json
{
  "task": "pick_and_deliver",
  "item": "parts_box",
  "source": "Shelf_1",
  "target": "Worker"
}
```

FSM 상태 흐름:

```text
IDLE
→ PLANNING
→ NAV_TO_SHELF
→ PICKING
→ NAV_TO_WORKER
→ PLACING
→ DONE
```

구현 내용:

1. `/mission` JSON 파싱
2. `task`, `item`, `source`, `target` 필드 검증
3. `source` 위치로 이동 요청 발행
4. 파지 요청 발행
5. `target` 위치로 이동 요청 발행
6. 배치 요청 발행
7. 모든 단계 성공 시 `DONE` 상태 전이
8. 오류 발생 시 `ERROR` 상태 전이

Coordinator가 발행하는 `/nav_request` 예시:

```json
{
  "target": "Shelf_1"
}
```

Coordinator가 발행하는 `/grasp_request` 예시:

```json
{
  "op": "grasp",
  "item": "parts_box"
}
```

```json
{
  "op": "place",
  "item": "parts_box"
}
```

---

### 8.3 Node B 구현 내용

Node B는 Coordinator로부터 `/nav_request`를 수신하고, 이동 결과를 `/nav_result`로 반환한다.

수신하는 `/nav_request` 형식:

```json
{
  "target": "Shelf_1"
}
```

구현 내용:

1. `/nav_request` JSON 파싱
2. `target` 값 검증
3. target 이름을 내부 좌표로 매핑
4. `sim_mode=True`에서는 실제 Nav2 이동 없이 성공 반환
5. 이동 성공 시 `/nav_result`에 `true` 발행
6. target이 유효하지 않으면 `/nav_result`에 `false` 발행

Node B 내부 target 좌표 매핑:

```text
Shelf_1   → (1.5, 0.5, 0.0)
Shelf_2   → (2.5, -1.0, 0.0)
Shelf_3   → (0.5, 2.0, 0.0)
Worker    → (0.0, 0.0, 0.0)
Workbench → (1.0, 0.0, 0.0)
```

---

### 8.4 Node C 구현 내용

Node C는 Coordinator로부터 `/grasp_request`를 수신하고, 파지 또는 배치 결과를 `/grasp_result`로 반환한다.

기본 `/grasp_request` 형식은 다음과 같다.

```json
{
  "op": "grasp",
  "x": 1.5,
  "y": 0.5,
  "item": "electronic_parts_box",
  "fragile": true
}
```

배치 요청 예시는 다음과 같다.

```json
{
  "op": "place",
  "x": 0.0,
  "y": 0.0,
  "item": "electronic_parts_box",
  "fragile": true
}
```

구현 내용:

* `/grasp_request` JSON 파싱
* `op` 값 검증

  * `grasp`
  * `place`
* `item` 값 처리

  * `parts_box`는 통합 테스트용 별칭으로 허용
  * 본 프로젝트에서는 `electronic_parts_box`, 즉 충격에 민감한 전자부품 상자로 해석
* `fragile=true`인 경우 충격 최소화 저속 파지 모드 로그 출력
* `x`, `y` 좌표가 있는 경우 해당 좌표를 기준으로 파지/배치 처리
* 작업 가능 범위를 벗어난 좌표 요청은 실패 처리
* `nd1_m7_ik` 기반 IK 계산 수행
* IK 계산 결과를 `/robot_status` 로그로 출력
* `sim_mode=True`에서는 Gazebo 텔레포트 동작을 성공으로 가정하여 통합 흐름 검증 가능
* `sim_mode=True`에서 Coordinator가 `{op,item}`만 보내는 경우에는 기본 좌표를 사용하여 통합 테스트가 멈추지 않도록 처리

이번 Node C 수정은 기존 IK/텔레포트 구조를 제거한 것이 아니라, Coordinator와 Node C 사이의 `/grasp_request` 형식 차이를 흡수하기 위한 호환성 수정이다. 최종적으로는 `op`, `x`, `y`, `item`, `fragile` 형식을 권장하며, `op`, `item`만 들어오는 요청은 sim_mode 통합 테스트용 fallback으로 처리한다.

---

### 8.5 통합 검증 결과

문법 검사:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile workspace/nd1_capstone/nd1_capstone/*.py
```

컨테이너 내부 빌드:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

통합 실행:

```bash
ros2 launch nd1_capstone bringup.launch.py sim_mode:=true
```

명령 발행:

```bash
ros2 topic pub --once /llm_command std_msgs/msg/String "{data: '1번 선반에 있는 부품 박스를 작업자에게 가져다줘'}"
```

확인된 결과:

```text
[node_a_llm] 미션 발행: {"task": "pick_and_deliver", "item": "parts_box", "source": "Shelf_1", "target": "Worker"}

[coordinator_fsm] mission received: task=pick_and_deliver, item=parts_box, source=Shelf_1, target=Worker
[coordinator_fsm] NAV_TO_SHELF: target=Shelf_1

[node_b_nav] 이동 요청 수신: target=Shelf_1
[node_b_nav] 이동 결과: 성공

[coordinator_fsm] PICKING: item=parts_box

[node_c_grasp] 파지 요청 수신: op=grasp, item=parts_box
[node_c_grasp] 파지 결과: 성공

[coordinator_fsm] NAV_TO_WORKER: target=Worker

[node_b_nav] 이동 요청 수신: target=Worker
[node_b_nav] 이동 결과: 성공

[coordinator_fsm] PLACING: item=parts_box

[node_c_grasp] 파지 요청 수신: op=place, item=parts_box
[node_c_grasp] 파지 결과: 성공

[coordinator_fsm] DONE
```

최종적으로 `sim_mode:=true` 환경에서 사용자 명령 입력부터 이동, 파지, 운반, 배치, 완료까지 전체 4노드 통합 흐름이 정상적으로 완료되었다.

---

### 9. 프로젝트 시나리오 가정

본 프로젝트는 전자부품 물류창고에서 자연어 명령을 받은 로봇이 충격에 민감한 전자부품 상자를 안전하게 운반하는 시스템을 가정한다.

사용자는 자연어로 물품 운반 명령을 내리고, 로봇은 지정된 선반에서 `electronic_parts_box`를 파지한 뒤 작업자 또는 작업대로 전달한다. 해당 상자는 내부에 충격에 민감한 소형 전자부품이 들어 있으므로 `fragile=true` 속성을 가진다.

예시 명령:

```text
1번 선반에 있는 전자부품 상자를 작업자에게 가져다줘
````

이 명령은 다음 내부 미션으로 변환된다.

```json
{
  "task": "pick_and_deliver",
  "item": "electronic_parts_box",
  "source": "Shelf_1",
  "target": "Worker",
  "fragile": true
}
```

Coordinator는 다음 순서로 미션을 수행한다.

```text
Shelf_1로 이동
→ electronic_parts_box 파지
→ Worker 위치로 이동
→ electronic_parts_box 배치
→ DONE
```

`parts_box`는 통합 테스트 과정에서 사용한 별칭이며, 최종 프로젝트 설명에서는 `electronic_parts_box`와 동일한 전자부품 상자로 해석한다.

```

---

## 10. Git 작업 규칙

작업 시작 전에는 항상 최신 내용을 받아온다.

```bash
git pull origin main
```

수정 후에는 담당 파일만 add한다.

```bash
git status
git add 수정한파일경로
git commit -m "작업 내용"
git push origin main
```

주의사항:

```text
.env 파일은 절대 GitHub에 올리지 않는다.
API Key, 비밀번호, 토큰은 커밋하지 않는다.
담당 파일 외 수정이 필요하면 팀원에게 먼저 공유한다.
토픽 이름과 JSON 키 이름은 임의로 바꾸지 않는다.
```
