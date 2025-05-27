# 🤖 DSR MoveIt-Gazebo 연동 실습 정리



## 1️⃣ MoveIt 구조

### 액션: `/dsr_moveit_controller/follow_joint_trajectory`

* **`/moveit_simple_controller_manager` 노드**

  * MoveIt에서 여러 로봇 컨트롤러를 관리하는 **컨트롤러 관리자 노드**
  * 액션 클라이언트로 동작, 명령(trajectory goal)을 보냄
    → **명령 내리는 쪽(클라이언트)**

* **`/dsr_moveit_controller` 노드**

  * 실제 로봇 또는 시뮬레이터의 조인트를 제어하는 **컨트롤러 노드**
  * 액션 서버로 동작, trajectory 명령을 받아 실제 동작 수행
    → **명령을 받아 실행하는 쪽(서버)**

* **동작 흐름**

  * MoveIt에서 **Plan & Execute** 실행 시

    * `/moveit_simple_controller_manager`가 trajectory goal을
    * `/dsr_moveit_controller`로 전송
    * 실제(또는 가상) 로봇이 해당 trajectory대로 움직임



## 2️⃣ Gazebo 구조

### 토픽: `/dsr01/gz/dsr_position_controller/commands`

* **`/dsr01/gz/dsr_position_controller` 노드**

  * 조인트 위치 제어 명령을 **토픽 구독** 방식으로 받음
  * Gazebo 상에서 각 조인트를 명령대로 움직임

---

## 3️⃣ MoveIt-Gazebo 동기화 목표

* MoveIt의 Plan & Execute 결과를 **Gazebo 시뮬레이터에도 동기 반영**
* 실제 로봇과 **Gazebo 시뮬로봇이 동시에 동일 궤적**을 따라 움직이도록 함



## 4️⃣ 기존 구조의 한계

* MoveIt은 **액션 서버**(`/dsr_moveit_controller/follow_joint_trajectory`)에만 goal 전송
  → **Gazebo는 이 액션 서버와 무관**

* `/dsr_moveit_controller/joint_trajectory` 등 topic에는 별도 publish가 없음

* Gazebo position controller(`/dsr01/gz/dsr_position_controller`)는
  **명령 토픽**(`/dsr01/gz/dsr_position_controller/commands`, `std_msgs/Float64MultiArray`)만 구독

* **결론:**

  * 액션 서버를 topic subscriber로 “가로채기” 불가
  * 상태 토픽(controller\_state 등)은 단순 상태 보고용, relay에 부적합



## 5️⃣ 해결 방법: 중계(Relay) 노드 도입

### 목적

* MoveIt trajectory goal을 받아 **실제 로봇 + Gazebo** 모두로 동시 분배
* MoveIt과 Gazebo를 연동하는 **중간 액션 서버 노드** 구현

### 구조 및 동작

1. **중계 노드가 액션 서버(`/dsr_moveit_controller/follow_joint_trajectory`)로 동작**
2. MoveIt에서 trajectory goal을 받음
3. goal에서 \*\*최종 포인트(최종 joint position)\*\*만 추출
4. Gazebo 명령 토픽(`/dsr01/gz/dsr_position_controller/commands`) 형식(`Float64MultiArray`)으로 변환하여 publish
5. 필요에 따라 여러 컨트롤러에도 동시에 분배 가능



## 6️⃣ 결론

* **MoveIt trajectory goal**
  → **중계(액션 서버) 노드**
  → **Gazebo 명령 topic 변환**
  → **Gazebo 시뮬로봇이 실시간 동기 동작**

* 중계 노드에 실제로봇/가상로봇 동시 제어나 추가 기능도 구현 가능



> 본 구조는 MoveIt과 Gazebo의 명령 체계 차이로 인한 **동기화 문제**를 해결하는 표준적이고 합리적인 접근 방식이다.
> 핵심은 **액션-토픽 변환 중계**이며, 실환경 확장/응용도 용이하다.
