"""펑션베이 시뮬레이터 백엔드 브리지.

펑션베이는 ros2_control 하드웨어 플러그인을 제공하지 않고 **ROS 2 토픽으로만**
연동한다. 그래서 이 백엔드에는 `controller_manager` / `joint_state_broadcaster` /
spawner 가 없고, 그 자리를 이 서브패키지의 브리지 노드들이 대신한다.

- :mod:`joint_state_fusion_node` — 시뮬레이터의 관절 보고를 `/joint_states` 로
  변환한다 (이름 없는 배열에 이름을 부여).
- :mod:`readiness_gate_node` — spawner 가 없으므로 "첫 관절 보고 수신"을 기동
  완료 신호로 삼는다.

설계 배경: ``docs/simulation/multi_simulator_backend_design.md`` §6.3 (B) 토픽 브리지.
"""
