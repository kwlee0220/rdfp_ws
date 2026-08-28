# robot_control

**ROS 2 로봇 제어 계층.** MoveIt2 클라이언트, 카메라, scene 상태 발행, 그리고 Panda
bringup launch 를 담는다. 학습 데이터 수집(`rdfp`)과 무관하게 **단독으로** 쓸 수
있는 것이 이 패키지의 존재 이유다.

## 구성

| 경로 | 내용 |
|---|---|
| `robot_control/moveit/` | `MoveGroupClient`(JTC/JGPC), `TrajectoryStreamer`, `ServoClient`, `ee_pose`/`ee_twist`, 그리퍼, `target_joint_cmds_*` |
| `robot_control/camera/` | `camera_node`, `image_capture_node`(JPEG 전용), `image_viewer_node`, OpenCV 캡처 헬퍼. 세션 인지 노드(`rdfp_camera_node` / `rdfp_image_viewer_node`)는 `rdfp` 패키지의 `rdfp/camera/` 에 있다 |
| `robot_control/scene/` | `mock_scene_state_node` — MoveIt planning scene → `/scene/objects` ([계약 문서](../../docs/scene/scene_objects_guide.md)) |
| `robot_control/launch_helpers/` | launch 파일들이 공유하는 argument 선언 + `Node` 팩토리 (**설치되는 모듈**) |
| `launch/` | `panda_mock`, `panda_jgpc_mock`, `panda_gazebo` |
| `config/`, `description/` | `image_pipeline.yaml`, 컨트롤러 YAML, RViz 설정, Panda xacro |

## 실행

```bash
ros2 launch robot_control panda_mock.launch.py        # Panda + MoveIt2 (mock)
ros2 launch robot_control panda_jgpc_mock.launch.py   # JointGroupPositionController 판
ros2 run   robot_control camera_node
```

## 계층 규칙

**이 패키지는 `rdfp`(수집 계층)를 import 하지 않는다.** `robot_control/tests/
test_layer_boundary.py` 가 모든 소스를 AST 파싱해 위반을 실패시킨다. 상위 계층의
기능이 필요하면 import 대신 entry point seam 을 쓴다
(`robot_twin/backend_registry.py` 참고).

문서: [../../docs/moveit/](../../docs/moveit/) · [../../docs/camera/](../../docs/camera/)
