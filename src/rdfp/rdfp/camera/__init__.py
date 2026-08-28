"""세션 상태를 인지하는 카메라 노드 모음.

카메라 하드웨어 추상화(`OpenCvCamera`)와 세션을 모르는 노드
(`camera_node` / `image_viewer_node`)는 제어 계층인 `robot_control.camera`
에 있다. 이 서브패키지에는 `/session` (`rdfp_msgs/SessionCommand`) 상태에
따라 동작이 달라지는 **수집 계층 어댑터**만 둔다.
"""
