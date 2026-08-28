from __future__ import annotations

import rclpy
from rclpy.node import Node
from robot_control.moveit import create_move_group_client


def main():
    rclpy.init()
    node = Node("sample_move_group")

    try:
        # 컨트롤러(JTC/JGPC)를 자동 판별해 알맞은 구현을 생성한다.
        with create_move_group_client(node) as client:
            # 클라이언트가 준비될 때까지 대기한다.
            client.wait_until_ready()

            # 기본 그룹의 Named targets 를 가져온다. 인자를 생략하면 생성자의
            # moveit_group_name(기본 "panda_arm") 한 그룹만 조회한다.
            targets = client.get_named_targets()
            print("Named targets:", targets)

            # 특정 그룹의 Named targets 를 가져온다.
            hand_targets = client.get_named_targets(group="hand")
            print("Hand targets:", hand_targets)

            # 모든 그룹의 Named targets 를 {group: [name, ...]} 로 한 번에 가져온다.
            all_targets = client.get_all_named_targets()
            print("All targets:", all_targets)

            # SRDF 에 정의된 planning group 이름 전체. 위와 달리 named target 이
            # 하나도 없는 그룹(예: "panda_arm_hand")도 포함된다.
            groups = client.get_planning_groups()
            print("Planning groups:", groups)

            # "ready"라는 Named target으로 이동한다.
            client.move_to_named_target("ready")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
