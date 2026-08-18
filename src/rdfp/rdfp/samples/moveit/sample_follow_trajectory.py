from __future__ import annotations

import rclpy
from rclpy.node import Node
from rdfp.moveit import create_move_group_client, pose


def main():
    rclpy.init()
    node = Node("sample_move_group")

    try:
        # 컨트롤러(JTC/JGPC)를 자동 판별해 알맞은 구현을 생성한다.
        with create_move_group_client(node) as client:
            # 클라이언트가 준비될 때까지 대기한다.
            client.wait_until_ready()

            # 이동 범위를 종전의 2/3 로 축소한다. 각 축의 중심을 유지한 채
            # 폭만 줄이므로 y 축 대칭성(로봇 중심선 기준)과 작업 영역 위치는
            # 그대로다. y: ±0.3 → ±0.2, x: 0.4~0.7 → 0.45~0.65.
            waypoints = [
                pose(0.45, 0.2, 0.4, 3.14, 0.0, 0.0),
                pose(0.45, -0.2, 0.4, 3.14, 0.0, 0.0),
                pose(0.65, -0.2, 0.4, 3.14, 0.0, 0.0),
            ]

            client.follow_trajectory(waypoints, velocity_scaling=0.5)
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
