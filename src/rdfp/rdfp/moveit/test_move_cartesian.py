import rclpy
from rclpy.node import Node

from rdfp.moveit import MoveGroupClient, pose


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Node('move_to_cartesian_points')
        with MoveGroupClient(node) as client:
            client.wait_until_ready()

            # Trajectory 1: 사각형 경로
            rectangular_path = [
                pose(0.4,  0.3, 0.4, 3.14, 0.0, 0.0),
                pose(0.4, -0.3, 0.4, 3.14, 0.0, 0.0),
                pose(0.7, -0.3, 0.4, 3.14, 0.0, 0.0),
                pose(0.7,  0.3, 0.4, 3.14, 0.0, 0.0),
                pose(0.4,  0.3, 0.4, 3.14, 0.0, 0.0),
            ]

            # Trajectory 2: 수직 지그재그 경로
            zigzag_path = [
                pose(0.3, 0.0, 0.6, 3.14, 0.0, 0.0),
                pose(0.3, 0.0, 0.3, 3.14, 0.0, 0.0),
                pose(0.6, 0.0, 0.6, 3.14, 0.0, 0.0),
                pose(0.6, 0.0, 0.3, 3.14, 0.0, 0.0),
            ]

            path = [
                pose(0.40, -0.20, 0.45, 0.0, 0.0, 0.0),
                pose(0.45, -0.10, 0.45, 0.0, 0.0, 0.0),
                pose(0.50,  0.00, 0.42, 0.0, 0.0, 0.0),
                pose(0.45,  0.10, 0.40, 0.0, 0.0, 0.0),
                pose(0.40,  0.20, 0.42, 0.0, 0.0, 0.0),
            ]

            node.get_logger().info('Starting rectangular path (50% speed)...')
            client.follow_trajectory(rectangular_path, velocity_scaling=0.5)

            node.get_logger().info('Starting zigzag path (100% speed)...')
            client.follow_trajectory(zigzag_path)

            node.get_logger().info('Starting another path (100% speed)...')
            client.follow_trajectory(path)

    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'Unhandled exception: {exc}')
        else:
            print(f'Unhandled exception before node init: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
