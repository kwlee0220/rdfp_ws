#!/usr/bin/env python3
"""MoveIt Servo utilities for starting, stopping, and monitoring servo functionality.

This module provides utilities for managing MoveIt Servo node operations,
including automatic startup, status monitoring, and service interactions.
"""

import time
from typing import Optional, Tuple
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from std_msgs.msg import Int8
from std_srvs.srv import Trigger


class ServoStatus(IntEnum):
    """MoveIt Servo status codes."""
    UNKNOWN = -1
    NO_WARNING = 0
    DECELERATE_FOR_APPROACHING_SINGULARITY = 1
    HALT_FOR_SINGULARITY = 2
    DECELERATE_FOR_LEAVING_SINGULARITY = 3
    DECELERATE_FOR_JOINT_LIMIT = 4
    HALT_FOR_JOINT_LIMIT = 5
    DECELERATE_FOR_COLLISION = 6
    HALT_FOR_COLLISION = 7


class ServoClient:
    """Utility class for managing MoveIt Servo operations."""

    def __init__(self, node: Node, servo_node_name: str = "/servo_node"):
        """Initialize ServoClient.

        Args:
            node: ROS2 node instance to use for services and subscriptions
            servo_node_name: Name of the servo node (default: "/servo_node")
        """
        self.node = node
        self.servo_node_name = servo_node_name
        self.logger = node.get_logger()

        # Initialize servo status
        self.current_status = ServoStatus.UNKNOWN
        self._status_received = False

        # Create service clients
        self._create_service_clients()

        # Create status subscriber
        self._create_status_subscriber()


    @classmethod
    def create(cls, node: Node, servo_node_name: str = "/servo_node") -> 'ServoClient':
        """Create a ServoClient instance.

        Args:
            node: ROS2 node instance
            servo_node_name: Name of the servo node

        Returns:
            ServoClient instance
        """
        return cls(node, servo_node_name)


    def get_status_string(self, status: Optional[ServoStatus] = None) -> str:
        """Get human-readable servo status string.

        Args:
            status: Status to convert. If None, uses current status.

        Returns:
            Human-readable status string
        """
        if status is None:
            status = self.current_status

        status_map = {
            ServoStatus.UNKNOWN: "UNKNOWN",
            ServoStatus.NO_WARNING: "OK",
            ServoStatus.DECELERATE_FOR_APPROACHING_SINGULARITY: "DECELERATE_FOR_APPROACHING_SINGULARITY",
            ServoStatus.HALT_FOR_SINGULARITY: "HALT_FOR_SINGULARITY",
            ServoStatus.DECELERATE_FOR_LEAVING_SINGULARITY: "DECELERATE_FOR_LEAVING_SINGULARITY",
            ServoStatus.DECELERATE_FOR_JOINT_LIMIT: "DECELERATE_FOR_JOINT_LIMIT",
            ServoStatus.HALT_FOR_JOINT_LIMIT: "HALT_FOR_JOINT_LIMIT",
            ServoStatus.DECELERATE_FOR_COLLISION: "DECELERATE_FOR_COLLISION",
            ServoStatus.HALT_FOR_COLLISION: "HALT_FOR_COLLISION"
        }
        return status_map.get(status, f"UNKNOWN_STATUS_{status}")


    def wait_for_services_ready(self, timeout_sec: float = 10.0) -> bool:
        """Wait for servo services to become ready.

        Args:
            timeout_sec: Maximum time to wait for services

        Returns:
            True if services are ready, False if timeout
        """
        self.logger.info(f"[servo_node] Waiting for servo services (timeout: {timeout_sec}s)")

        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.start_client.service_is_ready():
                self.logger.info("[servo_node] Servo services are ready")
                return True

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.logger.warning(f"[servo_node] ⏰ Timeout waiting for servo services after {timeout_sec}s")
        return False


    def call_servo_service(self, client: Client, service_name: str, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        """Call a servo service and wait for response.

        Args:
            client: Service client to use
            service_name: Name of the service (for logging)
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, message)
        """
        if not client.service_is_ready():
            return False, f"{service_name} service not ready"

        try:
            self.logger.info(f"[servo_node] Calling {service_name}...")

            request = Trigger.Request()
            future = client.call_async(request)

            # Wait for response
            start_time = time.time()
            while not future.done() and (time.time() - start_time < timeout_sec):
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if not future.done():
                return False, f"{service_name} call timed out after {timeout_sec}s"

            response = future.result()
            if response.success:
                self.logger.info(f"[servo_node] {service_name} succeeded: {response.message}")
                return True, response.message
            else:
                self.logger.warning(f"[servo_node] ❌ {service_name} failed: {response.message}")
                return False, response.message

        except Exception as e:
            error_msg = f"{service_name} call failed with exception: {e}"
            self.logger.error(f"[servo_node] {error_msg}")
            return False, error_msg


    def start(self, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        """Start the servo node.

        Args:
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, message)
        """
        return self.call_servo_service(self.start_client, "start_servo", timeout_sec)


    def stop(self, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        """Stop the servo node.

        Args:
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, message)
        """
        return self.call_servo_service(self.stop_client, "stop_servo", timeout_sec)


    def pause(self, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        """Pause the servo node.

        Args:
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, message)
        """
        return self.call_servo_service(self.pause_client, "pause_servo", timeout_sec)


    def unpause(self, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        """Unpause the servo node.

        Args:
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, message)
        """
        return self.call_servo_service(self.unpause_client, "unpause_servo", timeout_sec)


    def reset_status(self, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        """Reset servo status.

        Args:
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, message)
        """
        return self.call_servo_service(self.reset_status_client, "reset_servo_status", timeout_sec)


    def check_status(self, wait_for_status: bool = True, timeout_sec: float = 3.0) -> Tuple[ServoStatus, str]:
        """Check current servo status.

        Args:
            wait_for_status: Whether to wait for status message if not received yet
            timeout_sec: Maximum time to wait for status message

        Returns:
            Tuple of (status, status_string)
        """
        if wait_for_status and not self._status_received:
            self.logger.info("[servo_node] Waiting for servo status...")

            start_time = time.time()
            while not self._status_received and (time.time() - start_time < timeout_sec):
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if not self._status_received:
                self.logger.warning(f"[servo_node] No servo status received after {timeout_sec}s")

        status_str = self.get_status_string(self.current_status)
        return self.current_status, status_str


    def is_healthy(self) -> bool:
        """Check if servo is in a healthy state.

        Returns:
            True if servo status is OK, False otherwise
        """
        return self.current_status == ServoStatus.NO_WARNING


    def is_started(self) -> bool:
        """Check if servo appears to be started.

        Returns:
            True if at least one valid status message has been received.
        """
        return self.current_status != ServoStatus.UNKNOWN


    def auto_start(self) -> bool:
        """Automatically start servo with a single fire-and-forget request.

        Returns:
            True if start request was sent or servo is already healthy, False otherwise
        """
        self.logger.info("[servo_node] Auto-starting servo")

        # First wait for services to be ready
        if not self.wait_for_services_ready(timeout_sec=3.0):
            self.logger.error("[servo_node] Failed to start servo: services not ready")
            return False

        # Check current status first
        status, status_str = self.check_status(wait_for_status=True, timeout_sec=2.0)
        self.logger.info(f"[servo_node] Current status: {status_str}")

        # If already healthy, no need to start
        if self.is_healthy():
            self.logger.info("[servo_node] Servo already running and healthy!")
            return True

        # start_servo 요청을 보내고 응답/상태 확인 없이 즉시 다음 단계로 진행한다.
        if not self.start_client.service_is_ready():
            self.logger.warning("[servo_node] Start failed: start_servo service not ready")
            return False

        self.logger.info("[servo_node] Calling start_servo (fire-and-forget)...")
        self.start_client.call_async(Trigger.Request())
        self.logger.info("[servo_node] start_servo request sent; skipping success verification")
        return True

    def _create_service_clients(self):
        """Create servo service clients."""
        self.start_client = self.node.create_client(
            Trigger, f"{self.servo_node_name}/start_servo"
        )
        self.stop_client = self.node.create_client(
            Trigger, f"{self.servo_node_name}/stop_servo"
        )
        self.pause_client = self.node.create_client(
            Trigger, f"{self.servo_node_name}/pause_servo"
        )
        self.unpause_client = self.node.create_client(
            Trigger, f"{self.servo_node_name}/unpause_servo"
        )
        self.reset_status_client = self.node.create_client(
            Trigger, f"{self.servo_node_name}/reset_servo_status"
        )

    def _create_status_subscriber(self):
        """Create servo status subscriber."""
        # MoveIt Servo status publisher와의 QoS 호환성을 위해 BEST_EFFORT를 사용한다.
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_sub = self.node.create_subscription(
            Int8, f"{self.servo_node_name}/status", self._status_callback, status_qos
        )

    def _status_callback(self, msg: Int8):
        """Handle servo status updates."""
        self.current_status = ServoStatus(msg.data)
        self._status_received = True


# Standalone functions for convenience (when you already have a node)
def auto_start_servo_simple(node: Node, servo_node_name: str = "/servo_node") -> bool:
    """Simple auto-start function for convenience.

    Args:
        node: ROS2 node instance
        servo_node_name: Name of the servo node

    Returns:
        True if servo started successfully, False otherwise
    """
    servo_client = ServoClient.create(node, servo_node_name)
    return servo_client.auto_start()
