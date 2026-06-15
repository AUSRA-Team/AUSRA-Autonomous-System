"""
Topic Watchdog — monitors the health of a ROS 2 topic by tracking
the time since the last received message.

Usage:
    wd = TopicWatchdog(node, '/ausra_1/odom', Odometry, timeout_sec=0.5)
    if wd.healthy:
        ...
"""

import time


class TopicWatchdog:
    """Subscribes to a ROS 2 topic and reports whether messages are arriving
    within a configurable timeout.

    Starts with ``healthy = False`` (no messages received yet) — this is
    intentional so the supervisor begins in DEGRADED state until all topics
    come online.  The node should NOT block startup waiting for topics.
    """

    def __init__(self, node, topic: str, msg_type, timeout_sec: float):
        """
        Args:
            node:        The parent ROS 2 Node (used for subscription creation and logging).
            topic:       Fully-qualified topic name (e.g. '/ausra_1/odom').
            msg_type:    ROS 2 message class (e.g. nav_msgs.msg.Odometry).
            timeout_sec: Maximum allowed gap between messages before declaring unhealthy.
        """
        self._node = node
        self._topic = topic
        self._timeout = timeout_sec
        self._last_recv: float = 0.0  # epoch 0 → healthy=False until first message

        self._sub = node.create_subscription(
            msg_type,
            topic,
            self._on_msg,
            qos_profile=10,  # reliable, keep-last 10
        )

    def _on_msg(self, _msg) -> None:
        """Record the monotonic timestamp of the latest received message."""
        self._last_recv = time.monotonic()

    @property
    def healthy(self) -> bool:
        """Returns ``True`` if a message was received within the timeout window."""
        if self._last_recv == 0.0:
            return False  # never received a message
        return (time.monotonic() - self._last_recv) < self._timeout

    @property
    def topic(self) -> str:
        """The topic this watchdog is monitoring."""
        return self._topic

    @property
    def elapsed(self) -> float:
        """Seconds since last message, or ``inf`` if no message ever received."""
        if self._last_recv == 0.0:
            return float('inf')
        return time.monotonic() - self._last_recv
