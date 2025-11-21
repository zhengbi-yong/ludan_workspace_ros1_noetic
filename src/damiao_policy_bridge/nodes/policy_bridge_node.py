#!/usr/bin/env python3
"""Policy bridge node for RL policy inference, data recording, and playback."""

import csv
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rospy
import rosbag
from damiao_motor_control_board_serial.msg import MotorStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from damiao_policy_bridge.lib.action_filter import ActionFilter
from damiao_policy_bridge.lib.data_recorder import DataRecorder
from damiao_policy_bridge.lib.policy_loader import PolicyLoader


def _as_array(value, length: int) -> np.ndarray:
    """Convert value to numpy array of specified length."""
    if isinstance(value, (list, tuple)):
        arr = np.array(value, dtype=np.float32)
    else:
        arr = np.full(length, float(value), dtype=np.float32)
    if arr.size != length:
        raise ValueError(f"Expected parameter with length {length}, got {arr.size}")
    return arr


def _clip_if_needed(array: np.ndarray, limit: Optional[float]) -> np.ndarray:
    """Clip array if limit is specified."""
    if limit is None or limit <= 0.0:
        return array
    return np.clip(array, -limit, limit)


class PolicyBridge:
    """Main policy bridge class."""

    def __init__(self):
        """Initialize policy bridge."""
        # Load parameters
        self.motor_states_topic = rospy.get_param("~motor_states_topic", "motor_states")
        self.command_topic = rospy.get_param(
            "~command_topic", "joint_group_effort_controller/command"
        )
        self.command_type = rospy.get_param("~command_type", "effort").lower()
        self.joint_order: Sequence[int] = rospy.get_param("~joint_order", [])
        self.observation_fields: Sequence[str] = rospy.get_param(
            "~observation_fields", ["pos", "vel", "tor"]
        )
        self.observation_scale_param = rospy.get_param("~observation_scale", 1.0)
        self.observation_clip = rospy.get_param("~observation_clip", 0.0)
        self.action_scale_param = rospy.get_param("~action_scale", 1.0)
        self.action_clip = rospy.get_param("~action_clip", 0.0)
        self.inference_rate = float(rospy.get_param("~inference_rate", 100.0))
        action_filter_alpha = float(rospy.get_param("~action_filter_alpha", 0.25))
        action_damping = float(rospy.get_param("~action_damping", 0.0))
        policy_path = rospy.get_param("~policy_path", "")
        policy_type = rospy.get_param("~policy_type", "auto").lower()
        self.action_topic = rospy.get_param("~action_topic", "")
        self.playback_source = rospy.get_param("~playback_source", "")
        self.playback_topic = rospy.get_param("~playback_topic", self.command_topic)
        record_bag_path = rospy.get_param("~record_bag_path", "")
        record_csv_path = rospy.get_param("~record_csv_path", "")

        # Initialize components
        self.policy_loader = PolicyLoader(policy_path, policy_type)
        self.data_recorder = DataRecorder(
            record_bag_path=record_bag_path,
            record_csv_path=record_csv_path,
            motor_states_topic=self.motor_states_topic,
            joint_order=self.joint_order,
            observation_fields=self.observation_fields,
        )
        self.action_filter = ActionFilter(
            filter_alpha=action_filter_alpha, damping=action_damping
        )

        # State
        self.latest_states: Optional[MotorStates] = None
        self.latest_action: Optional[np.ndarray] = None
        self.playback_actions: List[Tuple[float, List[float]]] = []
        self.playback_index = 0
        self.playback_start_time = None

        # Publishers
        self.command_pub = rospy.Publisher(
            self.command_topic, Float64MultiArray, queue_size=1
        )
        self.twist_pub = None
        if self.command_type in {"twist", "cmd_vel"} or self.command_topic.endswith(
            "cmd_vel"
        ):
            self.twist_pub = rospy.Publisher(self.command_topic, Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber(
            self.motor_states_topic, MotorStates, self.motor_states_cb, queue_size=1
        )
        if self.action_topic:
            rospy.Subscriber(
                self.action_topic,
                Float64MultiArray,
                self.external_action_cb,
                queue_size=1,
            )

        # Load playback data
        self._load_playback()

    def _load_playback(self):
        """Load playback data from bag or CSV."""
        if not self.playback_source:
            return
        if self.playback_source.endswith(".bag"):
            self._load_bag_playback()
        elif self.playback_source.endswith(".csv"):
            self._load_csv_playback()
        else:
            rospy.logwarn("Unsupported playback source: %s", self.playback_source)

    def _load_bag_playback(self):
        """Load playback actions from rosbag."""
        try:
            with rosbag.Bag(self.playback_source, "r") as bag:
                for topic, msg, t in bag.read_messages(topics=[self.playback_topic]):
                    if isinstance(msg, Float64MultiArray):
                        self.playback_actions.append((t.to_sec(), list(msg.data)))
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("Failed to load playback bag: %s", exc)
        if self.playback_actions:
            rospy.loginfo("Loaded %d actions from rosbag", len(self.playback_actions))

    def _load_csv_playback(self):
        """Load playback actions from CSV."""
        try:
            with open(self.playback_source, "r") as handle:
                reader = csv.reader(handle)
                for row in reader:
                    if not row:
                        continue
                    try:
                        timestamp = float(row[0]) if len(row) > 1 else 0.0
                        values = [float(v) for v in (row[1:] if len(row) > 1 else row)]
                    except ValueError:
                        continue
                    self.playback_actions.append((timestamp, values))
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("Failed to load playback csv: %s", exc)
        if self.playback_actions:
            rospy.loginfo("Loaded %d actions from CSV", len(self.playback_actions))

    def motor_states_cb(self, msg: MotorStates):
        """Motor states callback."""
        self.latest_states = msg
        self.data_recorder.record(msg)

    def external_action_cb(self, msg: Float64MultiArray):
        """External action callback."""
        self.latest_action = np.array(msg.data, dtype=np.float32)

    def _index_motor_states(self, msg: MotorStates):
        """Index motor states by ID."""
        return {state.id: state for state in msg.motors}

    def _build_observation(self) -> Optional[np.ndarray]:
        """Build observation array from motor states."""
        if self.latest_states is None:
            return None
        motors = self._index_motor_states(self.latest_states)
        order = self.joint_order or list(motors.keys())
        obs_values: List[float] = []
        for joint in order:
            state = motors.get(joint)
            if state is None:
                rospy.logwarn_throttle(5.0, "Missing state for joint %s", joint)
                obs_values.extend([0.0] * len(self.observation_fields))
                continue
            for field in self.observation_fields:
                obs_values.append(float(getattr(state, field, 0.0)))
        obs = np.array(obs_values, dtype=np.float32)
        scale = _as_array(self.observation_scale_param, len(obs))
        obs = obs * scale
        obs = _clip_if_needed(obs, float(self.observation_clip))
        return obs

    def _inference(self, obs: np.ndarray) -> Optional[np.ndarray]:
        """Run policy inference or get playback action."""
        if self.playback_actions:
            return self._next_playback_action()
        if self.policy_loader.is_loaded():
            action = self.policy_loader.infer(obs)
            if action is not None:
                scale = _as_array(self.action_scale_param, len(action))
                action = action * scale
                action = _clip_if_needed(action, float(self.action_clip))
            return action
        return self.latest_action

    def _next_playback_action(self) -> Optional[np.ndarray]:
        """Get next action from playback."""
        if self.playback_index >= len(self.playback_actions):
            return None
        if self.playback_start_time is None:
            self.playback_start_time = rospy.Time.now().to_sec()
        timestamp, values = self.playback_actions[self.playback_index]
        now = rospy.Time.now().to_sec()
        if timestamp > 0.0 and now - self.playback_start_time < timestamp:
            return self.latest_action
        self.playback_index += 1
        self.latest_action = np.array(values, dtype=np.float32)
        return self.latest_action

    def _publish_action(self, action: np.ndarray):
        """Publish action to command topic."""
        if self.twist_pub is not None:
            twist = Twist()
            values = action.tolist()
            twist.linear.x = values[0] if len(values) > 0 else 0.0
            twist.linear.y = values[1] if len(values) > 1 else 0.0
            twist.linear.z = values[2] if len(values) > 2 else 0.0
            twist.angular.x = values[3] if len(values) > 3 else 0.0
            twist.angular.y = values[4] if len(values) > 4 else 0.0
            twist.angular.z = values[5] if len(values) > 5 else 0.0
            self.twist_pub.publish(twist)
            return

        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.command_pub.publish(msg)

    def spin(self):
        """Main loop."""
        rate = rospy.Rate(self.inference_rate)
        while not rospy.is_shutdown():
            obs = self._build_observation()
            if obs is None:
                rate.sleep()
                continue
            action = self._inference(obs)
            if action is None:
                rate.sleep()
                continue
            self.latest_action = action
            filtered_action = self.action_filter.filter(action, self.latest_action)
            self._publish_action(filtered_action)
            rate.sleep()


def main():
    """Main entry point."""
    rospy.init_node("policy_bridge")
    node = PolicyBridge()
    node.spin()


if __name__ == "__main__":
    main()

