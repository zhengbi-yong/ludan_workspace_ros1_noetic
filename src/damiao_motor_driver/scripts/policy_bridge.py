#!/usr/bin/env python3

import csv
import importlib.util
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rospy
import rosbag
from damiao_motor_control_board_serial.msg import MotorStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def _as_array(value, length: int) -> np.ndarray:
    if isinstance(value, (list, tuple)):
        arr = np.array(value, dtype=np.float32)
    else:
        arr = np.full(length, float(value), dtype=np.float32)
    if arr.size != length:
        raise ValueError(f"Expected parameter with length {length}, got {arr.size}")
    return arr


def _clip_if_needed(array: np.ndarray, limit: Optional[float]) -> np.ndarray:
    if limit is None or limit <= 0.0:
        return array
    return np.clip(array, -limit, limit)


class PolicyBridge:
    def __init__(self):
        self.motor_states_topic = rospy.get_param("~motor_states_topic", "motor_states")
        self.command_topic = rospy.get_param("~command_topic", "joint_group_effort_controller/command")
        self.command_type = rospy.get_param("~command_type", "effort").lower()
        self.joint_order: Sequence[int] = rospy.get_param("~joint_order", [])
        self.observation_fields: Sequence[str] = rospy.get_param("~observation_fields", ["pos", "vel", "tor"])
        self.observation_scale_param = rospy.get_param("~observation_scale", 1.0)
        self.observation_clip = rospy.get_param("~observation_clip", 0.0)
        self.action_scale_param = rospy.get_param("~action_scale", 1.0)
        self.action_clip = rospy.get_param("~action_clip", 0.0)
        self.inference_rate = float(rospy.get_param("~inference_rate", 100.0))
        self.action_filter_alpha = float(rospy.get_param("~action_filter_alpha", 0.25))
        self.action_damping = float(rospy.get_param("~action_damping", 0.0))
        self.policy_path = rospy.get_param("~policy_path", "")
        self.policy_type = rospy.get_param("~policy_type", "auto").lower()
        self.action_topic = rospy.get_param("~action_topic", "")
        self.playback_source = rospy.get_param("~playback_source", "")
        self.playback_topic = rospy.get_param("~playback_topic", self.command_topic)
        self.record_bag_path = rospy.get_param("~record_bag_path", "")
        self.record_csv_path = rospy.get_param("~record_csv_path", "")

        self.latest_states: Optional[MotorStates] = None
        self.latest_action: Optional[np.ndarray] = None
        self.filtered_action: Optional[np.ndarray] = None
        self.policy_impl = None
        self.playback_actions: List[Tuple[float, List[float]]] = []
        self.playback_index = 0
        self.playback_start_time = None

        self.command_pub = rospy.Publisher(self.command_topic, Float64MultiArray, queue_size=1)
        self.twist_pub = None
        if self.command_type in {"twist", "cmd_vel"} or self.command_topic.endswith("cmd_vel"):
            self.twist_pub = rospy.Publisher(self.command_topic, Twist, queue_size=1)

        self.record_bag = None
        if self.record_bag_path:
            self.record_bag = rosbag.Bag(self.record_bag_path, mode="w")
        self.record_csv = None
        if self.record_csv_path:
            self.record_csv = open(self.record_csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.record_csv)
            header = ["stamp"]
            for joint in self.joint_order or []:
                for field in self.observation_fields:
                    header.append(f"{joint}_{field}")
            self.csv_writer.writerow(header)

        rospy.Subscriber(self.motor_states_topic, MotorStates, self.motor_states_cb, queue_size=1)
        if self.action_topic:
            rospy.Subscriber(self.action_topic, Float64MultiArray, self.external_action_cb, queue_size=1)

        rospy.on_shutdown(self._cleanup_recorders)
        self._load_policy_if_needed()
        self._load_playback()

    def _cleanup_recorders(self):
        if self.record_bag is not None:
            self.record_bag.close()
        if self.record_csv is not None:
            self.record_csv.close()

    def _load_policy_if_needed(self):
        if not self.policy_path:
            rospy.loginfo("Policy path not provided, expecting actions from topic or playback.")
            return

        if self.policy_type == "auto":
            if self.policy_path.endswith(".onnx"):
                self.policy_type = "onnx"
            else:
                self.policy_type = "torch"

        if self.policy_type == "torch":
            torch_spec = importlib.util.find_spec("torch")
            if torch_spec is None:
                rospy.logerr("PyTorch is not available but policy_type is set to torch.")
                return
            import torch

            try:
                self.policy_impl = torch.jit.load(self.policy_path)
                self.policy_impl.eval()
                rospy.loginfo("Loaded TorchScript policy from %s", self.policy_path)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("Failed to load Torch policy: %s", exc)
                self.policy_impl = None
        elif self.policy_type == "onnx":
            ort_spec = importlib.util.find_spec("onnxruntime")
            if ort_spec is None:
                rospy.logerr("onnxruntime is not available but policy_type is set to onnx.")
                return
            import onnxruntime as ort

            try:
                self.policy_impl = ort.InferenceSession(self.policy_path, providers=["CPUExecutionProvider"])
                rospy.loginfo("Loaded ONNX policy from %s", self.policy_path)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("Failed to load ONNX policy: %s", exc)
                self.policy_impl = None
        else:
            rospy.logerr("Unsupported policy_type: %s", self.policy_type)

    def _load_playback(self):
        if not self.playback_source:
            return
        if self.playback_source.endswith(".bag"):
            self._load_bag_playback()
        elif self.playback_source.endswith(".csv"):
            self._load_csv_playback()
        else:
            rospy.logwarn("Unsupported playback source: %s", self.playback_source)

    def _load_bag_playback(self):
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
        self.latest_states = msg
        self._record_feedback(msg)

    def external_action_cb(self, msg: Float64MultiArray):
        self.latest_action = np.array(msg.data, dtype=np.float32)

    def _record_feedback(self, msg: MotorStates):
        if self.record_bag is not None:
            self.record_bag.write(self.motor_states_topic, msg, t=msg.header.stamp)
        if self.record_csv is not None and self.joint_order:
            row = [msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()]
            motors = self._index_motor_states(msg)
            for joint in self.joint_order:
                state = motors.get(joint)
                if state is None:
                    row.extend([math.nan] * len(self.observation_fields))
                    continue
                for field in self.observation_fields:
                    row.append(getattr(state, field, math.nan))
            self.csv_writer.writerow(row)

    def _index_motor_states(self, msg: MotorStates):
        return {state.id: state for state in msg.motors}

    def _build_observation(self) -> Optional[np.ndarray]:
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
        if self.playback_actions:
            return self._next_playback_action()
        if self.policy_impl is None:
            return self.latest_action
        if self.policy_type == "torch":
            import torch

            with torch.no_grad():
                tensor = torch.from_numpy(obs).unsqueeze(0)
                output = self.policy_impl(tensor).cpu().numpy().squeeze()
        elif self.policy_type == "onnx":
            input_name = self.policy_impl.get_inputs()[0].name
            outputs = self.policy_impl.run(None, {input_name: obs[np.newaxis, :]})
            output = outputs[0].squeeze()
        else:
            return None
        action = np.array(output, dtype=np.float32)
        scale = _as_array(self.action_scale_param, len(action))
        action = action * scale
        action = _clip_if_needed(action, float(self.action_clip))
        return action

    def _next_playback_action(self) -> Optional[np.ndarray]:
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

    def _filter_action(self, action: np.ndarray) -> np.ndarray:
        if self.filtered_action is None:
            self.filtered_action = action.copy()
            return self.filtered_action
        if self.action_damping > 0.0 and self.latest_action is not None:
            delta = action - self.filtered_action
            action = self.filtered_action + (1.0 - self.action_damping) * delta
        alpha = max(0.0, min(1.0, self.action_filter_alpha))
        self.filtered_action = self.filtered_action + alpha * (action - self.filtered_action)
        return self.filtered_action

    def _publish_action(self, action: np.ndarray):
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
            action = self._filter_action(action)
            self._publish_action(action)
            rate.sleep()


def main():
    rospy.init_node("policy_bridge")
    node = PolicyBridge()
    node.spin()


if __name__ == "__main__":
    main()
