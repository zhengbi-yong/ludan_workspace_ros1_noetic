#!/usr/bin/env python3
"""Data recording module for recording motor states to rosbag or CSV."""

import csv
import math
from typing import List, Optional, Sequence

import rospy
import rosbag
from damiao_motor_control_board_serial.msg import MotorStates


class DataRecorder:
    """Records motor states to rosbag or CSV file."""

    def __init__(
        self,
        record_bag_path: Optional[str] = None,
        record_csv_path: Optional[str] = None,
        motor_states_topic: str = "motor_states",
        joint_order: Optional[Sequence[int]] = None,
        observation_fields: Sequence[str] = None,
    ):
        """
        Initialize data recorder.

        Args:
            record_bag_path: Path to output rosbag file (optional)
            record_csv_path: Path to output CSV file (optional)
            motor_states_topic: Topic name for motor states
            joint_order: Ordered list of joint IDs to record
            observation_fields: Fields to record (e.g., ["pos", "vel", "tor"])
        """
        self.record_bag_path = record_bag_path
        self.record_csv_path = record_csv_path
        self.motor_states_topic = motor_states_topic
        self.joint_order = joint_order or []
        self.observation_fields = observation_fields or ["pos", "vel", "tor"]

        self.record_bag = None
        self.record_csv = None
        self.csv_writer = None

        if record_bag_path:
            try:
                self.record_bag = rosbag.Bag(record_bag_path, mode="w")
                rospy.loginfo("Recording to rosbag: %s", record_bag_path)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("Failed to open rosbag for writing: %s", exc)

        if record_csv_path:
            try:
                self.record_csv = open(record_csv_path, "w", newline="")
                self.csv_writer = csv.writer(self.record_csv)
                header = ["stamp"]
                for joint in self.joint_order:
                    for field in self.observation_fields:
                        header.append(f"{joint}_{field}")
                self.csv_writer.writerow(header)
                rospy.loginfo("Recording to CSV: %s", record_csv_path)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("Failed to open CSV for writing: %s", exc)

        rospy.on_shutdown(self.close)

    def record(self, msg: MotorStates):
        """
        Record motor states message.

        Args:
            msg: MotorStates message to record
        """
        if self.record_bag is not None:
            try:
                self.record_bag.write(self.motor_states_topic, msg, t=msg.header.stamp)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("Failed to write to rosbag: %s", exc)

        if self.record_csv is not None and self.joint_order:
            try:
                row = [
                    msg.header.stamp.to_sec()
                    if msg.header.stamp
                    else rospy.Time.now().to_sec()
                ]
                motors = {state.id: state for state in msg.motors}
                for joint in self.joint_order:
                    state = motors.get(joint)
                    if state is None:
                        row.extend([math.nan] * len(self.observation_fields))
                        continue
                    for field in self.observation_fields:
                        row.append(getattr(state, field, math.nan))
                self.csv_writer.writerow(row)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("Failed to write to CSV: %s", exc)

    def close(self):
        """Close recording files."""
        if self.record_bag is not None:
            try:
                self.record_bag.close()
                rospy.loginfo("Closed rosbag recording")
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("Failed to close rosbag: %s", exc)
            self.record_bag = None

        if self.record_csv is not None:
            try:
                self.record_csv.close()
                rospy.loginfo("Closed CSV recording")
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("Failed to close CSV: %s", exc)
            self.record_csv = None

