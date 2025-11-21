#!/usr/bin/env python3
"""Policy loader module for loading and running RL policies."""

import importlib.util
from typing import Optional, Union

import numpy as np
import rospy


class PolicyLoader:
    """Loads and runs RL policies in TorchScript or ONNX format."""

    def __init__(self, policy_path: str, policy_type: str = "auto"):
        """
        Initialize policy loader.

        Args:
            policy_path: Path to policy file (.pt, .onnx)
            policy_type: Policy type ("auto", "torch", "onnx")
        """
        self.policy_path = policy_path
        self.policy_type = policy_type.lower()
        self.policy_impl = None

        if policy_path:
            self._load_policy()

    def _load_policy(self):
        """Load policy from file."""
        if not self.policy_path:
            rospy.loginfo("Policy path not provided.")
            return

        if self.policy_type == "auto":
            if self.policy_path.endswith(".onnx"):
                self.policy_type = "onnx"
            else:
                self.policy_type = "torch"

        if self.policy_type == "torch":
            self._load_torch_policy()
        elif self.policy_type == "onnx":
            self._load_onnx_policy()
        else:
            rospy.logerr("Unsupported policy_type: %s", self.policy_type)

    def _load_torch_policy(self):
        """Load TorchScript policy."""
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

    def _load_onnx_policy(self):
        """Load ONNX policy."""
        ort_spec = importlib.util.find_spec("onnxruntime")
        if ort_spec is None:
            rospy.logerr("onnxruntime is not available but policy_type is set to onnx.")
            return

        import onnxruntime as ort

        try:
            self.policy_impl = ort.InferenceSession(
                self.policy_path, providers=["CPUExecutionProvider"]
            )
            rospy.loginfo("Loaded ONNX policy from %s", self.policy_path)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("Failed to load ONNX policy: %s", exc)
            self.policy_impl = None

    def is_loaded(self) -> bool:
        """Check if policy is loaded."""
        return self.policy_impl is not None

    def infer(self, observation: np.ndarray) -> Optional[np.ndarray]:
        """
        Run policy inference.

        Args:
            observation: Input observation array

        Returns:
            Action array or None if inference failed
        """
        if self.policy_impl is None:
            return None

        if self.policy_type == "torch":
            return self._infer_torch(observation)
        elif self.policy_type == "onnx":
            return self._infer_onnx(observation)
        return None

    def _infer_torch(self, observation: np.ndarray) -> Optional[np.ndarray]:
        """Run TorchScript inference."""
        import torch

        try:
            with torch.no_grad():
                tensor = torch.from_numpy(observation).unsqueeze(0)
                output = self.policy_impl(tensor).cpu().numpy().squeeze()
            return np.array(output, dtype=np.float32)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("Torch inference failed: %s", exc)
            return None

    def _infer_onnx(self, observation: np.ndarray) -> Optional[np.ndarray]:
        """Run ONNX inference."""
        try:
            input_name = self.policy_impl.get_inputs()[0].name
            outputs = self.policy_impl.run(None, {input_name: observation[np.newaxis, :]})
            output = outputs[0].squeeze()
            return np.array(output, dtype=np.float32)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("ONNX inference failed: %s", exc)
            return None

