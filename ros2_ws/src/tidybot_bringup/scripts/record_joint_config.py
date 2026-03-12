#!/usr/bin/env python3
"""
Record Taught Joint Configurations for Warm-Start IK

Disables torque so you can freely pose the arm by hand, shows live joint
values in the terminal, and saves the configuration to taught_configs.yaml
for use as a warm-start seed in the motion planner.

Usage:
    # Make sure real.launch.py is running first, then:
    ros2 run tidybot_bringup record_joint_config.py

    # For right arm:
    ros2 run tidybot_bringup record_joint_config.py --ros-args -p arm_name:=right

Commands (interactive):
    s + ENTER  →  snapshot & save as PICK  config
    p + ENTER  →  snapshot & save as PLACE config
    b + ENTER  →  snapshot & save as BOTH  (pick AND place, same pose)
    ENTER      →  refresh display (show current joint values again)
    q + ENTER  →  quit and re-enable torque
"""

import sys
import time
import threading
from pathlib import Path

import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.srv import TorqueEnable

try:
    from ament_index_python.packages import get_package_share_directory
    _HAS_AMENT = True
except ImportError:
    _HAS_AMENT = False

# Joint order expected by the motion planner (waist → wrist_rotate)
JOINT_ORDER = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']


def find_config_path() -> Path:
    """Locate taught_configs.yaml via ament or relative path fallback."""
    if _HAS_AMENT:
        try:
            share = get_package_share_directory('tidybot_bringup')
            return Path(share) / 'config' / 'taught_configs.yaml'
        except Exception:
            pass
    # Fallback: two levels up from scripts/ → tidybot_bringup/config/
    return Path(__file__).resolve().parents[1] / 'config' / 'taught_configs.yaml'


class Recorder(Node):
    def __init__(self):
        super().__init__('joint_recorder')

        self.declare_parameter('arm_name', 'left')
        self.arm_name: str = self.get_parameter('arm_name').value

        if self.arm_name not in ('left', 'right'):
            raise ValueError(f"arm_name must be 'left' or 'right', got '{self.arm_name}'")

        self._ns = f'{self.arm_name}_arm'
        # Full joint names as published by xs_sdk (e.g. left_waist, left_shoulder, ...)
        self._full_names = [f'{self.arm_name}_{j}' for j in JOINT_ORDER]

        self._positions: dict = {}
        self._lock = threading.Lock()

        self.create_subscription(
            JointState,
            f'/{self._ns}/joint_states',
            self._js_cb,
            10,
        )

        self._torque_client = self.create_client(TorqueEnable, f'/{self._ns}/torque_enable')
        if not self._torque_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                f'Service /{self._ns}/torque_enable not found. '
                'Is real.launch.py running?'
            )

    # -------------------------------------------------------------------------

    def _js_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._positions[name] = pos

    def get_positions(self) -> 'np.ndarray | None':
        """Return current 6-DOF positions in JOINT_ORDER, or None if not ready."""
        with self._lock:
            if not all(n in self._positions for n in self._full_names):
                return None
            return np.array([self._positions[n] for n in self._full_names])

    def set_torque(self, enable: bool):
        req = TorqueEnable.Request()
        req.cmd_type = 'group'
        req.name = f'{self.arm_name}_arm'
        req.enable = enable
        future = self._torque_client.call_async(req)
        # The background rclpy.spin() thread handles the response — just wait.
        # Do NOT call rclpy.spin_once() here; that would fight the background thread
        # and corrupt the executor, causing joint state callbacks to stop firing.
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        label = 'ENABLED' if enable else 'DISABLED'
        print(f'  Torque {label} on /{self._ns}')


# ─────────────────────────────────────────────────────────────────────────────

def save_config(config_path: Path, arm_name: str, label: str, pos: np.ndarray):
    """Read → update → write the YAML config file.

    Normalises keys so both 'left' and 'left_arm' in the existing file are
    treated as the same arm, and always written back as plain 'left'/'right'.
    """
    data: dict = {}
    if config_path.exists():
        with open(config_path) as f:
            raw = yaml.safe_load(f) or {}
        # Normalise any 'left_arm'/'right_arm' keys to plain 'left'/'right'
        for k, v in raw.items():
            norm = k.replace('_arm', '')
            data[norm] = v
    else:
        print(f'  NOTE: {config_path} not found — will create it.')

    if arm_name not in data:
        data[arm_name] = {}

    data[arm_name][label] = [round(float(v), 6) for v in pos]

    try:
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with open(config_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        print(f'  Saved  →  {arm_name}.{label}')
        print(f'           [{", ".join(f"{v:+.6f}" for v in pos)}]')
        print(f'  File   →  {config_path}')
    except Exception as e:
        print(f'  ERROR saving to {config_path}: {e}')
        # Try writing next to this script as a fallback
        fallback = Path(__file__).resolve().parents[1] / 'config' / 'taught_configs.yaml'
        try:
            fallback.parent.mkdir(parents=True, exist_ok=True)
            with open(fallback, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            print(f'  Saved to fallback: {fallback}')
        except Exception as e2:
            print(f'  Fallback also failed: {e2}')


def print_positions(pos: 'np.ndarray | None'):
    if pos is None:
        print('  (no joint data yet)')
        return
    print('  Current joints (rad):')
    for name, val in zip(JOINT_ORDER, pos):
        bar = '█' * int(abs(val) / 3.14 * 20)
        sign = '+' if val >= 0 else '-'
        print(f'    {name:14s}  {val:+.4f}  {sign}{bar}')
    print(f'  Array: [{", ".join(f"{v:+.4f}" for v in pos)}]')


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    try:
        node = Recorder()
    except RuntimeError as e:
        print(f'\nERROR: {e}')
        rclpy.shutdown()
        return

    config_path = find_config_path()

    # ROS spin runs in a background thread so the main thread can block on input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print()
    print('=' * 62)
    print(f'  Joint Config Recorder  |  arm: {node.arm_name}')
    print(f'  Joint order : {JOINT_ORDER}')
    print(f'  Config file : {config_path}')
    print('=' * 62)
    print()
    print('  Waiting for joint states from joint_states callback ...')

    deadline = time.time() + 10.0
    while node.get_positions() is None and time.time() < deadline:
        time.sleep(0.1)

    if node.get_positions() is None:
        print('  ERROR: No joint states received after 10 s.')
        print('         Make sure real.launch.py is running.')
        node.destroy_node()
        rclpy.shutdown()
        return

    print('  Joint states received.')
    print()
    print('  Disabling torque — arm will go LIMP now.')
    node.set_torque(False)
    print()
    print('  ┌──────────────────────────────────────────────────────┐')
    print('  │  Move the arm to your desired pre-pick/place pose.   │')
    print('  │                                                        │')
    print('  │  s + ENTER  →  save as PICK  config                   │')
    print('  │  p + ENTER  →  save as PLACE config                   │')
    print('  │  b + ENTER  →  save as BOTH  (pick AND place)         │')
    print('  │  ENTER      →  refresh display                        │')
    print('  │  q + ENTER  →  quit  (re-enables torque)              │')
    print('  └──────────────────────────────────────────────────────┘')
    print()

    try:
        while True:
            # Show current state before each prompt
            print_positions(node.get_positions())
            print()

            try:
                cmd = input('  Command > ').strip().lower()
            except EOFError:
                break

            # Snapshot the positions at the moment the command is confirmed
            pos = node.get_positions()
            print()

            if cmd == 'q':
                break

            if pos is None:
                print('  No joint data — cannot save.')
                continue

            if cmd == 's':
                save_config(config_path, node.arm_name, 'pick', pos)
            elif cmd == 'p':
                save_config(config_path, node.arm_name, 'place', pos)
            elif cmd == 'b':
                save_config(config_path, node.arm_name, 'pick', pos)
                save_config(config_path, node.arm_name, 'place', pos)
            elif cmd == '':
                pass  # just refresh
            else:
                print(f'  Unknown command "{cmd}". Use s / p / b / q / ENTER.')

            print()

    except KeyboardInterrupt:
        print()

    finally:
        print()
        print('  Re-enabling torque ...')
        node.set_torque(True)
        print('  Torque is ON. Arm is rigid again.')
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
