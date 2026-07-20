#!/usr/bin/env python3
"""Shared interactive calibration routine for Feetech-servo robot arms.

Each robot package (so101_ros2, lekiwi_ros2, lerre_ros2) ships its own thin
`*_calibration_node` executable so it has its own entry point and package
share directory, but all of them run the same connect / homing /
range-of-motion / save workflow defined here. Subclasses (or direct
instantiations) only need to supply the owning package name and, if
different from the standard SO101 arm, a motor map.
"""

import json
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import yaml

from .feetech_bus import FeetechMotorsBus, OperatingMode
from .motors_bus import Motor, MotorCalibration, MotorNormMode

# Default SO101 arm motor map: (joint_name, motor_id, model, norm_mode).
# This is the same physical arm/motor layout whether it's standalone
# (so101_ros2), mounted on LeKiwi, or mounted on LeRRe.
SO101_ARM_MOTORS = (
    ('shoulder_pan', 1, 'sts3215', MotorNormMode.DEGREES),
    ('shoulder_lift', 2, 'sts3215', MotorNormMode.DEGREES),
    ('elbow_flex', 3, 'sts3215', MotorNormMode.DEGREES),
    ('wrist_flex', 4, 'sts3215', MotorNormMode.DEGREES),
    ('wrist_roll', 5, 'sts3215', MotorNormMode.DEGREES),
    ('gripper', 6, 'sts3215', MotorNormMode.RANGE_0_100),
)


class So101CalibrationNodeBase(Node):
    """Interactive calibration routine shared by every robot package's calibration node."""

    motor_spec = SO101_ARM_MOTORS

    def __init__(self, node_name, package_name, default_port='/dev/ttyACM0', default_arm_id='arm'):
        super().__init__(node_name)
        self.package_name = package_name

        self.declare_parameter('port', default_port)
        self.declare_parameter('arm_id', default_arm_id)
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('output_file', '')

        self.port = str(self.get_parameter('port').value or default_port)
        self.arm_id = str(self.get_parameter('arm_id').value or default_arm_id)
        use_degrees_val = self.get_parameter('use_degrees').value
        self.use_degrees = bool(use_degrees_val) if use_degrees_val is not None else True
        output_file = str(self.get_parameter('output_file').value or '')

        # Generate default output filename if not provided
        if not output_file:
            # Use ROS 2 package share directory (works in both source and install space)
            try:
                package_share_dir = Path(get_package_share_directory(self.package_name))
                config_dir = package_share_dir / 'config'
                config_dir.mkdir(parents=True, exist_ok=True)
                output_file = str(config_dir / f'{self.arm_id}_calibration.yaml')
            except Exception as e:
                # Fallback: find package source directory by looking for package.xml or setup.py
                self.get_logger().warn(f'Could not find package share directory: {e}, using source directory')

        self.output_file = Path(output_file)

        motors = {}
        for joint_name, motor_id, model, norm_mode in self.motor_spec:
            resolved_mode = norm_mode
            if norm_mode == MotorNormMode.DEGREES and not self.use_degrees:
                resolved_mode = MotorNormMode.RANGE_M100_100
            motors[joint_name] = Motor(motor_id, model, resolved_mode)

        self.bus = FeetechMotorsBus(port=self.port, motors=motors, calibration=None)

    def load_calibration(self):
        """Load existing calibration file if it exists."""
        if not self.output_file.exists():
            return None

        try:
            with open(self.output_file, 'r') as f:
                if self.output_file.suffix.lower() in {'.yaml', '.yml'}:
                    data = yaml.safe_load(f)
                else:
                    data = json.load(f)

            # Handle ROS 2 parameter file format
            if isinstance(data, dict):
                # Check if it's in ROS 2 parameter format (arm_name -> ros__parameters -> calibration)
                for arm_name, node_data in data.items():
                    if isinstance(node_data, dict) and 'ros__parameters' in node_data:
                        if 'calibration' in node_data['ros__parameters']:
                            data = node_data['ros__parameters']['calibration']
                            break

            calibration = {}
            for joint_name, calib_dict in data.items():
                calibration[joint_name] = MotorCalibration(
                    id=calib_dict['id'],
                    drive_mode=calib_dict.get('drive_mode', 0),
                    homing_offset=calib_dict['homing_offset'],
                    range_min=calib_dict['range_min'],
                    range_max=calib_dict['range_max'],
                )
            return calibration
        except Exception as e:
            self.get_logger().warn(f'Could not load calibration file: {e}')
            return None

    def save_calibration(self, calibration_data):
        """Save calibration data to YAML or JSON file."""
        try:
            self.output_file.parent.mkdir(parents=True, exist_ok=True)

            # Build calibration data structure
            calib_dict = {}
            for joint_name, calib in calibration_data.items():
                calib_dict[joint_name] = {
                    'id': calib.id,
                    'drive_mode': calib.drive_mode,
                    'homing_offset': calib.homing_offset,
                    'range_min': calib.range_min,
                    'range_max': calib.range_max,
                }

            with open(self.output_file, 'w') as f:
                if self.output_file.suffix.lower() in {'.yaml', '.yml'}:
                    # Determine node name from arm_id for ROS 2 parameter format
                    if 'leader' in self.arm_id.lower():
                        arm_name = 'so101_leader'
                    elif 'follower' in self.arm_id.lower():
                        arm_name = 'so101_follower'
                    else:
                        # Fallback: use arm_id as node name
                        arm_name = self.arm_id.replace('_arm', '')

                    # Save in ROS 2 parameter file format
                    output_data = {
                        arm_name: {
                            'ros__parameters': {
                                'calibration': calib_dict,
                            }
                        }
                    }
                    yaml.safe_dump(output_data, f, sort_keys=True, default_flow_style=False)
                else:
                    # For JSON, save in simple format (backward compatible)
                    json.dump(calib_dict, f, indent=4)

            return True
        except Exception as e:
            self.get_logger().error(f'Error saving calibration: {e}')
            return False

    def run_calibration(self):
        """Main calibration procedure."""
        print('\n' + '=' * 70)
        print(f'  SO-101 ARM CALIBRATION: {self.arm_id.upper()}')
        print('=' * 70 + '\n')

        # Check if stdin is available (required for interactive calibration)
        if not sys.stdin.isatty():
            print('\n' + '=' * 70)
            print('  ERROR: Interactive input required')
            print('=' * 70)
            print('\n  Please run the calibration node directly:')
            print(f'    ros2 run {self.package_name} so101_calibration_node --ros-args \\')
            print(f'      -p port:={self.port} -p arm_id:={self.arm_id}\n')
            return False

        # Initialize connection
        print('  Connecting to arm...', end=' ', flush=True)
        try:
            self.bus.connect(handshake=True)
            print('✓ Connected')
        except Exception as e:
            print(f'\n  ERROR: Failed to connect: {e}')
            return False

        # Check for existing calibration
        existing_calib = self.load_calibration()
        if existing_calib:
            print(f'\n  Existing calibration found for {self.arm_id}')
            user_input = input("  Use existing? (ENTER=yes, 'c'=recalibrate): ")
            if user_input.strip().lower() != 'c':
                print('  Writing existing calibration to motors...', end=' ', flush=True)
                self.bus.write_calibration(existing_calib)
                print('✓ Done')
                self.bus.disconnect(disable_torque=False)
                return True

        # Run new calibration
        print('\n  Starting new calibration procedure...\n')

        # Step 1: Disable torque and configure motors
        print('  Step 1: Configuring motors...', end=' ', flush=True)
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write('Operating_Mode', motor, OperatingMode.POSITION.value)
        self.bus.disable_torque()  # Ensure torque stays disabled
        print('✓ Torque disabled - arm can be moved manually')

        # Step 2: Set homing offsets using middle position (mirroring lerobot logic)
        print('\n' + '-' * 70)
        print('  Step 2: Setting homing offsets from middle position')
        print('-' * 70)
        print(f'\n  Move {self.arm_id} to the middle of its range of motion, then press ENTER...')
        input()

        # Set half-turn homings (mirrors lerobot: makes current position = 2047 = half-turn)
        print('  Setting homing offsets...', end=' ', flush=True)
        try:
            homing_offsets = self.bus.set_half_turn_homings()
            print('✓ Done')
        except Exception as e:
            print(f'\n  ERROR: Failed to set homing offsets: {e}')
            import traceback
            traceback.print_exc()
            return False

        # Step 3: Record ranges of motion (after homing offsets are set, matching lerobot)
        print('\n' + '-' * 70)
        print('  Step 3: Recording joint ranges of motion')
        print('-' * 70)
        print('\n  Move all joints sequentially through their entire ranges of motion.')
        print('  Recording positions. Press ENTER to stop...\n')

        range_mins, range_maxes = self.bus.record_ranges_of_motion()

        # Build calibration data (mirroring lerobot structure)
        print('  Building calibration data...', end=' ', flush=True)
        calibration_data = {}
        for motor, m in self.bus.motors.items():
            calibration_data[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        print('✓ Done')

        print('  Writing calibration to motors...', end=' ', flush=True)
        self.bus.write_calibration(calibration_data)
        print('✓ Done')

        print('  Saving calibration to file...', end=' ', flush=True)
        if not self.save_calibration(calibration_data):
            print('\n  ERROR: Failed to save calibration file')
            return False
        print('✓ Done')

        # Success message
        print('\n' + '=' * 70)
        print('  ✓ CALIBRATION COMPLETE!')
        print('=' * 70)
        print(f'\n  Calibration saved to: {self.output_file}')
        print('  You can now use this calibration file with your leader/follower nodes.\n')

        self.bus.disconnect(disable_torque=False)
        return True

    def on_shutdown(self):
        """Cleanup on shutdown."""
        if self.bus.is_connected:
            try:
                self.bus.disconnect(disable_torque=False)
            except Exception:
                pass
