#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Woojin Wie

"""
SyncTable-based Dynamixel Model File Generator.

This script generates Dynamixel model files for hand finger joints and pressure sensors
based on the SyncTable concept. The hand models are virtual Dynamixels that use SyncTable
to read data from lower slave Dynamixels and store it in their control table.

Key concepts:
- Finger actuators: read size 6, write size 4
- Pressure sensors: read size 9, write size 0
- SyncTable groups: e.g., SyncTable 1 has finger1,2,3,4 and pressure sensor1
- Joint control table addresses start from SyncTable read/write areas in serial
"""

import os
from typing import Dict, List, Tuple


class SyncTableModelGenerator:
    """Generator for SyncTable-based Dynamixel model files."""

    def __init__(self, output_dir: str = '.',
                 hx_model_file: str = '../param/dxl_model/hx5_d20_rl.model') -> None:
        """Initialize the generator with output directory and hx model file."""
        self.output_dir = output_dir
        self.hx_model_file = hx_model_file
        self.synctable_base_addresses = {}
        self.parse_hx_model_file()

        # Default configuration for hand models
        self.finger_read_size = 6
        self.finger_write_size = 4
        self.pressure_read_size = 9
        self.pressure_write_size = 0

        # Unit info constants
        self.finger_unit_info = {
            'Present Current': {
                'value': '1.0',
                'unit': 'raw',
                'sign_type': 'signed',
                'offset': '0.0'
            },
            'Present Velocity': {
                'value': '0.0239691227',
                'unit': 'rad/s',
                'sign_type': 'signed',
                'offset': '0.0'
            },
            'Present Position': {
                'value': '0.0015339807878856412',
                'unit': 'rad',
                'sign_type': 'signed',
                'offset': '-3.14159265359'
            },
            'Goal Current': {
                'value': '1.0',
                'unit': 'raw',
                'sign_type': 'signed',
                'offset': '0.0'
            },
            'Goal Position': {
                'value': '0.0015339807878856412',
                'unit': 'rad',
                'sign_type': 'signed',
                'offset': '-3.14159265359'
            }
        }

        self.pressure_unit_info = {
            f'Present Pressure {i}': {
                'value': '1.0',
                'unit': 'raw',
                'sign_type': 'unsigned',
                'offset': '0.0'
            }
            for i in range(1, 10)  # 9 pressure sensors
        }

    def parse_hx_model_file(self) -> None:
        """Parse the hx model file to extract SyncTable base addresses."""
        try:
            with open(self.hx_model_file, 'r') as f:
                lines = f.readlines()

            # Find SyncTable read and write data base addresses
            for line in lines:
                if 'SyncTable' in line and 'Read Data 1' in line:
                    parts = line.strip().split('\t')
                    if len(parts) >= 1:
                        address = int(parts[0])
                        # Extract SyncTable number
                        synctable_num = int(line.split('SyncTable ')[1].split(' ')[0])
                        if synctable_num not in self.synctable_base_addresses:
                            self.synctable_base_addresses[synctable_num] = {}
                        # Only set if not already set (to get the first occurrence)
                        if 'read_base' not in self.synctable_base_addresses[synctable_num]:
                            self.synctable_base_addresses[synctable_num]['read_base'] = address

                elif 'SyncTable' in line and 'Write Data 1' in line:
                    parts = line.strip().split('\t')
                    if len(parts) >= 1:
                        address = int(parts[0])
                        # Extract SyncTable number
                        synctable_num = int(line.split('SyncTable ')[1].split(' ')[0])
                        if synctable_num not in self.synctable_base_addresses:
                            self.synctable_base_addresses[synctable_num] = {}
                        # Only set if not already set (to get the first occurrence)
                        if 'write_base' not in self.synctable_base_addresses[synctable_num]:
                            self.synctable_base_addresses[synctable_num]['write_base'] = address

            print(f'Parsed {len(self.synctable_base_addresses)} SyncTables from '
                  f'{self.hx_model_file}')
            for synctable_num, addresses in self.synctable_base_addresses.items():
                print(f'  SyncTable {synctable_num}: '
                      f'read_base={addresses.get("read_base", "N/A")}, '
                      f'write_base={addresses.get("write_base", "N/A")}')

        except FileNotFoundError:
            print(f'Warning: Could not find {self.hx_model_file}, using default addresses')
            # Set default addresses if file not found
            for i in range(1, 6):
                self.synctable_base_addresses[i] = {
                    'read_base': 1069 + (i-1) * 72,
                    'write_base': 1141 + (i-1) * 72
                }
        except Exception as e:
            print(f'Error parsing {self.hx_model_file}: {e}')
            # Set default addresses on error
            for i in range(1, 6):
                self.synctable_base_addresses[i] = {
                    'read_base': 1069 + (i-1) * 72,
                    'write_base': 1141 + (i-1) * 72
                }

    def calculate_synctable_addresses(
        self, synctable_num: int,
        finger_joints: List[int],
        pressure_sensors: List[int]
    ) -> Tuple[List[int], List[int]]:
        """
        Calculate addresses for SyncTable read and write data areas.

        Parameters
        ----------
        synctable_num : int
            SyncTable number (1, 2, 3, etc.).
        finger_joints : list[int]
            Finger joint numbers included in this SyncTable.
        pressure_sensors : list[int]
            Pressure sensor numbers included in this SyncTable.

        Returns
        -------
        tuple[list[int], list[int]]
            Tuple of (read_addresses, write_addresses)

        """
        # Get base addresses from parsed hx model file
        if synctable_num not in self.synctable_base_addresses:
            raise ValueError(f'SyncTable {synctable_num} not found in {self.hx_model_file}')

        base_read_addr = self.synctable_base_addresses[synctable_num]['read_base']
        base_write_addr = self.synctable_base_addresses[synctable_num]['write_base']

        read_addresses = []
        write_addresses = []

        # Calculate addresses for finger joints
        current_read_addr = base_read_addr
        current_write_addr = base_write_addr

        for joint_num in finger_joints:
            # Each finger joint takes 6 bytes for read, 4 bytes for write
            # For read addresses: 3 values of size 2 each = 6 bytes total
            joint_read_addrs = [current_read_addr, current_read_addr + 2, current_read_addr + 4]
            # For write addresses: 2 values of size 2 each = 4 bytes total
            joint_write_addrs = [current_write_addr, current_write_addr + 2]

            read_addresses.extend(joint_read_addrs)
            write_addresses.extend(joint_write_addrs)

            current_read_addr += self.finger_read_size
            current_write_addr += self.finger_write_size

        # Calculate addresses for pressure sensors
        for sensor_num in pressure_sensors:
            # Each pressure sensor takes 9 bytes for read, 0 bytes for write
            # For pressure sensors: 9 values of size 1 each = 9 bytes total
            sensor_read_addrs = list(
                range(current_read_addr, current_read_addr + self.pressure_read_size))

            read_addresses.extend(sensor_read_addrs)
            # No write addresses for pressure sensors

            current_read_addr += self.pressure_read_size

        return read_addresses, write_addresses

    def generate_finger_joint_model(self, joint_number: int,
                                    read_addresses: List[int],
                                    write_addresses: List[int]) -> str:
        """
        Generate a hand finger joint model file.

        Parameters
        ----------
        joint_number : int
            Joint number (1, 2, 3, etc.).
        read_addresses : list[int]
            Read addresses allocated to this joint.
        write_addresses : list[int]
            Write addresses allocated to this joint.

        Returns
        -------
        str
            The model file content as a string

        """
        # Generate unit info section
        unit_info_lines = ['[unit info]', 'Data Name	value	unit	Sign Type	Offset']
        for data_name, info in self.finger_unit_info.items():
            unit_info_lines.append(f'{data_name}	{info["value"]}	{info["unit"]}	'
                                   f'{info["sign_type"]}	{info["offset"]}')

        # Generate control table section
        control_table_lines = ['', '[control table]', 'Address	Size	Data Name']

        # Map read addresses to data names (present values only)
        read_data_mapping = [
            ('Present Current', 2),
            ('Present Velocity', 2),
            ('Present Position', 2)
        ]

        for i, (data_name, size) in enumerate(read_data_mapping):
            if i < len(read_addresses):
                control_table_lines.append(f'{read_addresses[i]}	{size}	{data_name}')

        # Map write addresses to data names (goal commands only)
        write_data_mapping = [
            ('Goal Current', 2),
            ('Goal Position', 2)
        ]

        for i, (data_name, size) in enumerate(write_data_mapping):
            if i < len(write_addresses):
                control_table_lines.append(f'{write_addresses[i]}	{size}	{data_name}')

        content = '\n'.join(unit_info_lines + control_table_lines) + '\n'
        return content

    def generate_pressure_sensor_model(self, sensor_number: int,
                                       read_addresses: List[int]) -> str:
        """
        Generate a hand pressure sensor model file.

        Parameters
        ----------
        sensor_number : int
            Sensor number (5, 10, 15, 20, 25).
        read_addresses : list[int]
            Read addresses allocated to this sensor.

        Returns
        -------
        str
            The model file content as a string

        """
        # Generate unit info section
        unit_info_lines = ['[unit info]', 'Data Name	value	unit	Sign Type	Offset']
        for data_name, info in self.pressure_unit_info.items():
            unit_info_lines.append(f'{data_name}	{info["value"]}	{info["unit"]}	'
                                   f'{info["sign_type"]}	{info["offset"]}')

        # Generate control table section
        control_table_lines = ['', '[control table]', 'Address	Size	Data Name']

        for i in range(1, 10):  # 9 pressure sensors
            if i-1 < len(read_addresses):
                control_table_lines.append(f'{read_addresses[i-1]}	1	Present Pressure {i}')

        content = '\n'.join(unit_info_lines + control_table_lines) + '\n'
        return content

    def generate_synctable_configuration(self, synctable_groups: Dict[int, Dict]) -> None:
        """
        Generate models based on SyncTable group configuration.

        Parameters
        ----------
        synctable_groups : dict[int, dict]
            Mapping of SyncTable numbers to their configurations. Example:
            {
                1: {'finger_joints': [1, 2, 3, 4], 'pressure_sensors': [1]},
                2: {'finger_joints': [5, 6, 7, 8], 'pressure_sensors': [2]},
            }

        """
        for synctable_num, config in synctable_groups.items():
            finger_joints = config.get('finger_joints', [])
            pressure_sensors = config.get('pressure_sensors', [])

            print(f'Processing SyncTable {synctable_num}: joints {finger_joints}, '
                  f'sensors {pressure_sensors}')
            print(f'  Will generate files: '
                  f'hx5_d20_r_synctable_{synctable_num}_device_1.model to '
                  f'hx5_d20_r_synctable_{synctable_num}_device_'
                  f'{len(finger_joints) + len(pressure_sensors)}.model')

            # Calculate addresses for this SyncTable
            read_addresses, write_addresses = self.calculate_synctable_addresses(
                synctable_num, finger_joints, pressure_sensors
            )

            # Generate finger joint models
            current_read_idx = 0
            current_write_idx = 0
            device_num = 1  # Start from device 1

            for joint_num in finger_joints:
                # Each finger joint uses 3 read addresses and 2 write addresses
                joint_read_addrs = read_addresses[current_read_idx:current_read_idx + 3]
                joint_write_addrs = write_addresses[current_write_idx:current_write_idx + 2]

                content = self.generate_finger_joint_model(
                    joint_num, joint_read_addrs, joint_write_addrs)
                filename = f'hx5_d20_r_synctable_{synctable_num}_device_{device_num}.model'
                self.save_model_file(filename, content)

                current_read_idx += 3  # 3 read addresses per joint
                current_write_idx += 2  # 2 write addresses per joint
                device_num += 1

            # Generate pressure sensor models
            for sensor_num in pressure_sensors:
                sensor_read_addrs = read_addresses[
                    current_read_idx:current_read_idx + self.pressure_read_size]

                content = self.generate_pressure_sensor_model(sensor_num, sensor_read_addrs)
                filename = f'hx5_d20_r_synctable_{synctable_num}_device_{device_num}.model'
                self.save_model_file(filename, content)

                current_read_idx += self.pressure_read_size
                device_num += 1

    def generate_default_hand_models(self) -> None:
        """Generate default hand models with standard SyncTable configuration."""
        # Default configuration: 5 SyncTables with 4 finger joints + 1 pressure sensor each
        synctable_groups = {
            1: {'finger_joints': [1, 2, 3, 4], 'pressure_sensors': [1]},
            2: {'finger_joints': [5, 6, 7, 8], 'pressure_sensors': [2]},
            3: {'finger_joints': [9, 10, 11, 12], 'pressure_sensors': [3]},
            4: {'finger_joints': [13, 14, 15, 16], 'pressure_sensors': [4]},
            5: {'finger_joints': [17, 18, 19, 20], 'pressure_sensors': [5]},
        }

        self.generate_synctable_configuration(synctable_groups)

    def generate_custom_hand_models(self, custom_config: Dict) -> None:
        """
        Generate custom hand models based on provided configuration.

        Parameters
        ----------
        custom_config : dict
            Custom configuration dictionary. Example:
            {
                'synctable_groups': {
                    1: {'finger_joints': [1, 2, 3, 4], 'pressure_sensors': [5]},
                    2: {'finger_joints': [5, 6, 7, 8], 'pressure_sensors': [10]},
                },
                'finger_read_size': 6,
                'finger_write_size': 4,
                'pressure_read_size': 9,
                'pressure_write_size': 0
            }

        """
        # Update configuration if provided
        if 'finger_read_size' in custom_config:
            self.finger_read_size = custom_config['finger_read_size']
        if 'finger_write_size' in custom_config:
            self.finger_write_size = custom_config['finger_write_size']
        if 'pressure_read_size' in custom_config:
            self.pressure_read_size = custom_config['pressure_read_size']
        if 'pressure_write_size' in custom_config:
            self.pressure_write_size = custom_config['pressure_write_size']

        synctable_groups = custom_config.get('synctable_groups', {})
        self.generate_synctable_configuration(synctable_groups)

    def save_model_file(self, filename: str, content: str) -> None:
        """Save model content to a file."""
        filepath = os.path.join(self.output_dir, filename)
        with open(filepath, 'w') as f:
            f.write(content)
        print(f'Generated: {filepath}')

    def generate_simple_hand_models(
        self, num_synctables: int = 5, joints_per_synctable: int = 4
    ) -> None:
        """
        Generate hand models with a simple, sequential configuration.

        Parameters
        ----------
        num_synctables : int
            Number of SyncTables to generate.
        joints_per_synctable : int
            Number of finger joints per SyncTable.

        """
        synctable_groups = {}

        for i in range(1, num_synctables + 1):
            # Calculate joint numbers for this SyncTable
            start_joint = (i - 1) * joints_per_synctable + 1
            end_joint = i * joints_per_synctable
            finger_joints = list(range(start_joint, end_joint + 1))

            # Each SyncTable gets one pressure sensor with the same number
            pressure_sensors = [i]

            synctable_groups[i] = {
                'finger_joints': finger_joints,
                'pressure_sensors': pressure_sensors
            }

        print(f'Generated configuration for {num_synctables} SyncTables:')
        for synctable_num, config in synctable_groups.items():
            print(f'  SyncTable {synctable_num}: joints {config["finger_joints"]}, '
                  f'sensor {config["pressure_sensors"]}')

        self.generate_synctable_configuration(synctable_groups)

    def analyze_existing_models(self, model_dir: str = '../param/dxl_model') -> Dict:
        """
        Analyze existing hand models to extract SyncTable configuration.

        Parameters
        ----------
        model_dir : str
            Directory containing existing model files.

        Returns
        -------
        dict
            Dictionary containing extracted configuration

        """
        # This would analyze existing files to extract the SyncTable mapping
        # For now, return a placeholder
        return {
            'message': 'Analysis functionality would extract '
                       'SyncTable configuration from existing models'
        }


def main():
    """Demonstrate usage of the SyncTable-based model generator."""
    # You can specify a different hx model file if needed
    generator = SyncTableModelGenerator(
        output_dir='../param/dxl_model',
        hx_model_file='../param/dxl_model/hx5_d20_rl.model'
    )

    print('SyncTable-based Dynamixel Model Generator')
    print('=' * 50)

    # Example 1: Generate simple hand models (recommended)
    print('\n1. Generating simple hand models...')
    generator.generate_simple_hand_models(num_synctables=5, joints_per_synctable=4)

    print('\nModel generation complete!')


if __name__ == '__main__':
    main()
