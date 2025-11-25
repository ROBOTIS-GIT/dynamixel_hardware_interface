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
Parse a Dynamixel XML configuration and generate a generic model file.

This script reads a Dynamixel XML configuration file and outputs
a corresponding model file, using a generic approach (no hardcoded categories).
"""

import os
import sys
import xml.etree.ElementTree as ET


def parse_xml_to_model(xml_file_path: str, output_file_path: str) -> None:
    """
    Parse XML file and generate model file using a generic approach.

    Parameters
    ----------
    xml_file_path : str
        Path to the XML file that will be parsed.
    output_file_path : str
        Path where the generated model file will be saved.

    """
    try:
        # Parse XML file
        tree = ET.parse(xml_file_path)
        root = tree.getroot()

        # Extract device information
        device_name = root.get('Name', 'Unknown')
        model_number = root.get('ModelNumber', '0')

        print(f'Processing device: {device_name} (Model: {model_number})')

        # Collect all items
        items = []

        # Process ControlItems section
        control_items = root.find('ControlItems')
        if control_items is not None:
            # Process direct Item elements (not in categories)
            for item in control_items.findall('Item'):
                # Skip hidden items
                if item.get('Hidden', '0') == '1':
                    continue

                address = item.get('Address', '0')
                length = item.get('Length', '1')
                name = item.get('Name', 'Unknown')

                # Convert address to integer for sorting
                try:
                    addr_int = int(address)
                    items.append((addr_int, int(length), name))
                except ValueError:
                    print(f'Warning: Invalid address "{address}" for item "{name}"')
                    continue

            # Process Category elements
            for category in control_items.findall('Category'):
                category_name = category.get('Name', 'Unknown')

                # Process items within this category
                for item in category.findall('Item'):
                    # Skip hidden items
                    if item.get('Hidden', '0') == '1':
                        continue

                    address = item.get('Address', '0')
                    length = item.get('Length', '1')
                    name_template = item.get('Name', 'Unknown')
                    continue_range = item.get('Continue', '')

                    if continue_range:
                        try:
                            start, end = map(int, continue_range.split('~'))

                            # Special handling for Indirect Address and Data
                            # (only include first entry)
                            if category_name in ['Indirect Address', 'Indirect Data']:
                                if start == 1:
                                    try:
                                        addr_int = int(address)
                                        item_name = f'{category_name} {start}'
                                        items.append((addr_int, int(length), item_name))
                                    except ValueError:
                                        print(f'Warning: Invalid address "{address}" '
                                              f'for item "{item_name}"')
                                        continue
                            else:
                                # Generic handling for all other categories
                                for i in range(start, end + 1):
                                    try:
                                        # Calculate the correct address for this entry
                                        base_addr = int(address)
                                        # For Continue entries, increment address by (i-1) * length
                                        if i > 1:
                                            addr_int = base_addr + (i - 1) * int(length)
                                        else:
                                            addr_int = base_addr

                                        # Generate item name using
                                        # category name + template replacement
                                        if name_template == '{0}':
                                            # If template is just {0}, use category name + number
                                            item_name = f'{category_name} {i}'
                                        else:
                                            # Otherwise use template replacement
                                            item_name = name_template.replace('{0}', str(i))
                                        items.append((addr_int, int(length), item_name))
                                    except ValueError:
                                        print(f'Warning: Invalid address "{address}" '
                                              f'for item "{item_name}"')
                                        continue
                        except (ValueError, IndexError):
                            print(
                                f'Warning: Invalid continue range "{continue_range}" '
                                f'for category "{category_name}"'
                            )
                            continue
                    else:
                        # Single item
                        try:
                            addr_int = int(address)
                            items.append((addr_int, int(length), name_template))
                        except ValueError:
                            print(
                                f'Warning: Invalid address "{address}" for item "{name_template}"'
                            )
                            continue

        # Add special entries that are in the reference but not in XML
        special_entries = [
            (122, 2, 'Indirect Address Read'),
            (634, 1, 'Indirect Data Read'),
            (452, 2, 'Indirect Address Write'),
            (799, 1, 'Indirect Data Write')
        ]

        for addr, size, name in special_entries:
            items.append((addr, size, name))

        # Remove duplicates while preserving order (based on address and name)
        unique_items = []
        seen = set()
        for item in items:
            # Use a tuple of (address, name) to identify unique items
            item_identifier = (item[0], item[2])
            if item_identifier not in seen:
                unique_items.append(item)
                seen.add(item_identifier)

        # Sort items by address
        unique_items.sort(key=lambda x: x[0])

        # Write to output file
        with open(output_file_path, 'w') as f:
            f.write('[control table]\n')
            f.write('Address\tSize\tData Name\n')
            for address, size, name in unique_items:
                f.write(f'{address}\t{size}\t{name}\n')

        print(f'Successfully generated model file: {output_file_path}')
        print(f'Total items processed: {len(unique_items)}')

    except ET.ParseError as e:
        print(f'Error parsing XML file: {e}')
        sys.exit(1)
    except FileNotFoundError:
        print(f'Error: XML file not found: {xml_file_path}')
        sys.exit(1)
    except Exception as e:
        print(f'Error: {e}')
        sys.exit(1)


def main() -> None:
    """Handle command-line arguments and run the XML-to-model parsing."""
    # Default paths
    default_xml_file = 'HX5-D20-RL.xml'
    default_model_files = [
        '../param/dxl_model/hx5_d20_rl.model',
        '../param/dxl_model/hx5_d20_rr.model'
    ]

    # Use command line arguments if provided, otherwise use defaults
    if len(sys.argv) == 3:
        xml_file = sys.argv[1]
        model_file = sys.argv[2]
    elif len(sys.argv) == 1:
        xml_file = default_xml_file
        print(f'Using default input: {xml_file}')
        print(f'Using default outputs: {default_model_files}')

        # Process each default model file
        for model_file in default_model_files:
            # Create output directory if it doesn't exist
            output_dir = os.path.dirname(model_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)

            # Parse XML and generate model file
            parse_xml_to_model(xml_file, model_file)
        return
    else:
        print('Usage: python xml_to_model_parser_generic.py [input_xml_file] [output_model_file]')
        print('If no arguments provided, uses defaults:')
        print(f'  Input: {default_xml_file}')
        print(f'  Outputs: {default_model_files}')
        sys.exit(1)

    # Check if input file exists
    if not os.path.exists(xml_file):
        print(f'Error: Input file "{xml_file}" does not exist')
        sys.exit(1)

    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(model_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Parse XML and generate model file
    parse_xml_to_model(xml_file, model_file)


if __name__ == '__main__':
    main()
