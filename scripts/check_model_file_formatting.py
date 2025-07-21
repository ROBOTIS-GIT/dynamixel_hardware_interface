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
Script to check formatting of Dynamixel model files.

This script checks for:
1. Trailing spaces on lines
2. Proper EOF (End of File) placement
3. Consistent line endings

Usage:
    python3 check_model_file_formatting.py [--fix] [--verbose]

Options:
    --fix      Automatically fix formatting issues
    --verbose  Show detailed information about each file
"""

import argparse
import os
import sys


def check_file_formatting(file_path, fix=False, verbose=False):
    """
    Check and optionally fix formatting issues in a file.

    Args:
        file_path (str): Path to the file to check
        fix (bool): Whether to automatically fix issues
        verbose (bool): Whether to show detailed output

    Returns
    -------
        dict: Dictionary with issues found and fixed

    """
    issues = {
        'trailing_spaces': [],
        'no_eof_newline': False,
        'empty_lines_at_end': 0,
        'fixed': [],
    }

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except Exception as e:
        print(f'Error reading {file_path}: {e}')
        return issues

    if not lines:
        if verbose:
            print(f'  {file_path}: Empty file')
        return issues

    # Check for trailing spaces
    for i, line in enumerate(lines, 1):
        # Remove newline and check if there are trailing spaces
        line_content = line.rstrip('\n')
        if line_content.rstrip() != line_content:
            issues['trailing_spaces'].append(i)

    # Check EOF
    if lines and not lines[-1].endswith('\n'):
        issues['no_eof_newline'] = True

    # Count empty lines at the end
    empty_lines_at_end = 0
    for line in reversed(lines):
        if line.strip() == '':
            empty_lines_at_end += 1
        else:
            break

    if empty_lines_at_end > 1:
        issues['empty_lines_at_end'] = empty_lines_at_end

    # Fix issues if requested
    if fix and (
        issues['trailing_spaces']
        or issues['no_eof_newline']
        or issues['empty_lines_at_end'] > 1
    ):
        fixed_lines = []

        for i, line in enumerate(lines):
            # Remove trailing spaces but preserve newline
            if i + 1 in issues['trailing_spaces']:
                line_content = line.rstrip('\n')
                fixed_lines.append(line_content.rstrip() + '\n')
                issues['fixed'].append(
                    f'Line {i + 1}: Removed trailing spaces')
            else:
                fixed_lines.append(line)

        # Ensure proper EOF
        if issues['no_eof_newline']:
            if fixed_lines and not fixed_lines[-1].endswith('\n'):
                fixed_lines[-1] = fixed_lines[-1] + '\n'
            issues['fixed'].append('Added EOF newline')

        # Remove excessive empty lines at the end
        if issues['empty_lines_at_end'] > 1:
            # Remove all empty lines at the end, then add one
            while fixed_lines and fixed_lines[-1].strip() == '':
                fixed_lines.pop()
            fixed_lines.append('\n')  # Add single newline at end
            issues['fixed'].append(
                f'Removed {issues["empty_lines_at_end"] - 1} excessive empty lines at end'
            )

        # Write fixed file
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.writelines(fixed_lines)
        except Exception as e:
            print(f'Error writing {file_path}: {e}')
            return issues

    return issues


def main():
    """Run the formatting check or fix process for Dynamixel model files."""
    parser = argparse.ArgumentParser(
        description='Check formatting of Dynamixel model files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 check_model_file_formatting.py
    python3 check_model_file_formatting.py --fix
    python3 check_model_file_formatting.py --verbose
    python3 check_model_file_formatting.py --fix --verbose
        """,
    )

    parser.add_argument(
        '--fix', action='store_true', help='Automatically fix formatting issues'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Show detailed information about each file',
    )

    args = parser.parse_args()

    # Find model files
    model_dir = 'param/dxl_model'
    if not os.path.exists(model_dir):
        print(f"Error: Model directory '{model_dir}' not found.")
        print('Please run this script from the dynamixel_hardware_interface directory.')
        sys.exit(1)

    model_files = []
    for file in os.listdir(model_dir):
        if file.endswith('.model'):
            model_files.append(os.path.join(model_dir, file))

    if not model_files:
        print('No .model files found.')
        sys.exit(0)

    print(f'Checking {len(model_files)} model files...')
    if args.fix:
        print('Auto-fix mode enabled.')
    print()

    total_issues = 0
    files_with_issues = 0

    for file_path in sorted(model_files):
        file_name = os.path.basename(file_path)
        issues = check_file_formatting(file_path, args.fix, args.verbose)

        has_issues = (
            issues['trailing_spaces']
            or issues['no_eof_newline']
            or issues['empty_lines_at_end'] > 1
        )

        if has_issues:
            files_with_issues += 1
            print(f'❌ {file_name}')

            if issues['trailing_spaces']:
                print(
                    f'   Trailing spaces on lines: \
                    {", ".join(map(str, issues["trailing_spaces"]))}'
                )
                total_issues += len(issues['trailing_spaces'])

            if issues['no_eof_newline']:
                print('   Missing EOF newline')
                total_issues += 1

            if issues['empty_lines_at_end'] > 1:
                print(
                    f'   {issues["empty_lines_at_end"]} empty lines at end of file')
                total_issues += 1

            if args.fix and issues['fixed']:
                print('   Fixed:')
                for fix in issues['fixed']:
                    print(f'     - {fix}')

            print()
        elif args.verbose:
            print(f'✅ {file_name} - No issues found')

    # Summary
    print('=' * 50)
    if files_with_issues == 0:
        print('🎉 All model files are properly formatted!')
    else:
        print(
            f'Found {total_issues} formatting issues in {files_with_issues} files.')
        if not args.fix:
            print('Run with --fix to automatically fix these issues.')

    return 0 if files_with_issues == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
