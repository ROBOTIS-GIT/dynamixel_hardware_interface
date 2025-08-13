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

from collections import defaultdict
import hashlib
import os

MODEL_DIR = os.path.join(os.path.dirname(__file__), '..', 'param', 'dxl_model')

# Get all files in the model directory
files = [f for f in os.listdir(MODEL_DIR) if os.path.isfile(os.path.join(MODEL_DIR, f))]

# Dictionary to map file content hash to list of files
hash_to_files = defaultdict(list)


def extract_control_table(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    in_control_table = False
    control_table_lines = []
    for line in lines:
        if line.strip() == '[control table]':
            in_control_table = True
            continue
        if in_control_table:
            if line.startswith('[') and not line.strip().startswith('[control table]'):
                break
            if line.strip() == '':
                continue
            control_table_lines.append(line.strip())
    return '\n'.join(control_table_lines)


def control_table_hash(control_table_str):
    return hashlib.sha256(control_table_str.encode('utf-8')).hexdigest()


for filename in files:
    path = os.path.join(MODEL_DIR, filename)
    control_table = extract_control_table(path)
    h = control_table_hash(control_table)
    hash_to_files[h].append(filename)

# Print clusters of files with identical [control table] sections
print('Clusters of files with identical [control table] sections:')
for file_list in hash_to_files.values():
    if len(file_list) > 1:
        print(', '.join(file_list))

# Print files with unique [control table] sections
print('\nFiles with unique [control table] sections:')
for file_list in hash_to_files.values():
    if len(file_list) == 1:
        print(file_list[0])
