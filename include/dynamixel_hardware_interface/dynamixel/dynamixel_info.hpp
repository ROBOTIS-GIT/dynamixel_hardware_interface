// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Hye-Jong KIM, Sungho Woo, Woojin Wie

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_INFO_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_INFO_HPP_

#include <cmath>
#include <cstring>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <regex>

#include <boost/algorithm/string.hpp>

namespace dynamixel_hardware_interface
{

typedef struct
{
  uint16_t address;
  uint8_t size;
  std::string item_name;
} ControlItem;

typedef  struct
{
  double min_radian;
  double max_radian;
  int32_t value_of_zero_radian_position;
  int32_t value_of_max_radian_position;
  int32_t value_of_min_radian_position;
  uint16_t model_num;
  uint8_t firmware_version;

  std::vector<ControlItem> item;
  std::map<std::string, double> unit_map;
  std::map<std::string, bool> sign_type_map;
} DxlInfo;

class DynamixelInfo
{
private:
  using DxlModelList = std::map<uint16_t, std::string>;
  DxlModelList dxl_model_list_;

  std::string dxl_model_file_dir;

  // Firmware version-aware model file selection
  std::string SelectModelFileByFirmwareVersion(
    const std::string & base_model_name,
    uint8_t firmware_version);
  uint8_t ExtractFirmwareVersionFromFilename(const std::string & filename);

public:
  // Id, Control table
  std::map<uint8_t, DxlInfo> dxl_info_;

  DynamixelInfo() {}
  ~DynamixelInfo() {}

  void SetDxlModelFolderPath(const char * path);
  void InitDxlModelInfo();

  void ReadDxlModelFile(uint8_t id, uint16_t model_num);
  void ReadDxlModelFile(uint8_t id, uint16_t model_num, uint8_t firmware_version);
  bool GetDxlControlItem(uint8_t id, std::string item_name, uint16_t & addr, uint8_t & size);
  bool CheckDxlControlItem(uint8_t id, std::string item_name);

  bool GetDxlUnitValue(uint8_t id, std::string data_name, double & unit_value);
  bool GetDxlSignType(uint8_t id, std::string data_name, bool & is_signed);

  // Template-based conversion methods
  template<typename T>
  double ConvertValueToUnit(uint8_t id, std::string data_name, T value);

  template<typename T>
  T ConvertUnitToValue(uint8_t id, std::string data_name, double unit_value);

  // Helper method for internal use
  double GetUnitMultiplier(uint8_t id, std::string data_name);

  int32_t ConvertRadianToValue(uint8_t id, double radian);
  double ConvertValueToRadian(uint8_t id, int32_t value);

  std::string GetModelName(uint16_t model_number) const;
};

// Template implementations
template<typename T>
double DynamixelInfo::ConvertValueToUnit(uint8_t id, std::string data_name, T value)
{
  auto it = dxl_info_[id].unit_map.find(data_name);
  if (it != dxl_info_[id].unit_map.end()) {
    return static_cast<double>(value) * it->second;
  }
  return static_cast<double>(value);
}

template<typename T>
T DynamixelInfo::ConvertUnitToValue(uint8_t id, std::string data_name, double unit_value)
{
  auto it = dxl_info_[id].unit_map.find(data_name);
  if (it != dxl_info_[id].unit_map.end()) {
    return static_cast<T>(unit_value / it->second);
  }
  return static_cast<T>(unit_value);
}

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_INFO_HPP_
