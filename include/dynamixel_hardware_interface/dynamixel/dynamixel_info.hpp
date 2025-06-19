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
  double torque_constant;
  double min_radian;
  double max_radian;
  int32_t value_of_zero_radian_position;
  int32_t value_of_max_radian_position;
  int32_t value_of_min_radian_position;
  uint16_t model_num;
  double velocity_unit;

  std::vector<ControlItem> item;
} DxlInfo;

class DynamixelInfo
{
private:
  using DxlModelList = std::map<uint16_t, std::string>;
  DxlModelList dxl_model_list_;

  std::string dxl_model_file_dir;

public:
  // Id, Control table
  std::map<uint8_t, DxlInfo> dxl_info_;

  DynamixelInfo() {}
  ~DynamixelInfo() {}

  void SetDxlModelFolderPath(const char * path);
  void InitDxlModelInfo();

  void ReadDxlModelFile(uint8_t id, uint16_t model_num);
  bool GetDxlControlItem(uint8_t id, std::string item_name, uint16_t & addr, uint8_t & size);
  bool CheckDxlControlItem(uint8_t id, std::string item_name);
  // bool GetDxlTypeInfo(
  //   uint8_t id,
  //   int32_t & value_of_zero_radian_position,
  //   int32_t & value_of_max_radian_position,
  //   int32_t & value_of_min_radian_position,
  //   double & min_radian,
  //   double & max_radian,
  //   double & torque_constant,
  //   double & velocity_unit);
  int32_t ConvertRadianToValue(uint8_t id, double radian);
  double ConvertValueToRadian(uint8_t id, int32_t value);
  inline int16_t ConvertEffortToCurrent(uint8_t id, double effort)
  {return static_cast<int16_t>(effort / dxl_info_[id].torque_constant);}
  inline double ConvertCurrentToEffort(uint8_t id, int16_t current)
  {return static_cast<double>(current * dxl_info_[id].torque_constant);}
  inline double ConvertValueRPMToVelocityRPS(uint8_t id, int32_t value_rpm)
  {return static_cast<double>(value_rpm * dxl_info_[id].velocity_unit / 60.0 * 2.0 * M_PI);}
  inline int32_t ConvertVelocityRPSToValueRPM(uint8_t id, double vel_rps)
  {return static_cast<int32_t>(vel_rps / dxl_info_[id].velocity_unit * 60.0 / 2.0 / M_PI);}
};

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_INFO_HPP_
