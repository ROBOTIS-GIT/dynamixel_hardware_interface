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

#include "dynamixel_hardware_interface/dynamixel/dynamixel.hpp"

#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <chrono>

namespace dynamixel_hardware_interface
{

Dynamixel::Dynamixel(const char * path)
: port_handler_(nullptr),
  packet_handler_(nullptr),
  group_sync_read_(nullptr),
  group_bulk_read_(nullptr),
  group_fast_sync_read_(nullptr),
  group_fast_bulk_read_(nullptr),
  group_sync_write_(nullptr),
  group_bulk_write_(nullptr)
{
  dxl_info_.SetDxlModelFolderPath(path);
  dxl_info_.InitDxlModelInfo();

  write_item_buf_.clear();
  read_item_buf_.clear();
}

Dynamixel::~Dynamixel()
{
  fprintf(stderr, "Dynamixel destructor start\n");
  if (group_sync_read_) {
    delete group_sync_read_;
    group_sync_read_ = nullptr;
  }
  if (group_fast_sync_read_) {
    delete group_fast_sync_read_;
    group_fast_sync_read_ = nullptr;
  }
  if (group_bulk_read_) {
    delete group_bulk_read_;
    group_bulk_read_ = nullptr;
  }
  if (group_fast_bulk_read_) {
    delete group_fast_bulk_read_;
    group_fast_bulk_read_ = nullptr;
  }
  if (group_sync_write_) {
    delete group_sync_write_;
    group_sync_write_ = nullptr;
  }
  if (group_bulk_write_) {
    delete group_bulk_write_;
    group_bulk_write_ = nullptr;
  }
  if (port_handler_) {
    port_handler_->closePort();
    delete port_handler_;
    port_handler_ = nullptr;
  }
  packet_handler_ = nullptr;
  fprintf(stderr, "Dynamixel destructor end\n");
}

DxlError Dynamixel::ReadDxlModelFile(uint8_t id, uint16_t model_num)
{
  try {
    dxl_info_.ReadDxlModelFile(id, model_num);
    return DxlError::OK;
  } catch (const std::exception & e) {
    fprintf(stderr, "[ReadDxlModelFile][ID:%03d] Error reading model file: %s\n", id, e.what());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
}

DxlError Dynamixel::InitTorqueStates(std::vector<uint8_t> id_arr, bool disable_torque)
{
  for (auto it_id : id_arr) {
    try {
      if (dxl_info_.CheckDxlControlItem(it_id, "Torque Enable")) {
        uint32_t torque_state = 0;
        DxlError result = ReadItem(it_id, "Torque Enable", torque_state);
        if (result != DxlError::OK) {
          fprintf(stderr, "[InitTorqueStates][ID:%03d] Error reading torque state\n", it_id);
          return result;
        }

        torque_state_[it_id] = torque_state;

        if (torque_state_[it_id] == TORQUE_ON) {
          if (disable_torque) {
            fprintf(
              stderr, "[InitTorqueStates][ID:%03d] Torque is enabled, disabling torque\n",
              it_id);
            result = WriteItem(it_id, "Torque Enable", TORQUE_OFF);
            if (result != DxlError::OK) {
              fprintf(stderr, "[InitTorqueStates][ID:%03d] Error disabling torque\n", it_id);
              return result;
            }
            torque_state_[it_id] = TORQUE_OFF;
          } else {
            torque_state_[it_id] = TORQUE_OFF;
            fprintf(
              stderr,
              "[InitTorqueStates][ID:%03d] Torque is enabled, cannot proceed. Set "
              "'disable_torque_at_init' parameter to 'true' to disable torque at initialization "
              "or disable torque manually.\n",
              it_id);
            return DxlError::DLX_HARDWARE_ERROR;
          }
        }

        fprintf(
          stderr, "[InitTorqueStates][ID:%03d] Current torque state: %s\n", it_id,
          torque_state_[it_id] ? "ON" : "OFF");
      }
    } catch (const std::exception & e) {
      fprintf(
        stderr, "[InitTorqueStates][ID:%03d] Error checking control item: %s\n", it_id,
        e.what());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::InitDxlComm(
  std::vector<uint8_t> id_arr,
  std::string port_name,
  std::string baudrate)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());  // port name
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler();

  if (port_handler_->openPort()) {
    fprintf(stderr, "Succeeded to open the port!\n");
  } else {
    fprintf(stderr, "Failed to open the port!\n");
    return DxlError::OPEN_PORT_FAIL;
  }

  if (port_handler_->setBaudRate(stoi(baudrate))) {
    fprintf(stderr, "Succeeded to change the [%d] baudrate!\n", stoi(baudrate));
  } else {
    fprintf(stderr, "Failed to change the baudrate!\n");
    return DxlError::OPEN_PORT_FAIL;
  }

  uint16_t dxl_model_number;
  uint8_t dxl_error = 0;

  for (auto it_id : id_arr) {
    fprintf(stderr, "[ID:%03d] Request ping\t", it_id);
    int dxl_comm_result = packet_handler_->ping(
      port_handler_, it_id, &dxl_model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(stderr, " - COMM_ERROR : %s\n", packet_handler_->getTxRxResult(dxl_comm_result));
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    } else if (dxl_error != 0) {
      fprintf(stderr, " - RX_PACKET_ERROR : %s\n", packet_handler_->getRxPacketError(dxl_error));
      uint32_t err = 0;
      if (ReadItem(it_id, "Hardware Error Status", err) == DxlError::OK) {
        fprintf(stderr, "[ID:%03d] Read Hardware Error Status : %x\n", it_id, err);
      }
      fprintf(stderr, "[ID:%03d] Hardware Error detected, rebooting...\n", it_id);
      Reboot(it_id);
      return DxlError::DLX_HARDWARE_ERROR;
    } else {
      fprintf(stderr, " - Ping succeeded. Dynamixel model number : %d\n", dxl_model_number);
    }

    try {
      dxl_info_.ReadDxlModelFile(it_id, dxl_model_number);
    } catch (const std::exception & e) {
      fprintf(stderr, "[InitDxlComm][ID:%03d] Error reading model file: %s\n", it_id, e.what());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
  }

  read_data_list_.clear();
  write_data_list_.clear();

  return DxlError::OK;
}

DxlError Dynamixel::Reboot(uint8_t id)
{
  fprintf(stderr, "[ID:%03d] Rebooting...\n", id);
  uint8_t dxl_error = 0;

  int dxl_comm_result = packet_handler_->reboot(port_handler_, id, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(
      stderr, "[ID:%03d] COMM_ERROR : %s\n",
      id, packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::DXL_REBOOT_FAIL;
  } else if (dxl_error != 0) {
    fprintf(
      stderr, "[ID:%03d] RX_PACKET_ERROR : %s\n",
      id, packet_handler_->getRxPacketError(dxl_error));
    return DxlError::DXL_REBOOT_FAIL;
  }

  fprintf(stderr, "[ID:%03d] Reboot Success!\n", id);

  return DxlError::OK;
}

void Dynamixel::RWDataReset()
{
  read_data_list_.clear();
  write_data_list_.clear();
}

DxlError Dynamixel::SetDxlReadItems(
  uint8_t id,
  uint8_t comm_id,
  std::vector<std::string> item_names,
  std::vector<std::shared_ptr<double>> data_vec_ptr)
{
  if (item_names.size() == 0) {
    fprintf(stderr, "[ID:%03d] No (Sync or Bulk) Read Item\n", id);
    return DxlError::OK;
  }

  if (item_names.size() != data_vec_ptr.size()) {
    fprintf(
      stderr, "Incorrect Read Data Size!!![%zu] [%zu]\n",
      item_names.size(), data_vec_ptr.size());
    return DxlError::SET_READ_ITEM_FAIL;
  }

  // Process items and get their addresses and sizes
  std::vector<uint8_t> item_ids;
  std::vector<uint16_t> item_addrs;
  std::vector<uint8_t> item_sizes;
  for (auto it_name : item_names) {
    uint16_t ITEM_ADDR;
    uint8_t ITEM_SIZE;
    if (dxl_info_.GetDxlControlItem(id, it_name, ITEM_ADDR, ITEM_SIZE) == false) {
      fprintf(
        stderr, "[ID:%03d] Cannot find control item in model file : %s\n", id,
        it_name.c_str());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
    item_ids.push_back(id);
    item_addrs.push_back(ITEM_ADDR);
    item_sizes.push_back(ITEM_SIZE);
  }

  // Check if there's already an entry with this comm_id
  for (auto & existing_item : read_data_list_) {
    if (existing_item.comm_id == comm_id) {
      // Found existing entry, append new items
      existing_item.id_arr.insert(existing_item.id_arr.end(), item_ids.begin(), item_ids.end());
      existing_item.item_name.insert(
        existing_item.item_name.end(), item_names.begin(),
        item_names.end());
      existing_item.item_addr.insert(
        existing_item.item_addr.end(), item_addrs.begin(),
        item_addrs.end());
      existing_item.item_size.insert(
        existing_item.item_size.end(), item_sizes.begin(),
        item_sizes.end());
      existing_item.item_data_ptr_vec.insert(
        existing_item.item_data_ptr_vec.end(),
        data_vec_ptr.begin(), data_vec_ptr.end());
      return DxlError::OK;
    }
  }

  // No existing entry found, create new one
  RWItemList read_item;
  read_item.comm_id = comm_id;
  read_item.id_arr = std::move(item_ids);
  read_item.item_name = std::move(item_names);
  read_item.item_addr = std::move(item_addrs);
  read_item.item_size = std::move(item_sizes);
  read_item.item_data_ptr_vec = std::move(data_vec_ptr);

  read_data_list_.push_back(read_item);

  return DxlError::OK;
}

DxlError Dynamixel::SetMultiDxlRead()
{
  read_type_ = checkReadType();

  fprintf(stderr, "Dynamixel Read Type : %s\n", read_type_ ? "bulk read" : "sync read");
  if (read_type_ == SYNC) {
    fprintf(stderr, "ID : ");
    for (auto it_read_data_list : read_data_list_) {
      fprintf(stderr, "%d, ", it_read_data_list.comm_id);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Read items : ");
    if (!read_data_list_.empty()) {
      for (auto it_read_data_list_item_name : read_data_list_.at(0).item_name) {
        fprintf(stderr, "\t%s", it_read_data_list_item_name.c_str());
      }
    } else {
      fprintf(stderr, "(none)");
    }
    fprintf(stderr, "\n");
  } else {
    for (auto it_read_data_list : read_data_list_) {
      fprintf(stderr, "ID : %d", it_read_data_list.comm_id);
      fprintf(stderr, "\tRead items : ");
      for (auto it_read_data_list_item_name : it_read_data_list.item_name) {
        fprintf(stderr, "\t%s", it_read_data_list_item_name.c_str());
      }
      fprintf(stderr, "\n");
    }
  }

  if (read_type_ == SYNC) {
    return SetSyncReadItemAndHandler();
  } else {
    return SetBulkReadItemAndHandler();
  }
}

DxlError Dynamixel::SetDxlWriteItems(
  uint8_t id,
  uint8_t comm_id,
  std::vector<std::string> item_names,
  std::vector<std::shared_ptr<double>> data_vec_ptr)
{
  if (item_names.size() == 0) {
    fprintf(stderr, "[ID:%03d] No (Sync or Bulk) Write Item\n", id);
    return DxlError::OK;
  }

  if (item_names.size() != data_vec_ptr.size()) {
    fprintf(
      stderr, "Incorrect Write Data Size!!![%zu] [%zu]\n",
      item_names.size(), data_vec_ptr.size());
    return DxlError::SET_WRITE_ITEM_FAIL;
  }

  std::vector<uint8_t> item_ids;
  std::vector<uint16_t> item_addrs;
  std::vector<uint8_t> item_sizes;
  for (auto it_name : item_names) {
    uint16_t ITEM_ADDR;
    uint8_t ITEM_SIZE;
    if (dxl_info_.GetDxlControlItem(id, it_name, ITEM_ADDR, ITEM_SIZE) == false) {
      fprintf(
        stderr, "[ID:%03d] Cannot find control item in model file : %s\n", id,
        it_name.c_str());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
    item_ids.push_back(id);
    item_addrs.push_back(ITEM_ADDR);
    item_sizes.push_back(ITEM_SIZE);
  }

  for (auto & existing_item : write_data_list_) {
    if (existing_item.comm_id == comm_id) {
      existing_item.id_arr.insert(existing_item.id_arr.end(), item_ids.begin(), item_ids.end());
      existing_item.item_name.insert(
        existing_item.item_name.end(), item_names.begin(),
        item_names.end());
      existing_item.item_addr.insert(
        existing_item.item_addr.end(), item_addrs.begin(),
        item_addrs.end());
      existing_item.item_size.insert(
        existing_item.item_size.end(), item_sizes.begin(),
        item_sizes.end());
      existing_item.item_data_ptr_vec.insert(
        existing_item.item_data_ptr_vec.end(),
        data_vec_ptr.begin(), data_vec_ptr.end());
      return DxlError::OK;
    }
  }

  // No existing entry found, create new one
  RWItemList write_item;
  write_item.comm_id = comm_id;
  write_item.id_arr = std::move(item_ids);
  write_item.item_name = std::move(item_names);
  write_item.item_addr = std::move(item_addrs);
  write_item.item_size = std::move(item_sizes);
  write_item.item_data_ptr_vec = std::move(data_vec_ptr);

  write_data_list_.push_back(write_item);

  return DxlError::OK;
}

DxlError Dynamixel::SetMultiDxlWrite()
{
  write_type_ = checkWriteType();

  fprintf(stderr, "Dynamixel Write Type : %s\n", write_type_ ? "bulk write" : "sync write");
  if (write_type_ == SYNC) {
    fprintf(stderr, "ID : ");
    for (auto it_id : write_data_list_) {
      fprintf(stderr, "%d, ", it_id.comm_id);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Write items : ");
    if (!write_data_list_.empty()) {
      for (auto it_name : write_data_list_.at(0).item_name) {
        fprintf(stderr, "\t%s", it_name.c_str());
      }
    } else {
      fprintf(stderr, "(none)");
    }
    fprintf(stderr, "\n");
  } else {
    for (auto it_id : write_data_list_) {
      fprintf(stderr, "ID : %d", it_id.comm_id);
      fprintf(stderr, "\tWrite items : ");
      for (auto it_name : it_id.item_name) {
        fprintf(stderr, "\t%s", it_name.c_str());
      }
      fprintf(stderr, "\n");
    }
  }

  if (write_type_ == SYNC) {
    return SetSyncWriteItemAndHandler();
  } else {
    return SetBulkWriteItemAndHandler();
  }
}

DxlError Dynamixel::DynamixelEnable(std::vector<uint8_t> id_arr)
{
  for (auto it_id : id_arr) {
    if (torque_state_[it_id] == TORQUE_OFF) {
      if (WriteItem(it_id, "Torque Enable", TORQUE_ON) < 0) {
        fprintf(stderr, "[ID:%03d] Cannot write \"Torque On\" command!\n", it_id);
        return DxlError::ITEM_WRITE_FAIL;
      }
      torque_state_[it_id] = TORQUE_ON;
      fprintf(stderr, "[ID:%03d] Torque ON\n", it_id);
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::DynamixelDisable(std::vector<uint8_t> id_arr)
{
  DxlError result = DxlError::OK;
  for (auto it_id : id_arr) {
    if (torque_state_[it_id] == TORQUE_ON) {
      if (WriteItem(it_id, "Torque Enable", TORQUE_OFF) < 0) {
        fprintf(stderr, "[ID:%03d] Cannot write \"Torque Off\" command!\n", it_id);
        result = DxlError::ITEM_WRITE_FAIL;
      } else {
        torque_state_[it_id] = TORQUE_OFF;
        fprintf(stderr, "[ID:%03d] Torque OFF\n", it_id);
      }
    }
  }
  return result;
}

// DxlError Dynamixel::SetOperatingMode(uint8_t dxl_id, uint8_t dynamixel_mode)
// {
//   if (WriteItem(dxl_id, "Operating Mode", dynamixel_mode) == false) {
//     return DxlError::ITEM_WRITE_FAIL;
//   }

//   fprintf(stderr, "[ID:%03d] Succeeded to set operating mode(", dxl_id);
//   if (dynamixel_mode == DXL_POSITION_CTRL_MODE) {
//     fprintf(stderr, "Position Control Mode)\n");
//   } else if (dynamixel_mode == DXL_CURRENT_CTRL_MODE) {
//     fprintf(stderr, "Current Control Mode)\n");
//   } else if (dynamixel_mode == DXL_VELOCITY_CTRL_MODE) {
//     fprintf(stderr, "Velocity Control Mode)\n");
//   } else {
//     fprintf(stderr, "Not Defined Control Mode)\n");
//   }

//   return DxlError::OK;
// }

DxlError Dynamixel::WriteItem(uint8_t id, std::string item_name, uint32_t data)
{
  uint16_t ITEM_ADDR;
  uint8_t ITEM_SIZE;
  if (dxl_info_.GetDxlControlItem(id, item_name, ITEM_ADDR, ITEM_SIZE) == false) {
    fprintf(
      stderr, "[WriteItem][ID:%03d] Cannot find control item in model file. : %s\n", id,
      item_name.c_str());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  return WriteItem(id, ITEM_ADDR, ITEM_SIZE, data);
}

DxlError Dynamixel::WriteItem(uint8_t id, uint16_t addr, uint8_t size, uint32_t data)
{
  uint8_t comm_id = comm_id_[id];
  for (int i = 0; i < MAX_COMM_RETRIES; i++) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    if (size == 1) {
      dxl_comm_result =
        packet_handler_->write1ByteTxRx(
        port_handler_, comm_id, addr, static_cast<uint8_t>(data),
        &dxl_error);
    } else if (size == 2) {
      dxl_comm_result = packet_handler_->write2ByteTxRx(
        port_handler_, comm_id, addr,
        static_cast<uint16_t>(data), &dxl_error);
    } else if (size == 4) {
      dxl_comm_result = packet_handler_->write4ByteTxRx(
        port_handler_, comm_id, addr,
        static_cast<uint32_t>(data), &dxl_error);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr,
        "[WriteItem][ID:%03d][comm_id:%03d] COMM_ERROR : %s (retry %d/%d)\n",
        id,
        comm_id,
        packet_handler_->getTxRxResult(dxl_comm_result),
        i + 1,
        MAX_COMM_RETRIES);
      if (i == MAX_COMM_RETRIES - 1) {
        return DxlError::ITEM_WRITE_FAIL;
      }
    } else if (dxl_error != 0) {
      fprintf(
        stderr,
        "[WriteItem][ID:%03d][comm_id:%03d] RX_PACKET_ERROR : %s (retry %d/%d)\n",
        id,
        comm_id,
        packet_handler_->getRxPacketError(dxl_error),
        i + 1,
        MAX_COMM_RETRIES);
      if (i == MAX_COMM_RETRIES - 1) {
        return DxlError::ITEM_WRITE_FAIL;
      }
    } else {
      return DxlError::OK;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  fprintf(stderr, "MAX_COMM_RETRIES should be set to 1 or more\n");
  return DxlError::ITEM_WRITE_FAIL;
}

DxlError Dynamixel::InsertWriteItemBuf(uint8_t id, std::string item_name, uint32_t data)
{
  RWItemBufInfo item;

  item.id = id;
  item.control_item.item_name = item_name;
  item.data = data;

  if (dxl_info_.GetDxlControlItem(
      id, item_name, item.control_item.address,
      item.control_item.size) == false)
  {
    fprintf(stderr, "Cannot find control item in model file.\n");
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  write_item_buf_.push_back(item);
  return DxlError::OK;
}

DxlError Dynamixel::WriteItemBuf()
{
  for (auto it_write_item : write_item_buf_) {
    // set torque state variable (ON or OFF)
    if (strcmp(it_write_item.control_item.item_name.c_str(), "Torque Enable") == 0) {
      if (WriteItem(
          it_write_item.id, it_write_item.control_item.address,
          it_write_item.control_item.size, it_write_item.data) < 0)
      {
        return DxlError::ITEM_WRITE_FAIL;
      }

      torque_state_[it_write_item.id] = it_write_item.data;
      fprintf(
        stderr, "[ID:%03d] Set Torque %s\n", it_write_item.id,
        torque_state_[it_write_item.id] ? "ON" : "OFF");
    } else {
      if ((dxl_info_.CheckDxlControlItem(it_write_item.id, "Torque Enable")) &&
        torque_state_[it_write_item.id] == TORQUE_ON)
      {
        if (WriteItem(it_write_item.id, "Torque Enable", TORQUE_OFF) < 0) {
          fprintf(
            stderr,
            "[ID:%03d] Cannot write \"Torque Off\" command! Cannot wirte a Item.\n",
            it_write_item.id);
          return DxlError::ITEM_WRITE_FAIL;
        }
        torque_state_[it_write_item.id] = TORQUE_OFF;
      }

      if (WriteItem(
          it_write_item.id, it_write_item.control_item.address,
          it_write_item.control_item.size, it_write_item.data) < 0)
      {
        return DxlError::ITEM_WRITE_FAIL;
      }
    }
  }
  write_item_buf_.clear();
  return DxlError::OK;
}

DxlError Dynamixel::ReadItem(uint8_t id, std::string item_name, uint32_t & data)
{
  uint16_t ITEM_ADDR;
  uint8_t ITEM_SIZE;
  if (dxl_info_.GetDxlControlItem(id, item_name, ITEM_ADDR, ITEM_SIZE) == false) {
    fprintf(
      stderr, "[ReadItem][ID:%03d] Cannot find control item in model file. : %s\n", id,
      item_name.c_str());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  uint8_t comm_id = comm_id_[id];
  for (int i = 0; i < MAX_COMM_RETRIES; i++) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    if (ITEM_SIZE == 1) {
      uint8_t read_data;
      dxl_comm_result = packet_handler_->read1ByteTxRx(
        port_handler_, comm_id, ITEM_ADDR,
        &read_data, &dxl_error);
      data = read_data;
    } else if (ITEM_SIZE == 2) {
      uint16_t read_data;
      dxl_comm_result = packet_handler_->read2ByteTxRx(
        port_handler_, comm_id, ITEM_ADDR,
        &read_data, &dxl_error);
      data = read_data;
    } else if (ITEM_SIZE == 4) {
      uint32_t read_data;
      dxl_comm_result = packet_handler_->read4ByteTxRx(
        port_handler_, comm_id, ITEM_ADDR,
        &read_data, &dxl_error);
      data = read_data;
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr,
        "[ReadItem][ID:%03d][comm_id:%03d] COMM_ERROR : %s (retry %d/%d)\n",
        id,
        comm_id,
        packet_handler_->getTxRxResult(dxl_comm_result),
        i + 1,
        MAX_COMM_RETRIES);
      if (i == MAX_COMM_RETRIES - 1) {
        return DxlError::ITEM_READ_FAIL;
      }
    } else if (dxl_error != 0) {
      fprintf(
        stderr,
        "[ReadItem][ID:%03d][comm_id:%03d] RX_PACKET_ERROR : %s (retry %d/%d)\n",
        id,
        comm_id,
        packet_handler_->getRxPacketError(dxl_error),
        i + 1,
        MAX_COMM_RETRIES);
      if (i == MAX_COMM_RETRIES - 1) {
        return DxlError::ITEM_READ_FAIL;
      }
    } else {
      return DxlError::OK;
    }
  }
  fprintf(stderr, "MAX_COMM_RETRIES should be set to 1 or more\n");
  return DxlError::ITEM_READ_FAIL;
}


DxlError Dynamixel::InsertReadItemBuf(uint8_t id, std::string item_name)
{
  RWItemBufInfo item;

  item.id = id;
  item.control_item.item_name = item_name;
  item.read_flag = false;

  if (dxl_info_.GetDxlControlItem(
      id, item_name, item.control_item.address,
      item.control_item.size) == false)
  {
    fprintf(stderr, "Cannot find control item in model file.\n");
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  read_item_buf_.push_back(item);
  return DxlError::OK;
}

DxlError Dynamixel::ReadItemBuf()
{
  for (auto it_read_item = read_item_buf_.begin(); it_read_item < read_item_buf_.end();
    it_read_item++)
  {
    if (it_read_item->read_flag == false) {
      uint8_t id = it_read_item->id;
      uint16_t addr = it_read_item->control_item.address;
      uint16_t size = it_read_item->control_item.size;

      int dxl_comm_result = COMM_TX_FAIL;
      uint8_t dxl_error = 0;

      if (size == 1) {
        uint8_t read_data;
        dxl_comm_result = packet_handler_->read1ByteTxRx(
          port_handler_, id, addr,
          &read_data, &dxl_error);
        it_read_item->data = read_data;
      } else if (size == 2) {
        uint16_t read_data;
        dxl_comm_result = packet_handler_->read2ByteTxRx(
          port_handler_, id, addr,
          &read_data, &dxl_error);
        it_read_item->data = read_data;
      } else if (size == 4) {
        uint32_t read_data;
        dxl_comm_result = packet_handler_->read4ByteTxRx(
          port_handler_, id, addr,
          &read_data, &dxl_error);
        it_read_item->data = read_data;
      }

      if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(
          stderr, "[ID:%03d] COMM_ERROR : %s\n",
          id,
          packet_handler_->getTxRxResult(dxl_comm_result));
        return DxlError::ITEM_READ_FAIL;
      } else if (dxl_error != 0) {
        fprintf(
          stderr, "[ID:%03d] RX_PACKET_ERROR : %s\n",
          id,
          packet_handler_->getRxPacketError(dxl_error));
        return DxlError::ITEM_READ_FAIL;
      } else {
        it_read_item->read_flag = true;
      }
    }
  }
  return DxlError::OK;
}

bool Dynamixel::CheckReadItemBuf(uint8_t id, std::string item_name)
{
  for (auto it_read_item_buf : read_item_buf_) {
    if (it_read_item_buf.id == id && it_read_item_buf.control_item.item_name == item_name) {
      return it_read_item_buf.read_flag;
    }
  }
  return false;
}
uint32_t Dynamixel::GetReadItemDataBuf(uint8_t id, std::string item_name)
{
  for (size_t i = 0; i < read_item_buf_.size(); i++) {
    if (read_item_buf_.at(i).id == id && read_item_buf_.at(i).control_item.item_name == item_name) {
      uint32_t res = read_item_buf_.at(i).data;
      read_item_buf_.erase(read_item_buf_.begin() + static_cast<int64_t>(i));
      return res;
    }
  }
  return 0;
}

std::string Dynamixel::DxlErrorToString(DxlError error_num)
{
  switch (error_num) {
    case OK:
      return "OK";
    case CANNOT_FIND_CONTROL_ITEM:
      return "CANNOT_FIND_CONTROL_ITEM";
    case OPEN_PORT_FAIL:
      return "OPEN_PORT_FAIL";
    case INDIRECT_ADDR_FAIL:
      return "INDIRECT_ADDR_FAIL";
    case ITEM_WRITE_FAIL:
      return "ITEM_WRITE_FAIL";
    case ITEM_READ_FAIL:
      return "ITEM_READ_FAIL";
    case SYNC_WRITE_FAIL:
      return "SYNC_WRITE_FAIL";
    case SYNC_READ_FAIL:
      return "SYNC_READ_FAIL";
    case SET_SYNC_WRITE_FAIL:
      return "SET_SYNC_WRITE_FAIL";
    case SET_SYNC_READ_FAIL:
      return "SET_SYNC_READ_FAIL";
    case BULK_WRITE_FAIL:
      return "BULK_WRITE_FAIL";
    case BULK_READ_FAIL:
      return "BULK_READ_FAIL";
    case SET_BULK_WRITE_FAIL:
      return "SET_BULK_WRITE_FAIL";
    case SET_BULK_READ_FAIL:
      return "SET_BULK_READ_FAIL";
    case SET_READ_ITEM_FAIL:
      return "SET_READ_ITEM_FAIL";
    case SET_WRITE_ITEM_FAIL:
      return "SET_WRITE_ITEM_FAIL";
    case DLX_HARDWARE_ERROR:
      return "DLX_HARDWARE_ERROR";
    case DXL_REBOOT_FAIL:
      return "DXL_REBOOT_FAIL";
    default:
      return "UNKNOWN_ERROR";
  }
}

DxlError Dynamixel::ReadMultiDxlData(double period_ms)
{
  if (read_data_list_.empty()) {
    return DxlError::OK;
  }

  if (read_type_ == SYNC) {
    return GetDxlValueFromSyncRead(period_ms);
  } else {
    return GetDxlValueFromBulkRead(period_ms);
  }
}

DxlError Dynamixel::WriteMultiDxlData()
{
  if (write_data_list_.empty()) {
    return DxlError::OK;
  }

  if (write_type_ == SYNC) {
    return SetDxlValueToSyncWrite();
  } else {
    return SetDxlValueToBulkWrite();
  }
}

bool Dynamixel::checkReadType()
{
  if (read_data_list_.size() == 1) {
    if (CheckIndirectReadAvailable(read_data_list_.at(0).comm_id) != DxlError::OK) {
      return BULK;
    }
  }

  for (size_t dxl_index = 1; dxl_index < read_data_list_.size(); dxl_index++) {
    // Check if Indirect Data Read address and size are different
    uint16_t indirect_addr[2];  // [i-1], [i]
    uint8_t indirect_size[2];   // [i-1], [i]

    if (CheckIndirectReadAvailable(read_data_list_.at(dxl_index - 1).comm_id) != DxlError::OK) {
      return BULK;
    }

    if (!dxl_info_.GetDxlControlItem(
        read_data_list_.at(dxl_index).comm_id, "Indirect Data Read", indirect_addr[1],
        indirect_size[1]) ||
      !dxl_info_.GetDxlControlItem(
        read_data_list_.at(dxl_index - 1).comm_id, "Indirect Data Read", indirect_addr[0],
        indirect_size[0]))
    {
      return BULK;
    }
    if (indirect_addr[1] != indirect_addr[0] || indirect_size[1] != indirect_size[0]) {
      return BULK;
    }

    if (read_data_list_.at(dxl_index).item_name.size() !=
      read_data_list_.at(dxl_index - 1).item_name.size())
    {
      return BULK;
    }
    for (size_t item_index = 0; item_index < read_data_list_.at(dxl_index).item_name.size();
      item_index++)
    {
      if (read_data_list_.at(dxl_index).item_name.at(item_index) !=
        read_data_list_.at(dxl_index - 1).item_name.at(item_index) ||
        read_data_list_.at(dxl_index).item_addr.at(item_index) !=
        read_data_list_.at(dxl_index - 1).item_addr.at(item_index) ||
        read_data_list_.at(dxl_index).item_size.at(item_index) !=
        read_data_list_.at(dxl_index - 1).item_size.at(item_index))
      {
        return BULK;
      }
    }
  }
  return SYNC;
}

bool Dynamixel::checkWriteType()
{
  if (write_data_list_.size() == 1) {
    if (CheckIndirectWriteAvailable(write_data_list_.at(0).comm_id) != DxlError::OK) {
      return BULK;
    }
  }

  for (size_t dxl_index = 1; dxl_index < write_data_list_.size(); dxl_index++) {
    // Check if Indirect Data Write address and size are different
    uint16_t indirect_addr[2];  // [i-1], [i]
    uint8_t indirect_size[2];   // [i-1], [i]

    if (CheckIndirectWriteAvailable(write_data_list_.at(dxl_index - 1).comm_id) != DxlError::OK) {
      return BULK;
    }

    if (!dxl_info_.GetDxlControlItem(
        write_data_list_.at(dxl_index).comm_id, "Indirect Data Write", indirect_addr[1],
        indirect_size[1]) ||
      !dxl_info_.GetDxlControlItem(
        write_data_list_.at(dxl_index - 1).comm_id, "Indirect Data Write", indirect_addr[0],
        indirect_size[0]))
    {
      return BULK;
    }
    if (indirect_addr[1] != indirect_addr[0] || indirect_size[1] != indirect_size[0]) {
      return BULK;
    }

    if (write_data_list_.at(dxl_index).item_name.size() !=
      write_data_list_.at(dxl_index - 1).item_name.size())
    {
      return BULK;
    }
    for (size_t item_index = 0; item_index < write_data_list_.at(dxl_index).item_name.size();
      item_index++)
    {
      if (write_data_list_.at(dxl_index).item_name.at(item_index) !=
        write_data_list_.at(dxl_index - 1).item_name.at(item_index) ||
        write_data_list_.at(dxl_index).item_addr.at(item_index) !=
        write_data_list_.at(dxl_index - 1).item_addr.at(item_index) ||
        write_data_list_.at(dxl_index).item_size.at(item_index) !=
        write_data_list_.at(dxl_index - 1).item_size.at(item_index))
      {
        return BULK;
      }
    }
  }
  return SYNC;
}

DxlError Dynamixel::CheckIndirectWriteAvailable(uint8_t id)
{
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  if (dxl_info_.GetDxlControlItem(
      id, "Indirect Address Write",
      INDIRECT_ADDR, INDIRECT_SIZE) == false)
  {
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetSyncReadItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_read_data : read_data_list_) {
    id_arr.push_back(it_read_data.comm_id);
  }

  DynamixelDisable(id_arr);
  ResetIndirectRead(id_arr);

  for (auto it_read_data : read_data_list_) {
    for (size_t item_index = 0; item_index < it_read_data.item_name.size(); item_index++) {
      auto result = AddIndirectRead(
        it_read_data.comm_id,
        it_read_data.item_name.at(item_index),
        it_read_data.item_addr.at(item_index),
        it_read_data.item_size.at(item_index));

      if (result == DxlError::OK) {
      } else {
        fprintf(
          stderr,
          "[ID:%03d] Failed to Set Indirect Address Read Item: [%s], Addr: %d, Size: %d, "
          "Error code: %d\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str(),
          it_read_data.item_addr.at(item_index),
          it_read_data.item_size.at(item_index),
          result);
        return DxlError::SET_SYNC_READ_FAIL;
      }
    }
  }

  if (SetSyncReadHandler(id_arr) != DxlError::OK) {
    fprintf(stderr, "Cannot set the SyncRead handler.\n");
    return DxlError::SYNC_READ_FAIL;
  }

  fprintf(stderr, "Success to set SyncRead handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetFastSyncReadHandler(std::vector<uint8_t> id_arr)
{
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    if (dxl_info_.GetDxlControlItem(it_id, "Indirect Data Read", IN_ADDR, IN_SIZE) == false) {
      fprintf(
        stderr,
        "Fail to set indirect address fast sync read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_SYNC_READ_FAIL;
    }
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;
  }
  fprintf(
    stderr,
    "set fast sync read (indirect addr) : addr %d, size %d\n",
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  if (group_fast_sync_read_) {
    delete group_fast_sync_read_;
    group_fast_sync_read_ = nullptr;
  }
  group_fast_sync_read_ =
    new dynamixel::GroupFastSyncRead(
    port_handler_, packet_handler_,
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  for (auto it_id : id_arr) {
    if (group_fast_sync_read_->addParam(it_id) != true) {
      fprintf(stderr, "[ID:%03d] groupFastSyncRead addparam failed", it_id);
      return DxlError::SET_SYNC_READ_FAIL;
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetSyncReadHandler(std::vector<uint8_t> id_arr)
{
  if (id_arr.size() == 0) {
    fprintf(stderr, "No Sync Read Item, not setting sync read handler\n");
    return DxlError::OK;
  }

  // Try to set up fast sync read first
  if (use_fast_read_protocol_) {
    DxlError fast_result = SetFastSyncReadHandler(id_arr);
    if (fast_result == DxlError::OK) {
      fprintf(stderr, "FastSyncRead handler set up successfully.\n");
      return DxlError::OK;
    } else {
      fprintf(stderr, "FastSyncRead handler failed, falling back to normal SyncRead.\n");
      use_fast_read_protocol_ = false;
    }
  }
  // Fallback to normal sync read
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(it_id, "Indirect Data Read", IN_ADDR, IN_SIZE) == false) {
      fprintf(
        stderr,
        "Fail to set indirect address sync read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_SYNC_READ_FAIL;
    }
    // Set indirect addr.
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;
  }
  fprintf(
    stderr,
    "set sync read (indirect addr) : addr %d, size %d\n",
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  if (group_sync_read_) {
    delete group_sync_read_;
    group_sync_read_ = nullptr;
  }
  group_sync_read_ =
    new dynamixel::GroupSyncRead(
    port_handler_, packet_handler_,
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  for (auto it_id : id_arr) {
    if (group_sync_read_->addParam(it_id) != true) {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", it_id);
      return DxlError::SET_SYNC_READ_FAIL;
    }
  }

  return DxlError::OK;
}

DxlError Dynamixel::GetDxlValueFromSyncRead(double period_ms)
{
  // Try fast sync read for the first 10 attempts after startup/handler setup.
  // If any of the first 10 attempts succeeds, use fast sync read permanently.
  // If all 10 attempts fail, permanently fallback to normal sync read.
  if (use_fast_read_protocol_ && group_fast_sync_read_ &&
    (fast_read_permanent_ || fast_read_fail_count_ < 10))
  {
    DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, true, true);
    if (comm_result == DxlError::OK) {
      // Success, process data, and use fast sync read permanently
      for (auto it_read_data : read_data_list_) {
        uint8_t id = it_read_data.comm_id;
        uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
        ProcessReadData(
          id,
          indirect_addr,
          it_read_data.id_arr,
          indirect_info_read_[id].item_name,
          indirect_info_read_[id].item_size,
          it_read_data.item_data_ptr_vec,
          [this](uint8_t id, uint16_t addr, uint8_t size) {
            return group_fast_sync_read_->getData(id, addr, size);
          });
      }
      // Mark as permanently using fast sync read after first success
      fast_read_permanent_ = true;
      return DxlError::OK;
    } else if (!fast_read_permanent_) {
      // Only increment fail count and fallback if not yet permanent
      ++fast_read_fail_count_;
      fprintf(stderr, "FastSyncRead TxRx failed (attempt %d/10)\n", fast_read_fail_count_);
      if (fast_read_fail_count_ >= 10) {
        // Permanently switch to normal sync read
        fprintf(
          stderr, "FastSyncRead failed 10 times, switching to normal SyncRead "
          "permanently.\n");
        use_fast_read_protocol_ = false;
        // Set up normal sync read handler
        std::vector<uint8_t> id_arr;
        for (auto it_read_data : read_data_list_) {
          id_arr.push_back(it_read_data.comm_id);
        }
        SetSyncReadHandler(id_arr);
      }
      // Return error for this attempt
      return DxlError::SYNC_READ_FAIL;
    } else {
      // If permanent, ignore failures and keep using fast sync read
      return comm_result;
    }
  }
  // Use normal sync read
  DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, true, false);
  if (comm_result != DxlError::OK) {
    return comm_result;
  }
  for (auto it_read_data : read_data_list_) {
    uint8_t id = it_read_data.comm_id;
    uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
    ProcessReadData(
      id,
      indirect_addr,
      it_read_data.id_arr,
      indirect_info_read_[id].item_name,
      indirect_info_read_[id].item_size,
      it_read_data.item_data_ptr_vec,
      [this](uint8_t id, uint16_t addr, uint8_t size) {
        return group_sync_read_->getData(id, addr, size);
      });
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkReadItemAndHandler()
{
  if (group_fast_bulk_read_) {
    delete group_fast_bulk_read_;
    group_fast_bulk_read_ = nullptr;
  }
  group_fast_bulk_read_ = new dynamixel::GroupFastBulkRead(port_handler_, packet_handler_);

  if (group_bulk_read_) {
    delete group_bulk_read_;
    group_bulk_read_ = nullptr;
  }
  group_bulk_read_ = new dynamixel::GroupBulkRead(port_handler_, packet_handler_);

  std::vector<uint8_t> indirect_id_arr;
  std::vector<uint8_t> direct_id_arr;

  std::vector<RWItemList> indirect_read_data_list;
  std::vector<RWItemList> direct_read_data_list;

  for (auto it_read_data : read_data_list_) {
    if (CheckIndirectReadAvailable(it_read_data.comm_id) == DxlError::OK) {
      indirect_id_arr.push_back(it_read_data.comm_id);
      indirect_read_data_list.push_back(it_read_data);
    } else {
      direct_id_arr.push_back(it_read_data.comm_id);
      direct_read_data_list.push_back(it_read_data);
    }
  }

  for (auto it_read_data : direct_read_data_list) {
    if (it_read_data.item_addr.empty() || it_read_data.item_size.empty()) {
      continue;
    }
    // Calculate min address and max end address
    uint16_t min_addr = it_read_data.item_addr[0];
    uint16_t max_end_addr = it_read_data.item_addr[0] + it_read_data.item_size[0];
    for (size_t item_index = 1; item_index < it_read_data.item_addr.size(); ++item_index) {
      uint16_t addr = it_read_data.item_addr[item_index];
      uint16_t size = it_read_data.item_size[item_index];
      if (addr < min_addr) {min_addr = addr;}
      if (addr + size > max_end_addr) {max_end_addr = addr + size;}
    }
    uint8_t total_size = max_end_addr - min_addr;
    // Concatenate all item names with '+'
    std::string group_item_names;
    for (size_t i = 0; i < it_read_data.item_name.size(); ++i) {
      group_item_names += it_read_data.item_name[i];
      if (i + 1 < it_read_data.item_name.size()) {group_item_names += " + ";}
    }
    // Call AddDirectRead once per id
    if (AddDirectRead(
        it_read_data.comm_id,
        group_item_names,
        min_addr,
        total_size) != DxlError::OK)
    {
      fprintf(stderr, "Cannot set the BulkRead handler.\n");
      return DxlError::BULK_READ_FAIL;
    }
    fprintf(
      stderr,
      "set bulk read (direct addr) : addr %d, size %d\n",
      min_addr, total_size);
  }

  DynamixelDisable(indirect_id_arr);
  ResetIndirectRead(indirect_id_arr);

  for (auto it_read_data : indirect_read_data_list) {
    for (size_t item_index = 0; item_index < it_read_data.item_name.size();
      item_index++)
    {
      auto result = AddIndirectRead(
        it_read_data.comm_id,
        it_read_data.item_name.at(item_index),
        it_read_data.item_addr.at(item_index),
        it_read_data.item_size.at(item_index));

      if (result == DxlError::OK) {
        fprintf(
          stderr, "[ID:%03d] Add Indirect Address Read Item : [%s]\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str());
      } else if (result == DxlError::INDIRECT_ADDR_FAIL) {
        fprintf(
          stderr,
          "[ID:%03d] Failed to Set Indirect Address Read Item : [%s], Addr: %d, Size: %d, "
          "Error code: %d\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str(),
          it_read_data.item_addr.at(item_index),
          it_read_data.item_size.at(item_index),
          result);
        return DxlError::INDIRECT_ADDR_FAIL;
      } else if (result == DxlError::CANNOT_FIND_CONTROL_ITEM) {
        fprintf(
          stderr, "[ID:%03d] 'Indirect Address Read' is not defined in control table, "
          "Cannot set Indirect Address Read for : [%s]\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str());
        return DxlError::CANNOT_FIND_CONTROL_ITEM;
      }
    }
  }

  if (SetBulkReadHandler(indirect_id_arr) != DxlError::OK) {
    fprintf(stderr, "Cannot set the BulkRead handler.\n");
    return DxlError::BULK_READ_FAIL;
  }

  fprintf(stderr, "Success to set BulkRead handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetFastBulkReadHandler(std::vector<uint8_t> id_arr)
{
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(it_id, "Indirect Data Read", IN_ADDR, IN_SIZE) == false) {
      fprintf(
        stderr,
        "Fail to set indirect address fast bulk read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_BULK_READ_FAIL;
    }
    // Set indirect addr.
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;

    fprintf(
      stderr,
      "set fast bulk read (indirect addr) : addr %d, size %d\n",
      IN_ADDR, indirect_info_read_[it_id].size);
  }

  for (auto it_id : id_arr) {
    uint8_t ID = it_id;
    uint16_t ADDR = indirect_info_read_[ID].indirect_data_addr;
    uint8_t SIZE = indirect_info_read_[ID].size;
    auto addParamResult = group_fast_bulk_read_->addParam(ID, ADDR, SIZE);
    if (addParamResult) {  // success
      fprintf(
        stderr, "[ID:%03d] Add BulkRead item : [Indirect Item Data][%d][%d]\n", ID, ADDR, SIZE);
    } else {
      fprintf(
        stderr, "[ID:%03d] Failed to BulkRead item : [Indirect Item Data]]\n", ID);
      return DxlError::SET_BULK_READ_FAIL;
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkReadHandler(std::vector<uint8_t> id_arr)
{
  // Try to set up fast bulk read first
  if (use_fast_read_protocol_) {
    DxlError fast_result = SetFastBulkReadHandler(id_arr);
    if (fast_result == DxlError::OK) {
      fprintf(stderr, "FastBulkRead handler set up successfully.\n");
      return DxlError::OK;
    } else {
      fprintf(stderr, "FastBulkRead handler failed, falling back to normal BulkRead.\n");
      use_fast_read_protocol_ = false;
    }
  }

  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(it_id, "Indirect Data Read", IN_ADDR, IN_SIZE) == false) {
      fprintf(
        stderr,
        "Fail to set indirect address bulk read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_BULK_READ_FAIL;
    }
    // Set indirect addr.
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;

    fprintf(
      stderr,
      "set bulk read (indirect addr) : addr %d, size %d\n",
      IN_ADDR, indirect_info_read_[it_id].size);
  }

  for (auto it_id : id_arr) {
    uint8_t ID = it_id;
    uint16_t ADDR = indirect_info_read_[ID].indirect_data_addr;
    uint8_t SIZE = indirect_info_read_[ID].size;
    auto addParamResult = group_bulk_read_->addParam(ID, ADDR, SIZE);
    if (addParamResult) {  // success
      fprintf(
        stderr, "[ID:%03d] Add BulkRead item : [Indirect Item Data][%d][%d]\n", ID, ADDR, SIZE);
    } else {
      fprintf(
        stderr, "[ID:%03d] Failed to BulkRead item : [Indirect Item Data]]\n", ID);
      return DxlError::SET_BULK_READ_FAIL;
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::AddDirectRead(
  uint8_t id, std::string item_name, uint16_t item_addr, uint8_t item_size)
{
  if (group_bulk_read_->addParam(id, item_addr, item_size) == true) {
    fprintf(
      stderr, "[ID:%03d] Add BulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
  } else {
    fprintf(
      stderr, "[ID:%03d] Failed to BulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
    return DxlError::SET_BULK_READ_FAIL;
  }

  if (group_fast_bulk_read_->addParam(id, item_addr, item_size) == true) {
    fprintf(
      stderr, "[ID:%03d] Add FastBulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
  } else {
    fprintf(
      stderr, "[ID:%03d] Failed to FastBulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
    return DxlError::SET_BULK_READ_FAIL;
  }
  return DxlError::OK;
}

DxlError Dynamixel::GetDxlValueFromBulkRead(double period_ms)
{
  // Try fast bulk read for the first 10 attempts after startup/handler setup.
  // If any of the first 10 attempts succeeds, use fast bulk read permanently.
  // If all 10 attempts fail, permanently fallback to normal bulk read.
  if (use_fast_read_protocol_ && group_fast_bulk_read_ &&
    (fast_read_permanent_ || fast_read_fail_count_ < 10))
  {
    DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, false, true);
    if (comm_result == DxlError::OK) {
      // Success, process data, and use fast bulk read permanently
      if (group_bulk_read_) {
        delete group_bulk_read_;
        group_bulk_read_ = nullptr;
      }

      for (auto it_read_data : read_data_list_) {
        uint8_t id = it_read_data.comm_id;
        uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
        if (CheckIndirectReadAvailable(id) != DxlError::OK) {
          ProcessDirectReadData(
            id,
            it_read_data.item_addr,
            it_read_data.item_size,
            it_read_data.item_data_ptr_vec,
            [this](uint8_t id, uint16_t addr, uint8_t size) {
              return group_fast_bulk_read_->getData(id, addr, size);
            });
        } else {
          ProcessReadData(
            id,
            indirect_addr,
            it_read_data.id_arr,
            indirect_info_read_[id].item_name,
            indirect_info_read_[id].item_size,
            it_read_data.item_data_ptr_vec,
            [this](uint8_t id, uint16_t addr, uint8_t size) {
              return group_fast_bulk_read_->getData(id, addr, size);
            });
        }
      }
      // Mark as permanently using fast bulk read after first success
      fast_read_permanent_ = true;
      return DxlError::OK;
    } else if (!fast_read_permanent_) {
      // Only increment fail count and fallback if not yet permanent
      ++fast_read_fail_count_;
      fprintf(stderr, "FastBulkRead TxRx failed (attempt %d/10)\n", fast_read_fail_count_);
      if (fast_read_fail_count_ >= 10) {
        if (group_fast_bulk_read_) {
          delete group_fast_bulk_read_;
          group_fast_bulk_read_ = nullptr;
        }
        // Permanently switch to normal bulk read
        fprintf(
          stderr,
          "FastBulkRead failed 10 times, switching to normal BulkRead permanently.\n");
        use_fast_read_protocol_ = false;
        // Set up normal bulk read handler
        std::vector<uint8_t> indirect_id_arr;

        for (auto it_read_data : read_data_list_) {
          if (CheckIndirectReadAvailable(it_read_data.comm_id) == DxlError::OK) {
            indirect_id_arr.push_back(it_read_data.comm_id);
          }
        }

        SetBulkReadHandler(indirect_id_arr);
      }
      // Return error for this attempt
      return DxlError::BULK_READ_FAIL;
    } else {
      // If permanent, ignore failures and keep using fast bulk read
      return comm_result;
    }
  }
  DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, false, false);
  if (comm_result != DxlError::OK) {
    return comm_result;
  }

  for (auto it_read_data : read_data_list_) {
    uint8_t id = it_read_data.comm_id;
    uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
    if (CheckIndirectReadAvailable(id) != DxlError::OK) {
      ProcessDirectReadData(
        id,
        it_read_data.item_addr,
        it_read_data.item_size,
        it_read_data.item_data_ptr_vec,
        [this](uint8_t id, uint16_t addr, uint8_t size) {
          return group_bulk_read_->getData(id, addr, size);
        });
    } else {
      ProcessReadData(
        id,
        indirect_addr,
        it_read_data.id_arr,
        indirect_info_read_[id].item_name,
        indirect_info_read_[id].item_size,
        it_read_data.item_data_ptr_vec,
        [this](uint8_t id, uint16_t addr, uint8_t size) {
          return group_bulk_read_->getData(id, addr, size);
        });
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::ProcessReadCommunication(
  dynamixel::PortHandler * port_handler,
  double period_ms,
  bool is_sync,
  bool is_fast)
{
  int dxl_comm_result;

  // Send packet
  if (is_sync) {
    if (is_fast && group_fast_sync_read_) {
      dxl_comm_result = group_fast_sync_read_->txPacket();
    } else if (group_sync_read_) {
      dxl_comm_result = group_sync_read_->txPacket();
    } else {
      return DxlError::SYNC_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Tx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastSyncRead" : "SyncRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::SYNC_READ_FAIL;
    }
  } else {
    // Bulk read
    if (is_fast && group_fast_bulk_read_) {
      dxl_comm_result = group_fast_bulk_read_->txPacket();
    } else if (group_bulk_read_) {
      dxl_comm_result = group_bulk_read_->txPacket();
    } else {
      return DxlError::BULK_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Tx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastBulkRead" : "BulkRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::BULK_READ_FAIL;
    }
  }

  // Set timeout if period_ms is specified
  if (period_ms > 0) {
    port_handler->setPacketTimeout(period_ms);
  }

  // Receive packet
  if (is_sync) {
    if (is_fast && group_fast_sync_read_) {
      dxl_comm_result = group_fast_sync_read_->rxPacket();
    } else if (group_sync_read_) {
      dxl_comm_result = group_sync_read_->rxPacket();
    } else {
      return DxlError::SYNC_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Rx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastSyncRead" : "SyncRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::SYNC_READ_FAIL;
    }
  } else {
    // Bulk read
    if (is_fast && group_fast_bulk_read_) {
      dxl_comm_result = group_fast_bulk_read_->rxPacket();
    } else if (group_bulk_read_) {
      dxl_comm_result = group_bulk_read_->rxPacket();
    } else {
      return DxlError::BULK_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Rx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastBulkRead" : "BulkRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::BULK_READ_FAIL;
    }
  }

  return DxlError::OK;
}

DxlError Dynamixel::ProcessReadData(
  uint8_t comm_id,
  uint16_t indirect_addr,
  const std::vector<uint8_t> & id_arr,
  const std::vector<std::string> & item_names,
  const std::vector<uint8_t> & item_sizes,
  const std::vector<std::shared_ptr<double>> & data_ptrs,
  std::function<uint32_t(uint8_t, uint16_t, uint8_t)> get_data_func)
{
  uint16_t current_addr = indirect_addr;

  for (size_t item_index = 0; item_index < item_names.size(); item_index++) {
    uint8_t ID = id_arr[item_index];
    uint8_t size = item_sizes[item_index];
    if (item_index > 0) {current_addr += item_sizes[item_index - 1];}

    uint32_t dxl_getdata = get_data_func(comm_id, current_addr, size);

    if (item_names[item_index] == "Present Position") {
      *data_ptrs[item_index] = dxl_info_.ConvertValueToRadian(
        ID,
        static_cast<int32_t>(dxl_getdata));
    } else if (item_names[item_index] == "Present Velocity") {
      *data_ptrs[item_index] = dxl_info_.ConvertValueRPMToVelocityRPS(
        ID,
        static_cast<int32_t>(dxl_getdata));
    } else if (item_names[item_index] == "Present Current") {
      *data_ptrs[item_index] = dxl_info_.ConvertCurrentToEffort(
        ID,
        static_cast<int16_t>(dxl_getdata));
    } else {
      *data_ptrs[item_index] = static_cast<double>(dxl_getdata);
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::ProcessDirectReadData(
  uint8_t comm_id,
  const std::vector<uint16_t> & item_addrs,
  const std::vector<uint8_t> & item_sizes,
  const std::vector<std::shared_ptr<double>> & data_ptrs,
  std::function<uint32_t(uint8_t, uint16_t, uint8_t)> get_data_func)
{
  for (size_t item_index = 0; item_index < item_addrs.size(); item_index++) {
    uint16_t current_addr = item_addrs[item_index];
    uint8_t size = item_sizes[item_index];

    uint32_t dxl_getdata = get_data_func(comm_id, current_addr, size);

    *data_ptrs[item_index] = static_cast<double>(dxl_getdata);
  }
  return DxlError::OK;
}

void Dynamixel::ResetIndirectRead(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.indirect_data_addr = 0;
  temp.size = 0;
  temp.cnt = 0;
  temp.item_name.clear();
  temp.item_size.clear();
  for (auto it_id : id_arr) {
    indirect_info_read_[it_id] = temp;
  }
}

DxlError Dynamixel::CheckIndirectReadAvailable(uint8_t id)
{
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  if (dxl_info_.GetDxlControlItem(
      id, "Indirect Address Read",
      INDIRECT_ADDR, INDIRECT_SIZE) == false)
  {
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
  return DxlError::OK;
}

DxlError Dynamixel::AddIndirectRead(
  uint8_t id,
  std::string item_name,
  uint16_t item_addr,
  uint8_t item_size)
{
  // write address to indirect addr control item
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  if (dxl_info_.GetDxlControlItem(
      id, "Indirect Address Read",
      INDIRECT_ADDR, INDIRECT_SIZE) == true)
  {
    uint8_t using_size = indirect_info_read_[id].size;

    for (uint16_t i = 0; i < item_size; i++) {
      DxlError write_result = DxlError::INDIRECT_ADDR_FAIL;
      uint16_t addr = INDIRECT_ADDR + (using_size * 2);
      uint16_t item_addr_i = item_addr + i;

      write_result = WriteItem(id, addr, 2, item_addr_i);

      if (write_result != DxlError::OK) {
        fprintf(stderr, "[AddIndirectRead][ID:%03d] WriteItem failed\n", id);
        return DxlError::INDIRECT_ADDR_FAIL;
      }
      using_size++;
    }
    indirect_info_read_[id].size = using_size;
    indirect_info_read_[id].cnt += 1;
    indirect_info_read_[id].item_name.push_back(item_name);
    indirect_info_read_[id].item_size.push_back(item_size);

    return DxlError::OK;
  } else {
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
}

DxlError Dynamixel::SetSyncWriteItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_write_data : write_data_list_) {
    id_arr.push_back(it_write_data.comm_id);
  }

  DynamixelDisable(id_arr);
  ResetIndirectWrite(id_arr);

  for (auto it_write_data : write_data_list_) {
    for (size_t item_index = 0; item_index < it_write_data.item_name.size();
      item_index++)
    {
      if (AddIndirectWrite(
          it_write_data.comm_id,
          it_write_data.item_name.at(item_index),
          it_write_data.item_addr.at(item_index),
          it_write_data.item_size.at(item_index)) != DxlError::OK)
      {
        fprintf(stderr, "Cannot set the SyncWrite handler.\n");
        return DxlError::SYNC_WRITE_FAIL;
      }
    }
  }

  if (SetSyncWriteHandler(id_arr) < 0) {
    fprintf(stderr, "Cannot set the SyncWrite handler.\n");
    return DxlError::SYNC_WRITE_FAIL;
  }

  fprintf(stderr, "Success to set SyncWrite handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetSyncWriteHandler(std::vector<uint8_t> id_arr)
{
  if (id_arr.size() == 0) {
    fprintf(stderr, "No Sync Write Item, not setting sync write handler\n");
    return DxlError::OK;
  }

  uint16_t INDIRECT_ADDR = 0;
  uint8_t INDIRECT_SIZE;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(
        it_id, "Indirect Data Write", INDIRECT_ADDR,
        INDIRECT_SIZE) == false)
    {
      fprintf(
        stderr,
        "Fail to set indirect address sync write. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_SYNC_WRITE_FAIL;
    }
    // Set indirect addr.
    indirect_info_write_[it_id].indirect_data_addr = INDIRECT_ADDR;
  }
  fprintf(
    stderr,
    "set sync write (indirect addr) : addr %d, size %d\n",
    INDIRECT_ADDR, indirect_info_write_[id_arr.at(0)].size);

  group_sync_write_ =
    new dynamixel::GroupSyncWrite(
    port_handler_, packet_handler_,
    INDIRECT_ADDR, indirect_info_write_[id_arr.at(0)].size);

  return DxlError::OK;
}
DxlError Dynamixel::SetDxlValueToSyncWrite()
{
  for (auto it_write_data : write_data_list_) {
    uint8_t comm_id = it_write_data.comm_id;
    uint8_t * param_write_value = new uint8_t[indirect_info_write_[comm_id].size];
    uint8_t added_byte = 0;

    for (uint16_t item_index = 0; item_index < indirect_info_write_[comm_id].cnt; item_index++) {
      double data = *it_write_data.item_data_ptr_vec.at(item_index);
      uint8_t ID = it_write_data.id_arr.at(item_index);
      if (indirect_info_write_[comm_id].item_name.at(item_index) == "Goal Position") {
        int32_t goal_position = dxl_info_.ConvertRadianToValue(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
        param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
        param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
        param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_position));
      } else if (indirect_info_write_[comm_id].item_name.at(item_index) == "Goal Velocity") {
        int32_t goal_velocity = dxl_info_.ConvertVelocityRPSToValueRPM(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity));
        param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity));
        param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity));
        param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity));
      } else if (indirect_info_write_[comm_id].item_name.at(item_index) == "Goal Current") {
        int16_t goal_current = dxl_info_.ConvertEffortToCurrent(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(goal_current);
        param_write_value[added_byte + 1] = DXL_HIBYTE(goal_current);
      } else {
        if (indirect_info_write_[comm_id].item_size.at(item_index) == 4) {
          int32_t value = static_cast<int32_t>(data);
          param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(value));
          param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(value));
          param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(value));
          param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(value));
        } else if (indirect_info_write_[comm_id].item_size.at(item_index) == 2) {
          int16_t value = static_cast<int16_t>(data);
          param_write_value[added_byte + 0] = DXL_LOBYTE(value);
          param_write_value[added_byte + 1] = DXL_HIBYTE(value);
        } else if (indirect_info_write_[comm_id].item_size.at(item_index) == 1) {
          param_write_value[added_byte] = static_cast<uint8_t>(data);
        }
      }
      added_byte += indirect_info_write_[comm_id].item_size.at(item_index);
    }

    if (group_sync_write_->addParam(comm_id, param_write_value) != true) {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed\n", comm_id);
      return DxlError::SYNC_WRITE_FAIL;
    }
  }

  int dxl_comm_result = group_sync_write_->txPacket();
  group_sync_write_->clearParam();

  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(stderr, "%s\n", packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::SYNC_WRITE_FAIL;
  } else {
    return DxlError::OK;
  }
}

DxlError Dynamixel::SetBulkWriteItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_write_data : write_data_list_) {
    id_arr.push_back(it_write_data.comm_id);
  }

  group_bulk_write_ = new dynamixel::GroupBulkWrite(port_handler_, packet_handler_);

  std::vector<uint8_t> indirect_id_arr;
  std::vector<uint8_t> direct_id_arr;

  std::vector<RWItemList> indirect_write_data_list;
  std::vector<RWItemList> direct_write_data_list;

  for (auto it_write_data : write_data_list_) {
    if (CheckIndirectWriteAvailable(it_write_data.comm_id) == DxlError::OK) {
      indirect_id_arr.push_back(it_write_data.comm_id);
      indirect_write_data_list.push_back(it_write_data);
    } else {
      direct_id_arr.push_back(it_write_data.comm_id);
      direct_write_data_list.push_back(it_write_data);
    }
  }

  ResetDirectWrite(direct_id_arr);

  // Handle direct writes
  for (auto it_write_data : direct_write_data_list) {
    if (it_write_data.item_addr.empty() || it_write_data.item_size.empty()) {
      continue;
    }
    // Calculate min address and max end address
    uint16_t min_addr = it_write_data.item_addr[0];
    uint16_t max_end_addr = it_write_data.item_addr[0] + it_write_data.item_size[0];
    for (size_t item_index = 1; item_index < it_write_data.item_addr.size(); ++item_index) {
      uint16_t addr = it_write_data.item_addr[item_index];
      uint16_t size = it_write_data.item_size[item_index];
      if (addr < min_addr) {min_addr = addr;}
      if (addr + size > max_end_addr) {max_end_addr = addr + size;}
    }
    uint8_t total_size = max_end_addr - min_addr;

    // Check for gaps between items
    std::vector<std::pair<uint16_t, uint16_t>> addr_ranges;
    for (size_t item_index = 0; item_index < it_write_data.item_addr.size(); ++item_index) {
      addr_ranges.push_back(
        {
          it_write_data.item_addr[item_index],
          it_write_data.item_addr[item_index] + it_write_data.item_size[item_index]
        });
    }
    std::sort(addr_ranges.begin(), addr_ranges.end());

    for (size_t i = 0; i < addr_ranges.size() - 1; ++i) {
      if (addr_ranges[i].second != addr_ranges[i + 1].first) {
        fprintf(
          stderr, "[ID:%03d] Error: Gap detected between items at addresses %d and %d\n",
          it_write_data.comm_id, addr_ranges[i].second, addr_ranges[i + 1].first);
        return DxlError::BULK_WRITE_FAIL;
      }
    }

    // Store direct write info
    direct_info_write_[it_write_data.comm_id].indirect_data_addr = min_addr;
    direct_info_write_[it_write_data.comm_id].size = total_size;
    direct_info_write_[it_write_data.comm_id].cnt = it_write_data.item_name.size();
    direct_info_write_[it_write_data.comm_id].item_name = it_write_data.item_name;
    direct_info_write_[it_write_data.comm_id].item_size = it_write_data.item_size;

    fprintf(
      stderr,
      "set bulk write (direct addr) : addr %d, size %d\n",
      min_addr, total_size);
  }

  // Handle indirect writes
  DynamixelDisable(indirect_id_arr);
  ResetIndirectWrite(indirect_id_arr);

  for (auto it_write_data : indirect_write_data_list) {
    for (size_t item_index = 0; item_index < it_write_data.item_name.size();
      item_index++)
    {
      if (AddIndirectWrite(
          it_write_data.comm_id,
          it_write_data.item_name.at(item_index),
          it_write_data.item_addr.at(item_index),
          it_write_data.item_size.at(item_index)) != DxlError::OK)
      {
        fprintf(stderr, "Cannot set the BulkWrite handler.\n");
        return DxlError::BULK_WRITE_FAIL;
      }

      fprintf(
        stderr, "[ID:%03d] Add Indirect Address Write Item : [%s]\n",
        it_write_data.comm_id,
        it_write_data.item_name.at(item_index).c_str());
    }
  }

  if (SetBulkWriteHandler(indirect_id_arr) < 0) {
    fprintf(stderr, "Cannot set the BulkWrite handler.\n");
    return DxlError::BULK_WRITE_FAIL;
  }

  fprintf(stderr, "Success to set BulkWrite handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkWriteHandler(std::vector<uint8_t> id_arr)
{
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(it_id, "Indirect Data Write", IN_ADDR, IN_SIZE) == false) {
      fprintf(
        stderr,
        "Fail to set indirect address bulk write. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_BULK_WRITE_FAIL;
    }
    // Set indirect addr.
    indirect_info_write_[it_id].indirect_data_addr = IN_ADDR;

    fprintf(
      stderr,
      "set bulk write (indirect addr) : addr %d, size %d\n",
      IN_ADDR, indirect_info_write_[it_id].size);
  }

  return DxlError::OK;
}

DxlError Dynamixel::SetDxlValueToBulkWrite()
{
  for (auto it_write_data : write_data_list_) {
    uint8_t comm_id = it_write_data.comm_id;
    uint8_t * param_write_value;
    uint8_t added_byte = 0;

    // Check if this is a direct write
    if (direct_info_write_.find(comm_id) != direct_info_write_.end()) {
      param_write_value = new uint8_t[direct_info_write_[comm_id].size];

      for (uint16_t item_index = 0; item_index < direct_info_write_[comm_id].cnt; item_index++) {
        double data = *it_write_data.item_data_ptr_vec.at(item_index);
        uint8_t size = direct_info_write_[comm_id].item_size.at(item_index);
        if (size == 4) {
          int32_t value = static_cast<int32_t>(data);
          param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(value));
          param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(value));
          param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(value));
          param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(value));
        } else if (size == 2) {
          int16_t value = static_cast<int16_t>(data);
          param_write_value[added_byte + 0] = DXL_LOBYTE(value);
          param_write_value[added_byte + 1] = DXL_HIBYTE(value);
        } else if (size == 1) {
          param_write_value[added_byte] = static_cast<uint8_t>(data);
        }
        added_byte += size;
      }

      if (group_bulk_write_->addParam(
          comm_id,
          direct_info_write_[comm_id].indirect_data_addr,
          direct_info_write_[comm_id].size,
          param_write_value) != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed\n", comm_id);
        return DxlError::BULK_WRITE_FAIL;
      }
    } else {
      // Handle indirect write
      param_write_value = new uint8_t[indirect_info_write_[comm_id].size];

      for (uint16_t item_index = 0; item_index < indirect_info_write_[comm_id].cnt; item_index++) {
        double data = *it_write_data.item_data_ptr_vec.at(item_index);
        uint8_t ID = it_write_data.id_arr.at(item_index);
        if (indirect_info_write_[comm_id].item_name.at(item_index) == "Goal Position") {
          int32_t goal_position = dxl_info_.ConvertRadianToValue(ID, data);
          param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
          param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
          param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
          param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_position));
        } else if (indirect_info_write_[comm_id].item_name.at(item_index) == "Goal Velocity") {
          int32_t goal_velocity = dxl_info_.ConvertVelocityRPSToValueRPM(ID, data);
          param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity));
          param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity));
          param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity));
          param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity));
        } else if (indirect_info_write_[comm_id].item_name.at(item_index) == "Goal Current") {
          int16_t goal_current = dxl_info_.ConvertEffortToCurrent(ID, data);
          param_write_value[added_byte + 0] = DXL_LOBYTE(goal_current);
          param_write_value[added_byte + 1] = DXL_HIBYTE(goal_current);
        } else {
          if (indirect_info_write_[comm_id].item_size.at(item_index) == 4) {
            int32_t value = static_cast<int32_t>(data);
            param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(value));
            param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(value));
            param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(value));
            param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(value));
          } else if (indirect_info_write_[comm_id].item_size.at(item_index) == 2) {
            int16_t value = static_cast<int16_t>(data);
            param_write_value[added_byte + 0] = DXL_LOBYTE(value);
            param_write_value[added_byte + 1] = DXL_HIBYTE(value);
          } else if (indirect_info_write_[comm_id].item_size.at(item_index) == 1) {
            param_write_value[added_byte] = static_cast<uint8_t>(data);
          }
        }
        added_byte += indirect_info_write_[comm_id].item_size.at(item_index);
      }

      if (group_bulk_write_->addParam(
          comm_id,
          indirect_info_write_[comm_id].indirect_data_addr,
          indirect_info_write_[comm_id].size,
          param_write_value) != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed\n", comm_id);
        return DxlError::BULK_WRITE_FAIL;
      }
    }
  }

  int dxl_comm_result = group_bulk_write_->txPacket();
  group_bulk_write_->clearParam();

  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(stderr, "%s\n", packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::BULK_WRITE_FAIL;
  } else {
    return DxlError::OK;
  }
}

void Dynamixel::ResetIndirectWrite(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.indirect_data_addr = 0;
  temp.size = 0;
  temp.cnt = 0;
  temp.item_name.clear();
  temp.item_size.clear();
  for (auto it_id : id_arr) {
    indirect_info_write_[it_id] = temp;
  }
}

DxlError Dynamixel::AddIndirectWrite(
  uint8_t id,
  std::string item_name,
  uint16_t item_addr,
  uint8_t item_size)
{
  // write address to indirect addr control item
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  dxl_info_.GetDxlControlItem(id, "Indirect Address Write", INDIRECT_ADDR, INDIRECT_SIZE);

  uint8_t using_size = indirect_info_write_[id].size;

  for (uint16_t i = 0; i < item_size; i++) {
    if (WriteItem(id, INDIRECT_ADDR + (using_size * 2), 2, item_addr + i) != DxlError::OK) {
      return DxlError::SET_BULK_WRITE_FAIL;
    }
    using_size++;
  }
  indirect_info_write_[id].size = using_size;
  indirect_info_write_[id].cnt += 1;
  indirect_info_write_[id].item_name.push_back(item_name);
  indirect_info_write_[id].item_size.push_back(item_size);

  return DxlError::OK;
}

void Dynamixel::ResetDirectWrite(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.indirect_data_addr = 0;
  temp.size = 0;
  temp.cnt = 0;
  temp.item_name.clear();
  temp.item_size.clear();
  for (auto it_id : id_arr) {
    direct_info_write_[it_id] = temp;
  }
}
}  // namespace dynamixel_hardware_interface
