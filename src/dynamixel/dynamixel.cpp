/*******************************************************************************
* Copyright 2024 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
/* Authors: Hye-Jong KIM, Sungho Woo, Woojin Wie */

#include "dynamixel_hardware_interface/dynamixel/dynamixel.hpp"

#include <queue>
#include <vector>
#include <string>
#include <memory>

namespace dynamixel_hardware_interface
{

Dynamixel::Dynamixel(const char * path)
{
  dxl_info_.SetDxlModelFolderPath(path);
  dxl_info_.InitDxlModelInfo();

  write_item_buf_.clear();
  read_item_buf_.clear();
}

Dynamixel::~Dynamixel()
{
  port_handler_->closePort();
  fprintf(stderr, "closed port\n");
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
      ReadItem(it_id, "Hardware Error Status", err);
      fprintf(stderr, "Read Hardware Error Status : %x\n", err);
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    } else {
      fprintf(stderr, " - Ping succeeded. Dynamixel model number : %d\n", dxl_model_number);
    }

    dxl_info_.ReadDxlModelFile(it_id, dxl_model_number);
  }

  read_data_list_.clear();
  write_data_list_.clear();

  for (auto it_id : id_arr) {
    if (dxl_info_.CheckDxlControlItem(it_id, "Torque Enable")) {
      torque_state_[it_id] = TORQUE_OFF;
    }
  }

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
  std::vector<std::string> item_names,
  std::vector<std::shared_ptr<double>> data_vec_ptr)
{
  if (item_names.size() == 0) {
    fprintf(stderr, "[ID:%03d] No (Sync or Bulk) Read Item\n", id);
    return DxlError::OK;
  }

  RWItemList read_item;
  read_item.id = id;

  for (auto it_name : item_names) {
    uint16_t ITEM_ADDR;
    uint8_t ITEM_SIZE;
    if (dxl_info_.GetDxlControlItem(id, it_name, ITEM_ADDR, ITEM_SIZE) == false) {
      fprintf(
        stderr, "[ID:%03d] Cannot find control item in model file. : %s\n", id,
        it_name.c_str());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }

    read_item.item_name.push_back(it_name);
    read_item.item_addr.push_back(ITEM_ADDR);
    read_item.item_size.push_back(ITEM_SIZE);
  }
  if (item_names.size() != data_vec_ptr.size()) {
    fprintf(
      stderr, "Incorrect Read Data Size!!![%zu] [%zu]\n",
      item_names.size(), data_vec_ptr.size());
    return DxlError::SET_READ_ITEM_FAIL;
  }

  read_item.item_data_ptr_vec = data_vec_ptr;

  read_data_list_.push_back(read_item);

  return DxlError::OK;
}

DxlError Dynamixel::SetMultiDxlRead()
{
  if (read_data_list_.size() < 2) {
    read_type_ = SYNC;
  } else {
    read_type_ = checkReadType();
  }

  fprintf(stderr, "Dynamixel Read Type : %s\n", read_type_ ? "bulk read" : "sync read");
  if (read_type_ == SYNC) {
    fprintf(stderr, "ID : ");
    for (auto it_read_data_list : read_data_list_) {
      fprintf(stderr, "%d, ", it_read_data_list.id);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Read items : ");
    for (auto it_read_data_list_item_name : read_data_list_.at(0).item_name) {
      fprintf(stderr, "\t%s", it_read_data_list_item_name.c_str());
    }
    fprintf(stderr, "\n");
  } else {
    for (auto it_read_data_list : read_data_list_) {
      fprintf(stderr, "ID : %d", it_read_data_list.id);
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
  std::vector<std::string> item_names,
  std::vector<std::shared_ptr<double>> data_vec_ptr)
{
  if (item_names.size() == 0) {
    fprintf(stderr, "[ID:%03d] No (Sync or Bulk) Write Item\n", id);
    return DxlError::OK;
  }

  RWItemList write_item;
  write_item.id = id;

  for (auto it_name : item_names) {
    uint16_t ITEM_ADDR;
    uint8_t ITEM_SIZE;
    if (dxl_info_.GetDxlControlItem(id, it_name, ITEM_ADDR, ITEM_SIZE) == false) {
      fprintf(
        stderr, "[ID:%03d] Cannot find control item in model file. : .%s\n", id,
        it_name.c_str());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }

    write_item.item_name.push_back(it_name);
    write_item.item_addr.push_back(ITEM_ADDR);
    write_item.item_size.push_back(ITEM_SIZE);
  }
  if (item_names.size() != data_vec_ptr.size()) {
    fprintf(stderr, "Incorrect Write Data Size!!!");
    return DxlError::SET_WRITE_ITEM_FAIL;
  }
  write_item.item_data_ptr_vec = data_vec_ptr;

  write_data_list_.push_back(write_item);

  return DxlError::OK;
}
DxlError Dynamixel::SetMultiDxlWrite()
{
  if (write_data_list_.size() < 2) {
    write_type_ = SYNC;
  } else {
    write_type_ = checkWriteType();
  }

  fprintf(stderr, "Dynamixel Write Type : %s\n", write_type_ ? "bulk write" : "sync write");
  if (write_type_ == SYNC) {
    fprintf(stderr, "ID : ");
    for (auto it_id : write_data_list_) {
      fprintf(stderr, "%d, ", it_id.id);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Write items : ");

    for (auto it_name : write_data_list_.at(0).item_name) {
      fprintf(stderr, "\t%s", it_name.c_str());
    }
    fprintf(stderr, "\n");
  } else {
    for (auto it_id : write_data_list_) {
      fprintf(stderr, "ID : %d", it_id.id);
      fprintf(stderr, "\tWrite items : ");
      for (auto it_name : it_id.item_name) {
        fprintf(stderr, "\t%s", it_name.c_str());
      }
      fprintf(stderr, "\n");
    }
  }

  if (write_type_ == SYNC) {
    SetSyncWriteItemAndHandler();
  } else {
    SetBulkWriteItemAndHandler();
  }

  return DxlError::OK;
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
  for (auto it_id : id_arr) {
    if (torque_state_[it_id] == TORQUE_ON) {
      if (WriteItem(it_id, "Torque Enable", TORQUE_OFF) < 0) {
        fprintf(stderr, "[ID:%03d] Cannot write \"Torque Off\" command!\n", it_id);
        return DxlError::ITEM_WRITE_FAIL;
      } else {
        torque_state_[it_id] = TORQUE_OFF;
        fprintf(stderr, "[ID:%03d] Torque OFF\n", it_id);
      }
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetOperatingMode(uint8_t dxl_id, uint8_t dynamixel_mode)
{
  if (WriteItem(dxl_id, "Operating Mode", dynamixel_mode) == false) {
    return DxlError::ITEM_WRITE_FAIL;
  }

  fprintf(stderr, "[ID:%03d] Succeeded to set operating mode(", dxl_id);
  if (dynamixel_mode == DXL_POSITION_CTRL_MODE) {
    fprintf(stderr, "Position Control Mode)\n");
  } else if (dynamixel_mode == DXL_CURRENT_CTRL_MODE) {
    fprintf(stderr, "Current Control Mode)\n");
  } else if (dynamixel_mode == DXL_VELOCITY_CTRL_MODE) {
    fprintf(stderr, "Velocity Control Mode)\n");
  } else {
    fprintf(stderr, "Not Defined Control Mode)\n");
  }

  return DxlError::OK;
}

DxlError Dynamixel::WriteItem(uint8_t id, std::string item_name, uint32_t data)
{
  uint16_t ITEM_ADDR;
  uint8_t ITEM_SIZE;
  if (dxl_info_.GetDxlControlItem(id, item_name, ITEM_ADDR, ITEM_SIZE) == false) {
    fprintf(
      stderr, "[ID:%03d] Cannot find control item in model file. : %s\n", id,
      item_name.c_str());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  return WriteItem(id, ITEM_ADDR, ITEM_SIZE, data);
}

DxlError Dynamixel::WriteItem(uint8_t id, uint16_t addr, uint8_t size, uint32_t data)
{
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;


  if (size == 1) {
    dxl_comm_result =
      packet_handler_->write1ByteTxRx(
      port_handler_, id, addr, static_cast<uint8_t>(data),
      &dxl_error);
  } else if (size == 2) {
    dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id, addr,
      static_cast<uint16_t>(data), &dxl_error);
  } else if (size == 4) {
    dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_, id, addr,
      static_cast<uint32_t>(data), &dxl_error);
  }

  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(
      stderr,
      "[ID:%03d] COMM_ERROR : %s\n",
      id,
      packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::ITEM_WRITE_FAIL;
  } else if (dxl_error != 0) {
    fprintf(
      stderr,
      "[ID:%03d] RX_PACKET_ERROR : %s\n",
      id,
      packet_handler_->getRxPacketError(dxl_error));
    return DxlError::ITEM_WRITE_FAIL;
  }
  return DxlError::OK;
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
      stderr, "[ID:%03d] Cannot find control item in model file. : %s\n", id,
      item_name.c_str());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  if (ITEM_SIZE == 1) {
    uint8_t read_data;
    dxl_comm_result = packet_handler_->read1ByteTxRx(
      port_handler_, id, ITEM_ADDR,
      &read_data, &dxl_error);
    data = read_data;
  } else if (ITEM_SIZE == 2) {
    uint16_t read_data;
    dxl_comm_result = packet_handler_->read2ByteTxRx(
      port_handler_, id, ITEM_ADDR,
      &read_data, &dxl_error);
    data = read_data;
  } else if (ITEM_SIZE == 4) {
    uint32_t read_data;
    dxl_comm_result = packet_handler_->read4ByteTxRx(
      port_handler_, id, ITEM_ADDR,
      &read_data, &dxl_error);
    data = read_data;
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
  }
  return DxlError::OK;
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
  }
}

DxlError Dynamixel::ReadMultiDxlData()
{
  if (read_type_ == SYNC) {
    return GetDxlValueFromSyncRead();
  } else {
    return GetDxlValueFromBulkRead();
  }
}

DxlError Dynamixel::WriteMultiDxlData()
{
  if (write_type_ == SYNC) {
    return SetDxlValueToSyncWrite();
  } else {
    return SetDxlValueToBulkWrite();
  }
}

bool Dynamixel::checkReadType()
{
  for (size_t dxl_index = 1; dxl_index < read_data_list_.size(); dxl_index++) {
    if (read_data_list_.at(dxl_index).item_name.size() !=
      read_data_list_.at(dxl_index - 1).item_name.size())
    {
      return BULK;
    }
    for (size_t item_index = 0; item_index < read_data_list_.at(dxl_index).item_name.size();
      item_index++)
    {
      if (read_data_list_.at(dxl_index).item_name.at(item_index) !=
        read_data_list_.at(dxl_index - 1).item_name.at(item_index))
      {
        return BULK;
      }
    }
  }
  return SYNC;
}

bool Dynamixel::checkWriteType()
{
  for (size_t dxl_index = 1; dxl_index < write_data_list_.size(); dxl_index++) {
    if (write_data_list_.at(dxl_index).item_name.size() !=
      write_data_list_.at(dxl_index - 1).item_name.size())
    {
      return BULK;
    }
    for (size_t item_index = 0; item_index < write_data_list_.at(dxl_index).item_name.size();
      item_index++)
    {
      if (write_data_list_.at(dxl_index).item_name.at(item_index) !=
        write_data_list_.at(dxl_index - 1).item_name.at(item_index))
      {
        return BULK;
      }
    }
  }
  return SYNC;
}

DxlError Dynamixel::SetSyncReadItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_read_data : read_data_list_) {
    id_arr.push_back(it_read_data.id);
  }

  DynamixelDisable(id_arr);
  ResetIndirectRead(id_arr);

  for (auto it_read_data : read_data_list_) {
    for (size_t item_index = 0; item_index < it_read_data.item_name.size(); item_index++) {
      auto result = AddIndirectRead(
        it_read_data.id,
        it_read_data.item_name.at(item_index),
        it_read_data.item_addr.at(item_index),
        it_read_data.item_size.at(item_index));

      if (result == DxlError::OK) {
      } else {
        fprintf(
          stderr, "[ID:%03d] Failed to Indirect Address Read Item : [%s], %d\n",
          it_read_data.id,
          it_read_data.item_name.at(item_index).c_str(),
          result);
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


DxlError Dynamixel::SetSyncReadHandler(std::vector<uint8_t> id_arr)
{
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

DxlError Dynamixel::GetDxlValueFromSyncRead()
{
  // SyncRead tx
  int dxl_comm_result = group_sync_read_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(stderr, "SyncRead TxRx Fail [Error code : %d]\n", dxl_comm_result);
    return DxlError::SYNC_READ_FAIL;
  }

  for (auto it_read_data : read_data_list_) {
    uint8_t ID = it_read_data.id;
    uint16_t IN_ADDR = indirect_info_read_[ID].indirect_data_addr;

    for (size_t item_index = 0; item_index < indirect_info_read_[ID].cnt; item_index++) {
      uint8_t SIZE = indirect_info_read_[ID].item_size.at(item_index);
      if (item_index > 0) {IN_ADDR += indirect_info_read_[ID].item_size.at(item_index - 1);}

      uint32_t dxl_getdata = group_sync_read_->getData(ID, IN_ADDR, SIZE);

      if (indirect_info_read_[ID].item_name.at(item_index) == "Present Position") {
        *it_read_data.item_data_ptr_vec.at(item_index) =
          dxl_info_.ConvertValueToRadian(ID, static_cast<int32_t>(dxl_getdata));
      } else if (indirect_info_read_[ID].item_name.at(item_index) == "Present Velocity") {
        *it_read_data.item_data_ptr_vec.at(item_index) =
          dxl_info_.ConvertValueRPMToVelocityRPS(ID, static_cast<int32_t>(dxl_getdata));
      } else if (indirect_info_read_[ID].item_name.at(item_index) == "Present Current") {
        *it_read_data.item_data_ptr_vec.at(item_index) =
         dxl_info_.ConvertCurrentToEffort(ID, static_cast<int16_t>(dxl_getdata));
      } else {
        *it_read_data.item_data_ptr_vec.at(item_index) =
          static_cast<double>(dxl_getdata);
      }
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkReadItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_read_data : read_data_list_) {
    id_arr.push_back(it_read_data.id);
  }

  DynamixelDisable(id_arr);
  ResetIndirectRead(id_arr);

  for (auto it_read_data : read_data_list_) {
    for (size_t item_index = 0; item_index < it_read_data.item_name.size();
      item_index++)
    {
      auto result = AddIndirectRead(
        it_read_data.id,
        it_read_data.item_name.at(item_index),
        it_read_data.item_addr.at(item_index),
        it_read_data.item_size.at(item_index));

      if (result == DxlError::OK) {
        fprintf(
          stderr, "[ID:%03d] Add Indirect Address Read Item : [%s]\n",
          it_read_data.id,
          it_read_data.item_name.at(item_index).c_str());
      } else {
        fprintf(
          stderr, "[ID:%03d] Failed to Indirect Address Read Item : [%s], %d\n",
          it_read_data.id,
          it_read_data.item_name.at(item_index).c_str(),
          result);
      }
    }
  }

  if (SetBulkReadHandler(id_arr) != DxlError::OK) {
    fprintf(stderr, "Cannot set the BulkRead handler.\n");
    return DxlError::SYNC_READ_FAIL;
  }

  fprintf(stderr, "Success to set BulkRead handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkReadHandler(std::vector<uint8_t> id_arr)
{
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
  }
  fprintf(
    stderr,
    "set bulk read (indirect addr) : addr %d, size %d\n",
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  group_bulk_read_ = new dynamixel::GroupBulkRead(port_handler_, packet_handler_);

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

DxlError Dynamixel::GetDxlValueFromBulkRead()
{
  int dxl_comm_result = group_bulk_read_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(stderr, "BulkRead TxRx Fail [Error code : %d]\n", dxl_comm_result);
    return DxlError::BULK_READ_FAIL;
  }

  for (auto it_read_data : read_data_list_) {
    uint8_t ID = it_read_data.id;
    uint16_t IN_ADDR = indirect_info_read_[ID].indirect_data_addr;

    for (size_t item_index = 0; item_index < indirect_info_read_[ID].cnt; item_index++) {
      uint8_t SIZE = indirect_info_read_[ID].item_size.at(item_index);
      if (item_index > 0) {IN_ADDR += indirect_info_read_[ID].item_size.at(item_index - 1);}

      uint32_t dxl_getdata = group_bulk_read_->getData(ID, IN_ADDR, SIZE);

      if (indirect_info_read_[ID].item_name.at(item_index) == "Present Position") {
        *it_read_data.item_data_ptr_vec.at(item_index) =
          dxl_info_.ConvertValueToRadian(ID, static_cast<int32_t>(dxl_getdata));
      } else if (indirect_info_read_[ID].item_name.at(item_index) == "Present Velocity") {
        *it_read_data.item_data_ptr_vec.at(item_index) =
          dxl_info_.ConvertValueRPMToVelocityRPS(ID, static_cast<int32_t>(dxl_getdata));
      } else if (indirect_info_read_[ID].item_name.at(item_index) == "Present Current") {
        *it_read_data.item_data_ptr_vec.at(item_index) =
         dxl_info_.ConvertCurrentToEffort(ID, static_cast<int16_t>(dxl_getdata));
      } else {
        *it_read_data.item_data_ptr_vec.at(item_index) =
          static_cast<double>(dxl_getdata);
      }
    }
  }
  return DxlError::OK;
}

void Dynamixel::ResetIndirectRead(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.cnt = temp.size = 0;
  temp.item_name.clear();
  temp.item_size.clear();
  for (auto it_id : id_arr) {
    indirect_info_read_[it_id] = temp;
  }
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
      if (WriteItem(id, INDIRECT_ADDR + (using_size * 2), 2, item_addr + i) != DxlError::OK) {
        return DxlError::SET_BULK_READ_FAIL;
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
    id_arr.push_back(it_write_data.id);
  }

  DynamixelDisable(id_arr);
  ResetIndirectWrite(id_arr);

  for (auto it_write_data : write_data_list_) {
    for (size_t item_index = 0; item_index < it_write_data.item_name.size();
      item_index++)
    {
      if (AddIndirectWrite(
          it_write_data.id,
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
    uint8_t ID = it_write_data.id;
    uint8_t * param_write_value = new uint8_t[indirect_info_write_[ID].size];
    uint8_t added_byte = 0;


    for (uint16_t item_index = 0; item_index < indirect_info_write_[ID].cnt; item_index++) {
      double data = *it_write_data.item_data_ptr_vec.at(item_index);
      if (indirect_info_write_[ID].item_name.at(item_index) == "Goal Position") {
        int32_t goal_position = dxl_info_.ConvertRadianToValue(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
        param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
        param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
        param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_position));
      } else if (indirect_info_write_[ID].item_name.at(item_index) == "Goal Current") {
        int16_t goal_current = dxl_info_.ConvertEffortToCurrent(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(goal_current);
        param_write_value[added_byte + 1] = DXL_HIBYTE(goal_current);
      } else if (indirect_info_write_[ID].item_name.at(item_index) == "Goal Velocity") {
        int16_t goal_velocity = dxl_info_.ConvertVelocityRPSToValueRPM(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity));
        param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity));
        param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity));
        param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity));
      }
      added_byte += indirect_info_write_[ID].item_size.at(item_index);
    }

    if (group_sync_write_->addParam(ID, param_write_value) != true) {
      printf("[ID:%03d] groupSyncWrite addparam failed\n", ID);
      return DxlError::SYNC_WRITE_FAIL;
    }
  }

  int dxl_comm_result = group_sync_write_->txPacket();
  group_sync_write_->clearParam();

  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::SYNC_WRITE_FAIL;
  } else {
    return DxlError::OK;
  }
}

DxlError Dynamixel::SetBulkWriteItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_write_data : write_data_list_) {
    id_arr.push_back(it_write_data.id);
  }

  DynamixelDisable(id_arr);
  ResetIndirectWrite(id_arr);

  for (auto it_write_data : write_data_list_) {
    for (size_t item_index = 0; item_index < it_write_data.item_name.size();
      item_index++)
    {
      if (AddIndirectWrite(
          it_write_data.id,
          it_write_data.item_name.at(item_index),
          it_write_data.item_addr.at(item_index),
          it_write_data.item_size.at(item_index)) != DxlError::OK)
      {
        fprintf(stderr, "Cannot set the BulkWrite handler.\n");
        return DxlError::BULK_WRITE_FAIL;
      }

      fprintf(
        stderr, "[ID:%03d] Add Indirect Address Write Item : [%s]\n",
        it_write_data.id,
        it_write_data.item_name.at(item_index).c_str());
    }
  }

  if (SetBulkWriteHandler(id_arr) < 0) {
    fprintf(stderr, "Cannot set the BulkWrite handler.\n");
    return DxlError::BULK_WRITE_FAIL;
  }

  fprintf(stderr, "Success to set BulkWrite handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkWriteHandler(std::vector<uint8_t> id_arr)
{
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
        "Fail to set indirect address bulk write. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_BULK_WRITE_FAIL;
    }
    // Set indirect addr.
    indirect_info_write_[it_id].indirect_data_addr = INDIRECT_ADDR;
  }
  fprintf(
    stderr,
    "set bulk write (indirect addr) : addr %d, size %d\n",
    INDIRECT_ADDR, indirect_info_write_[id_arr.at(0)].size);

  group_bulk_write_ = new dynamixel::GroupBulkWrite(port_handler_, packet_handler_);
  return DxlError::OK;
}

DxlError Dynamixel::SetDxlValueToBulkWrite()
{
  for (auto it_write_data : write_data_list_) {
    uint8_t ID = it_write_data.id;
    uint8_t * param_write_value = new uint8_t[indirect_info_write_[ID].size];
    uint8_t added_byte = 0;


    for (uint16_t item_index = 0; item_index < indirect_info_write_[ID].cnt; item_index++) {
      double data = *it_write_data.item_data_ptr_vec.at(item_index);
      if (indirect_info_write_[ID].item_name.at(item_index) == "Goal Position") {
        int32_t goal_position = dxl_info_.ConvertRadianToValue(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
        param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
        param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
        param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_position));
      } else if (indirect_info_write_[ID].item_name.at(item_index) == "Goal Current") {
        int16_t goal_current = dxl_info_.ConvertEffortToCurrent(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(goal_current);
        param_write_value[added_byte + 1] = DXL_HIBYTE(goal_current);
      } else if (indirect_info_write_[ID].item_name.at(item_index) == "Goal Velocity") {
        int32_t goal_velocity = dxl_info_.ConvertVelocityRPSToValueRPM(ID, data);
        param_write_value[added_byte + 0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity));
        param_write_value[added_byte + 1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity));
        param_write_value[added_byte + 2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity));
        param_write_value[added_byte + 3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity));
      }
      added_byte += indirect_info_write_[ID].item_size.at(item_index);
    }

    if (group_bulk_write_->addParam(
        ID,
        indirect_info_write_[ID].indirect_data_addr,
        indirect_info_write_[ID].size,
        param_write_value) != true)
    {
      printf("[ID:%03d] groupBulkWrite addparam failed\n", ID);
      return DxlError::BULK_WRITE_FAIL;
    }
  }

  int dxl_comm_result = group_bulk_write_->txPacket();
  group_bulk_write_->clearParam();

  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::BULK_WRITE_FAIL;
  } else {
    return DxlError::OK;
  }
}

void Dynamixel::ResetIndirectWrite(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.cnt = temp.size = 0;
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
}  // namespace dynamixel_hardware_interface
