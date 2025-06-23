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

#include "dynamixel_hardware_interface/dynamixel_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware_interface
{

DynamixelHardware::DynamixelHardware()
: rclcpp::Node("dynamixel_hardware_interface"),
  logger_(rclcpp::get_logger("dynamixel_hardware_interface"))
{
  dxl_status_ = DXL_OK;
  dxl_torque_status_ = TORQUE_ENABLED;
  err_timeout_ms_ = 500;
  is_read_in_error_ = false;
  is_write_in_error_ = false;
  read_error_duration_ = rclcpp::Duration(0, 0);
  write_error_duration_ = rclcpp::Duration(0, 0);
}

DynamixelHardware::~DynamixelHardware()
{
  stop();

  if (rclcpp::ok()) {
    RCLCPP_INFO(logger_, "Shutting down ROS2 node...");
  }
}

hardware_interface::CallbackReturn DynamixelHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to initialize DynamixelHardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  num_of_joints_ = static_cast<size_t>(stoi(info_.hardware_parameters["number_of_joints"]));
  num_of_transmissions_ =
    static_cast<size_t>(stoi(info_.hardware_parameters["number_of_transmissions"]));
  SetMatrix();

  port_name_ = info_.hardware_parameters["port_name"];
  baud_rate_ = info_.hardware_parameters["baud_rate"];
  try {
    err_timeout_ms_ = stod(info_.hardware_parameters["error_timeout_ms"]);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Failed to parse error_timeout_ms parameter: %s, using default value",
      e.what());
  }

  // Add new parameter for torque initialization
  bool disable_torque_at_init = false;
  if (info_.hardware_parameters.find("disable_torque_at_init") != info_.hardware_parameters.end()) {
    disable_torque_at_init = info_.hardware_parameters.at("disable_torque_at_init") == "true";
    if (disable_torque_at_init) {
      RCLCPP_INFO(
        logger_,
        "Torque will be disabled during initialization if it is enabled at initialization.");
    }
  } else {
    RCLCPP_INFO(
      logger_,
      "If there is a torque enabled Dynamixel, the program will be terminated. Set "
      "'disable_torque_at_init' parameter to 'true' to disable torque at initialization.");
  }

  RCLCPP_INFO_STREAM(
    logger_,
    "port_name " << port_name_.c_str() << " / baudrate " << baud_rate_.c_str());

  std::string dxl_model_folder = info_.hardware_parameters["dynamixel_model_folder"];
  dxl_comm_ = std::unique_ptr<Dynamixel>(
    new Dynamixel(
      (ament_index_cpp::get_package_share_directory("dynamixel_hardware_interface") +
      dxl_model_folder).c_str()));

  RCLCPP_INFO_STREAM(logger_, "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Comm Port");

  if (info_.hardware_parameters.find("use_revolute_to_prismatic_gripper") !=
    info_.hardware_parameters.end())
  {
    use_revolute_to_prismatic_ =
      std::stoi(info_.hardware_parameters.at("use_revolute_to_prismatic_gripper")) != 0;
  }

  if (use_revolute_to_prismatic_) {
    RCLCPP_INFO(logger_, "Revolute to Prismatic gripper conversion enabled.");
    initRevoluteToPrismaticParam();
  }
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    if (gpio.parameters.find("ID") == gpio.parameters.end()) {
      RCLCPP_ERROR_STREAM(logger_, "ID is not found in gpio parameters");
      exit(-1);
    }
    if (gpio.parameters.find("type") == gpio.parameters.end()) {
      RCLCPP_ERROR_STREAM(logger_, "type is not found in gpio parameters");
      exit(-1);
    }

    uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
    const std::string & type = gpio.parameters.at("type");

    if (type == "dxl") {
      dxl_id_.push_back(id);
    } else if (type == "sensor") {
      sensor_id_.push_back(id);
    } else if (type == "controller") {
      controller_id_.push_back(id);
    } else if (type == "virtual_dxl") {
      dxl_comm_->ReadDxlModelFile(id, static_cast<uint16_t>(stoi(gpio.parameters.at("model_num"))));
      virtual_dxl_id_.push_back(id);
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Invalid DXL / Sensor type");
      exit(-1);
    }

    uint8_t comm_id = (gpio.parameters.find("comm_id") != gpio.parameters.end()) ?
      static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id"))) : id;
    dxl_comm_->SetCommId(id, comm_id);
  }

  if (controller_id_.size() > 0) {
    for (int i = 0; i < 10; i++) {
      std::vector<uint8_t> id_arr;
      for (auto controller : controller_id_) {
        id_arr.push_back(controller);
      }
      if (dxl_comm_->InitDxlComm(id_arr, port_name_, baud_rate_) == DxlError::OK) {
        RCLCPP_INFO_STREAM(logger_, "Trying to connect to the communication port...");
        break;
      } else {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (i == 9) {
          RCLCPP_ERROR_STREAM(logger_, "Cannot connect communication port! :(");
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }
  }

  if (!InitControllerItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitControllerItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < 10; i++) {
    std::vector<uint8_t> id_arr;
    for (auto dxl : dxl_id_) {
      id_arr.push_back(dxl);
    }
    for (auto sensor : sensor_id_) {
      id_arr.push_back(sensor);
    }
    if (dxl_comm_->InitDxlComm(id_arr, port_name_, baud_rate_) == DxlError::OK) {
      RCLCPP_INFO_STREAM(logger_, "Trying to connect to the communication port...");
      break;
    } else {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (i == 9) {
        RCLCPP_ERROR_STREAM(logger_, "Cannot connect communication port! :(");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  std::vector<uint8_t> dxl_id_with_virtual_dxl;
  dxl_id_with_virtual_dxl.insert(dxl_id_with_virtual_dxl.end(), dxl_id_.begin(), dxl_id_.end());
  dxl_id_with_virtual_dxl.insert(
    dxl_id_with_virtual_dxl.end(), virtual_dxl_id_.begin(),
    virtual_dxl_id_.end());
  if (dxl_comm_->InitTorqueStates(
      dxl_id_with_virtual_dxl,
      disable_torque_at_init) != DxlError::OK)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitTorqueStates");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!InitDxlItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitDxlItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!InitDxlReadItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitDxlReadItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!InitDxlWriteItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitDxlWriteItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (num_of_transmissions_ != hdl_trans_commands_.size() &&
    num_of_transmissions_ != hdl_trans_states_.size())
  {
    RCLCPP_ERROR_STREAM(
      logger_, "Error: number of transmission " << num_of_transmissions_ << ", " <<
        hdl_trans_commands_.size() << ", " << hdl_trans_states_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  dxl_status_ = DXL_OK;

  hdl_joint_states_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    HandlerVarType temp_state;
    temp_state.name = joint.name;

    for (auto it : joint.state_interfaces) {
      if (hardware_interface::HW_IF_POSITION != it.name &&
        hardware_interface::HW_IF_VELOCITY != it.name &&
        hardware_interface::HW_IF_ACCELERATION != it.name &&
        hardware_interface::HW_IF_EFFORT != it.name &&
        HW_IF_HARDWARE_STATE != it.name &&
        HW_IF_TORQUE_ENABLE != it.name)
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Error: invalid joint state interface " << it.name);
        return hardware_interface::CallbackReturn::ERROR;
      }
      temp_state.interface_name_vec.push_back(it.name);
      temp_state.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_joint_states_.push_back(temp_state);
  }

  hdl_joint_commands_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    HandlerVarType temp_cmd;
    temp_cmd.name = joint.name;

    for (auto it : joint.command_interfaces) {
      if (hardware_interface::HW_IF_POSITION != it.name &&
        hardware_interface::HW_IF_VELOCITY != it.name &&
        hardware_interface::HW_IF_ACCELERATION != it.name &&
        hardware_interface::HW_IF_EFFORT != it.name)
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Error: invalid joint command interface " << it.name);
        return hardware_interface::CallbackReturn::ERROR;
      }
      temp_cmd.interface_name_vec.push_back(it.name);
      temp_cmd.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_joint_commands_.push_back(temp_cmd);
  }

  if (num_of_joints_ != hdl_joint_commands_.size() &&
    num_of_joints_ != hdl_joint_states_.size())
  {
    RCLCPP_ERROR_STREAM(
      logger_, "Error: number of joints " << num_of_joints_ << ", " <<
        hdl_joint_commands_.size() << ", " << hdl_joint_commands_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hdl_sensor_states_.clear();
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    HandlerVarType temp_state;
    temp_state.name = sensor.name;

    for (auto it : sensor.state_interfaces) {
      temp_state.interface_name_vec.push_back(it.name);
      temp_state.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_sensor_states_.push_back(temp_state);
  }

  std::string str_dxl_state_pub_name =
    info_.hardware_parameters["dynamixel_state_pub_msg_name"];
  dxl_state_pub_ = this->create_publisher<DynamixelStateMsg>(
    str_dxl_state_pub_name, rclcpp::SystemDefaultsQoS());
  dxl_state_pub_uni_ptr_ = std::make_unique<StatePublisher>(dxl_state_pub_);

  size_t num_of_pub_data = hdl_trans_states_.size();
  dxl_state_pub_uni_ptr_->lock();
  dxl_state_pub_uni_ptr_->msg_.id.resize(num_of_pub_data);
  dxl_state_pub_uni_ptr_->msg_.dxl_hw_state.resize(num_of_pub_data);
  dxl_state_pub_uni_ptr_->msg_.torque_state.resize(num_of_pub_data);
  dxl_state_pub_uni_ptr_->unlock();

  using namespace std::placeholders;
  std::string str_get_dxl_data_srv_name =
    info_.hardware_parameters["get_dynamixel_data_srv_name"];
  get_dxl_data_srv_ = create_service<dynamixel_interfaces::srv::GetDataFromDxl>(
    str_get_dxl_data_srv_name,
    std::bind(&DynamixelHardware::get_dxl_data_srv_callback, this, _1, _2));

  std::string str_set_dxl_data_srv_name =
    info_.hardware_parameters["set_dynamixel_data_srv_name"];
  set_dxl_data_srv_ = create_service<dynamixel_interfaces::srv::SetDataToDxl>(
    str_set_dxl_data_srv_name,
    std::bind(&DynamixelHardware::set_dxl_data_srv_callback, this, _1, _2));

  std::string str_reboot_dxl_srv_name =
    info_.hardware_parameters["reboot_dxl_srv_name"];
  reboot_dxl_srv_ = create_service<dynamixel_interfaces::srv::RebootDxl>(
    str_reboot_dxl_srv_name,
    std::bind(&DynamixelHardware::reboot_dxl_srv_callback, this, _1, _2));

  std::string str_set_dxl_torque_srv_name =
    info_.hardware_parameters["set_dxl_torque_srv_name"];
  set_dxl_torque_srv_ = create_service<std_srvs::srv::SetBool>(
    str_set_dxl_torque_srv_name,
    std::bind(&DynamixelHardware::set_dxl_torque_srv_callback, this, _1, _2));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DynamixelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  try {
    for (auto it : hdl_trans_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_,
            "Interface name vector size mismatch for " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_joint_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for joint " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_sensor_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for sensor " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_gpio_controller_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for gpio controller " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error in export_state_interfaces: " << e.what());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  try {
    for (auto it : hdl_trans_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_joint_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for joint " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_gpio_controller_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for gpio controller " <<
              it.name << ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error in export_command_interfaces: " << e.what());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DynamixelHardware::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return start();
}

hardware_interface::CallbackReturn DynamixelHardware::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return stop();
}

hardware_interface::CallbackReturn DynamixelHardware::start()
{
  rclcpp::Time start_time = this->now();
  rclcpp::Duration error_duration(0, 0);
  while (true) {
    dxl_comm_err_ = CheckError(dxl_comm_->ReadMultiDxlData(0.0));
    if (dxl_comm_err_ == DxlError::OK) {
      break;
    }
    error_duration = this->now() - start_time;
    if (error_duration.seconds() * 1000 >= err_timeout_ms_) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Dynamixel Start Fail (Timeout: " << error_duration.seconds() * 1000 << "ms/" <<
          err_timeout_ms_ << "ms): " << Dynamixel::DxlErrorToString(dxl_comm_err_));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  CalcTransmissionToJoint();

  SyncJointCommandWithStates();

  CalcJointToTransmission();

  dxl_comm_->WriteMultiDxlData();

  // Enable torque only for Dynamixels that have torque enabled in their parameters
  std::vector<uint8_t> torque_enabled_ids;
  for (const auto & [id, enabled] : dxl_torque_enable_) {
    if (enabled) {
      torque_enabled_ids.push_back(id);
    }
  }

  if (torque_enabled_ids.size() > 0) {
    RCLCPP_INFO_STREAM(logger_, "Enabling torque for Dynamixels");
    for (int i = 0; i < 10; i++) {
      if (dxl_comm_->DynamixelEnable(torque_enabled_ids) == DxlError::OK) {
        break;
      }
      RCLCPP_ERROR_STREAM(logger_, "Failed to enable torque for Dynamixels, retry...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  RCLCPP_INFO_STREAM(logger_, "Dynamixel Hardware Start!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::stop()
{
  if (dxl_comm_) {
    dxl_comm_->DynamixelDisable(dxl_id_);
    dxl_comm_->DynamixelDisable(virtual_dxl_id_);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Dynamixel Hardware Stop Fail : dxl_comm_ is nullptr");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger_, "Dynamixel Hardware Stop!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardware::read(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double period_ms = period.seconds() * 1000;

  if (dxl_status_ == REBOOTING) {
    RCLCPP_ERROR_STREAM(logger_, "Dynamixel Read Fail : REBOOTING");
    return hardware_interface::return_type::ERROR;
  } else if (dxl_status_ == DXL_OK || dxl_status_ == COMM_ERROR) {
    dxl_comm_err_ = CheckError(dxl_comm_->ReadMultiDxlData(period_ms));
    if (dxl_comm_err_ != DxlError::OK) {
      if (!is_read_in_error_) {
        is_read_in_error_ = true;
        read_error_duration_ = rclcpp::Duration(0, 0);
      }
      read_error_duration_ = read_error_duration_ + period;

      RCLCPP_ERROR_STREAM(
        logger_,
        "Dynamixel Read Fail (Duration: " << read_error_duration_.seconds() * 1000 << "ms/" <<
          err_timeout_ms_ << "ms)");

      if (read_error_duration_.seconds() * 1000 >= err_timeout_ms_) {
        return hardware_interface::return_type::ERROR;
      }
      return hardware_interface::return_type::OK;
    }
    is_read_in_error_ = false;
    read_error_duration_ = rclcpp::Duration(0, 0);
  } else if (dxl_status_ == HW_ERROR) {
    dxl_comm_err_ = CheckError(dxl_comm_->ReadMultiDxlData(period_ms));
    if (dxl_comm_err_ != DxlError::OK) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Dynamixel Read Fail :" << Dynamixel::DxlErrorToString(dxl_comm_err_));
    }
  }

  CalcTransmissionToJoint();

  for (auto sensor : hdl_gpio_sensor_states_) {
    ReadSensorData(sensor);
  }

  dxl_comm_->ReadItemBuf();

  size_t index = 0;
  if (dxl_state_pub_uni_ptr_ && dxl_state_pub_uni_ptr_->trylock()) {
    dxl_state_pub_uni_ptr_->msg_.header.stamp = this->now();
    dxl_state_pub_uni_ptr_->msg_.comm_state = dxl_comm_err_;
    for (auto it : hdl_trans_states_) {
      dxl_state_pub_uni_ptr_->msg_.id.at(index) = it.id;
      dxl_state_pub_uni_ptr_->msg_.dxl_hw_state.at(index) = dxl_hw_err_[it.id];
      dxl_state_pub_uni_ptr_->msg_.torque_state.at(index) = dxl_torque_state_[it.id];
      index++;
    }
    dxl_state_pub_uni_ptr_->unlockAndPublish();
  }

  if (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type DynamixelHardware::write(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (dxl_status_ == DXL_OK || dxl_status_ == HW_ERROR) {
    dxl_comm_->WriteItemBuf();

    ChangeDxlTorqueState();

    CalcJointToTransmission();

    dxl_comm_->WriteMultiDxlData();

    is_write_in_error_ = false;
    write_error_duration_ = rclcpp::Duration(0, 0);

    return hardware_interface::return_type::OK;
  } else {
    write_error_duration_ = write_error_duration_ + period;

    RCLCPP_ERROR_STREAM(
      logger_,
      "Dynamixel Write Fail (Duration: " << write_error_duration_.seconds() * 1000 << "ms/" <<
        err_timeout_ms_ << "ms)");

    if (write_error_duration_.seconds() * 1000 >= err_timeout_ms_) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
}

DxlError DynamixelHardware::CheckError(DxlError dxl_comm_err)
{
  DxlError error_state = DxlError::OK;
  dxl_status_ = DXL_OK;

  // check comm error
  if (dxl_comm_err != DxlError::OK) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Communication Fail --> " << Dynamixel::DxlErrorToString(dxl_comm_err));
    dxl_status_ = COMM_ERROR;
    return dxl_comm_err;
  }
  // check hardware error
  for (size_t i = 0; i < num_of_transmissions_; i++) {
    for (size_t j = 0; j < hdl_trans_states_.at(i).interface_name_vec.size(); j++) {
      if (hdl_trans_states_.at(i).interface_name_vec.at(j) == "Hardware Error Status") {
        dxl_hw_err_[hdl_trans_states_.at(i).id] = *hdl_trans_states_.at(i).value_ptr_vec.at(j);
        std::string error_string = "";
        if (dxl_hw_err_[hdl_trans_states_.at(i).id] & 0x01) {
          error_string += "input voltage error/ ";
        }
        if (dxl_hw_err_[hdl_trans_states_.at(i).id] & 0x04) {
          error_string += "overheating/ ";
        }
        if (dxl_hw_err_[hdl_trans_states_.at(i).id] & 0x08) {
          error_string += "motor encoder/ ";
        }
        if (dxl_hw_err_[hdl_trans_states_.at(i).id] & 0x16) {
          error_string += "electrical shork/ ";
        }
        if (dxl_hw_err_[hdl_trans_states_.at(i).id] & 0x32) {
          error_string += "Overload/ ";
        }

        if (!error_string.empty()) {
          RCLCPP_WARN_STREAM(
            logger_, "Dynamixel Hardware Error States [ ID:" <<
              static_cast<int>(hdl_trans_states_.at(i).id) << "] --> " <<
              static_cast<int>(dxl_hw_err_[hdl_trans_states_.at(i).id]) <<
              "/ " << error_string);
          dxl_status_ = HW_ERROR;
          error_state = DxlError::DLX_HARDWARE_ERROR;
        }
      }
    }
  }

  for (size_t i = 0; i < num_of_joints_; i++) {
    for (size_t j = 0; j < hdl_joint_states_.at(i).interface_name_vec.size(); j++) {
      if (hdl_joint_states_.at(i).interface_name_vec.at(j) == HW_IF_HARDWARE_STATE) {
        *hdl_joint_states_.at(i).value_ptr_vec.at(j) = error_state;
      }
    }
  }

  return error_state;
}

bool DynamixelHardware::CommReset()
{
  dxl_status_ = REBOOTING;
  stop();
  RCLCPP_INFO_STREAM(logger_, "Communication Reset Start");
  dxl_comm_->RWDataReset();

  auto start_time = this->now();
  while ((this->now() - start_time) < rclcpp::Duration(3, 0)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO_STREAM(logger_, "Reset Start");
    bool result = true;
    for (auto id : dxl_id_) {
      if (dxl_comm_->Reboot(id) != DxlError::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Cannot reboot dynamixel! :(");
        result = false;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!result) {continue;}
    if (!InitControllerItems()) {continue;}
    if (!InitDxlItems()) {continue;}
    if (!InitDxlReadItems()) {continue;}
    if (!InitDxlWriteItems()) {continue;}

    RCLCPP_INFO_STREAM(logger_, "RESET Success");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    start();
    dxl_status_ = DXL_OK;
    return true;
  }
  RCLCPP_ERROR_STREAM(logger_, "RESET Failure");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  start();
  return false;
}

bool DynamixelHardware::initItems(const std::string & type_filter)
{
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Items for type: " << type_filter);
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    if (gpio.parameters.at("type") != type_filter) {
      continue;
    }
    uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));

    // Handle torque enable parameter
    bool torque_enabled = type_filter == "dxl" || type_filter == "virtual_dxl";
    if (gpio.parameters.find("Torque Enable") != gpio.parameters.end()) {
      torque_enabled = std::stoi(gpio.parameters.at("Torque Enable")) != 0;
    }
    dxl_torque_enable_[id] = torque_enabled;

    // 1. First pass: Write Operating Mode parameters
    for (const auto & param : gpio.parameters) {
      const std::string & param_name = param.first;
      if (param_name == "Operating Mode") {
        RCLCPP_INFO_STREAM(
          logger_,
          "[ID:" << std::to_string(id) << "] item_name:" << param_name.c_str() << "\tdata:" <<
            param.second);
        if (dxl_comm_->WriteItem(
            id, param_name,
            static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
        {
          return false;
        }
      }
    }

    // 2. Second pass: Write all Limit parameters
    for (const auto & param : gpio.parameters) {
      const std::string & param_name = param.first;
      if (param_name == "ID" || param_name == "type" ||
        param_name == "Torque Enable" || param_name == "Operating Mode" ||
        param_name == "model_num" || param_name == "comm_id")
      {
        continue;
      }
      if (param_name.find("Limit") != std::string::npos) {
        RCLCPP_INFO_STREAM(
          logger_,
          "[ID:" << std::to_string(id) << "] item_name:" << param_name.c_str() << "\tdata:" <<
            param.second);
        if (dxl_comm_->WriteItem(
            id, param_name,
            static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
        {
          return false;
        }
      }
    }

    // 3. Third pass: Write all other parameters (excluding already written ones)
    for (const auto & param : gpio.parameters) {
      const std::string & param_name = param.first;
      if (param_name == "ID" || param_name == "type" ||
        param_name == "Torque Enable" || param_name == "Operating Mode" ||
        param_name == "model_num" || param_name == "comm_id" ||
        param_name.find("Limit") != std::string::npos)
      {
        continue;
      }
      RCLCPP_INFO_STREAM(
        logger_,
        "[ID:" << std::to_string(id) << "] item_name:" << param_name.c_str() << "\tdata:" <<
          param.second);
      if (dxl_comm_->WriteItem(
          id, param_name,
          static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
      {
        return false;
      }
    }
  }
  return true;
}

bool DynamixelHardware::InitControllerItems()
{
  return initItems("controller") && initItems("virtual_dxl");
}

bool DynamixelHardware::InitDxlItems()
{
  return initItems("dxl") && initItems("sensor");
}

bool DynamixelHardware::InitDxlReadItems()
{
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Read Items");
  is_set_hdl_ = false;

  if (!is_set_hdl_) {
    hdl_trans_states_.clear();
    hdl_gpio_sensor_states_.clear();
    hdl_gpio_controller_states_.clear();
    for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
      if (gpio.parameters.at("type") == "dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_read;
        temp_read.id = id;
        temp_read.comm_id = id;
        temp_read.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_read.interface_name_vec.push_back(it.name);
          temp_read.value_ptr_vec.push_back(std::make_shared<double>(0.0));

          if (it.name == "Hardware Error Status") {
            dxl_hw_err_[id] = 0x00;
          }
        }
        hdl_trans_states_.push_back(temp_read);
      } else if (gpio.parameters.at("type") == "sensor") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_sensor;
        temp_sensor.id = id;
        temp_sensor.comm_id = id;
        temp_sensor.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_sensor.interface_name_vec.push_back(it.name);
          temp_sensor.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_sensor_states_.push_back(temp_sensor);
      } else if (gpio.parameters.at("type") == "controller") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_controller;
        temp_controller.id = id;
        temp_controller.comm_id = id;
        temp_controller.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_controller.interface_name_vec.push_back(it.name);
          temp_controller.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_controller_states_.push_back(temp_controller);
      } else if (gpio.parameters.at("type") == "virtual_dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_read;
        temp_read.id = id;
        temp_read.comm_id = comm_id;
        temp_read.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_read.interface_name_vec.push_back(it.name);
          temp_read.value_ptr_vec.push_back(std::make_shared<double>(0.0));

          if (it.name == "Hardware Error Status") {
            dxl_hw_err_[id] = 0x00;
          }
        }
        hdl_trans_states_.push_back(temp_read);
      }
    }
    is_set_hdl_ = true;
  }
  for (auto it : hdl_gpio_controller_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.id, it.comm_id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  for (auto it : hdl_trans_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.id, it.comm_id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  for (auto it : hdl_gpio_sensor_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.id, it.comm_id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  if (dxl_comm_->SetMultiDxlRead() != DxlError::OK) {
    return false;
  }
  return true;
}

bool DynamixelHardware::InitDxlWriteItems()
{
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Write Items");
  is_set_hdl_ = false;

  if (!is_set_hdl_) {
    hdl_trans_commands_.clear();
    hdl_gpio_controller_commands_.clear();
    for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
      if (gpio.parameters.at("type") == "dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_write;
        temp_write.id = id;
        temp_write.comm_id = id;
        temp_write.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          if (it.name != "Goal Position" &&
            it.name != "Goal Velocity" &&
            it.name != "Goal Current")
          {
            continue;
          }
          temp_write.interface_name_vec.push_back(it.name);
          temp_write.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_trans_commands_.push_back(temp_write);
      } else if (gpio.parameters.at("type") == "controller") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_controller;
        temp_controller.id = id;
        temp_controller.comm_id = id;
        temp_controller.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          temp_controller.interface_name_vec.push_back(it.name);
          temp_controller.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_controller_commands_.push_back(temp_controller);
      } else if (gpio.parameters.at("type") == "virtual_dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_write;
        temp_write.id = id;
        temp_write.comm_id = comm_id;
        temp_write.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          if (it.name != "Goal Position" &&
            it.name != "Goal Velocity" &&
            it.name != "Goal Current")
          {
            continue;
          }
          temp_write.interface_name_vec.push_back(it.name);
          temp_write.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_trans_commands_.push_back(temp_write);
      }
    }
    is_set_hdl_ = true;
  }

  for (auto it : hdl_trans_commands_) {
    if (dxl_comm_->SetDxlWriteItems(
        it.id, it.comm_id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }

  for (auto it : hdl_gpio_controller_commands_) {
    if (dxl_comm_->SetDxlWriteItems(
        it.id, it.comm_id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  if (dxl_comm_->SetMultiDxlWrite() != DxlError::OK) {
    return false;
  }

  return true;
}

void DynamixelHardware::ReadSensorData(const HandlerVarType & sensor)
{
  for (auto item : sensor.interface_name_vec) {
    for (size_t i = 0; i < hdl_sensor_states_.size(); i++) {
      for (size_t j = 0; j < hdl_sensor_states_.at(i).interface_name_vec.size(); j++) {
        if (hdl_sensor_states_.at(i).name == sensor.name &&
          hdl_sensor_states_.at(i).interface_name_vec.at(j) == item)
        {
          *hdl_sensor_states_.at(i).value_ptr_vec.at(j) = *sensor.value_ptr_vec.at(j);
        }
      }
    }
  }
}

void DynamixelHardware::SetMatrix()
{
  std::string str;
  std::vector<double> d_vec;

  // dynamic allocation (number_of_transmissions x number_of_joint)
  transmission_to_joint_matrix_ = new double *[num_of_joints_];
  for (size_t i = 0; i < num_of_joints_; i++) {
    transmission_to_joint_matrix_[i] = new double[num_of_transmissions_];
  }

  d_vec.clear();
  std::stringstream ss_tj(info_.hardware_parameters["transmission_to_joint_matrix"]);
  while (std::getline(ss_tj, str, ',')) {
    d_vec.push_back(stod(str));
  }
  for (size_t i = 0; i < num_of_joints_; i++) {
    for (size_t j = 0; j < num_of_transmissions_; j++) {
      transmission_to_joint_matrix_[i][j] = d_vec.at(i * num_of_transmissions_ + j);
    }
  }

  fprintf(stderr, "transmission_to_joint_matrix_ \n");
  for (size_t i = 0; i < num_of_joints_; i++) {
    for (size_t j = 0; j < num_of_transmissions_; j++) {
      fprintf(stderr, "[%zu][%zu] %lf, ", i, j, transmission_to_joint_matrix_[i][j]);
    }
    fprintf(stderr, "\n");
  }

  joint_to_transmission_matrix_ = new double *[num_of_transmissions_];
  for (size_t i = 0; i < num_of_transmissions_; i++) {
    joint_to_transmission_matrix_[i] = new double[num_of_joints_];
  }

  d_vec.clear();
  std::stringstream ss_jt(info_.hardware_parameters["joint_to_transmission_matrix"]);
  while (std::getline(ss_jt, str, ',')) {
    d_vec.push_back(stod(str));
  }
  for (size_t i = 0; i < num_of_transmissions_; i++) {
    for (size_t j = 0; j < num_of_joints_; j++) {
      joint_to_transmission_matrix_[i][j] = d_vec.at(i * num_of_joints_ + j);
    }
  }


  fprintf(stderr, "joint_to_transmission_matrix_ \n");
  for (size_t i = 0; i < num_of_transmissions_; i++) {
    for (size_t j = 0; j < num_of_joints_; j++) {
      fprintf(stderr, "[%zu][%zu] %lf, ", i, j, joint_to_transmission_matrix_[i][j]);
    }
    fprintf(stderr, "\n");
  }
}

void DynamixelHardware::MapInterfaces(
  size_t outer_size,
  size_t inner_size,
  std::vector<HandlerVarType> & outer_handlers,
  std::vector<HandlerVarType> & inner_handlers,
  double ** matrix,
  const std::unordered_map<std::string, std::vector<std::string>> & iface_map,
  const std::string & conversion_iface,
  const std::string & conversion_name,
  std::function<double(double)> conversion)
{
  for (size_t i = 0; i < outer_size; ++i) {
    for (size_t k = 0; k < outer_handlers.at(i).interface_name_vec.size(); ++k) {
      double value = 0.0;
      const std::string & outer_iface = outer_handlers.at(i).interface_name_vec.at(k);
      auto map_it = iface_map.find(outer_iface);
      if (map_it == iface_map.end()) {
        std::ostringstream oss;
        oss << "No mapping found for '" << outer_handlers.at(i).name
            << "', interface '" << outer_iface
            << "'. Skipping. Available mapping keys: [";
        size_t key_count = 0;
        for (const auto & pair : iface_map) {
          oss << pair.first;
          if (++key_count < iface_map.size()) {oss << ", ";}
        }
        oss << "]";
        RCLCPP_WARN_STREAM(logger_, oss.str());
        continue;
      }
      const std::vector<std::string> & mapped_ifaces = map_it->second;
      for (size_t j = 0; j < inner_size; ++j) {
        for (const auto & mapped_iface : mapped_ifaces) {
          auto it = std::find(
            inner_handlers.at(j).interface_name_vec.begin(),
            inner_handlers.at(j).interface_name_vec.end(),
            mapped_iface);
          if (it != inner_handlers.at(j).interface_name_vec.end()) {
            size_t idx = std::distance(inner_handlers.at(j).interface_name_vec.begin(), it);
            value += matrix[i][j] * (*inner_handlers.at(j).value_ptr_vec.at(idx));
            break;
          }
        }
      }
      if (!conversion_iface.empty() && !conversion_name.empty() &&
        outer_iface == conversion_iface &&
        outer_handlers.at(i).name == conversion_name &&
        conversion)
      {
        value = conversion(value);
      }
      *outer_handlers.at(i).value_ptr_vec.at(k) = value;
    }
  }
}

void DynamixelHardware::CalcTransmissionToJoint()
{
  std::function<double(double)> conv = use_revolute_to_prismatic_ ?
    std::function<double(double)>(
    std::bind(&DynamixelHardware::revoluteToPrismatic, this, std::placeholders::_1)) :
    std::function<double(double)>();
  this->MapInterfaces(
    num_of_joints_,
    num_of_transmissions_,
    hdl_joint_states_,
    hdl_trans_states_,
    transmission_to_joint_matrix_,
    dynamixel_hardware_interface::ros2_to_dxl_state_map,
    hardware_interface::HW_IF_POSITION,
    conversion_joint_name_,
    conv
  );
}

void DynamixelHardware::CalcJointToTransmission()
{
  std::function<double(double)> conv = use_revolute_to_prismatic_ ?
    std::function<double(double)>(
    std::bind(&DynamixelHardware::prismaticToRevolute, this, std::placeholders::_1)) :
    std::function<double(double)>();
  this->MapInterfaces(
    num_of_transmissions_,
    num_of_joints_,
    hdl_trans_commands_,
    hdl_joint_commands_,
    joint_to_transmission_matrix_,
    dynamixel_hardware_interface::dxl_to_ros2_cmd_map,
    "Goal Position",
    conversion_dxl_name_,
    conv
  );
}

void DynamixelHardware::SyncJointCommandWithStates()
{
  for (auto & it_states : hdl_joint_states_) {
    for (auto & it_commands : hdl_joint_commands_) {
      if (it_states.name == it_commands.name) {
        std::string pos_cmd_name = hardware_interface::HW_IF_POSITION;
        // Find index in command interfaces
        auto cmd_it = std::find(
          it_commands.interface_name_vec.begin(),
          it_commands.interface_name_vec.end(),
          pos_cmd_name);
        if (cmd_it == it_commands.interface_name_vec.end()) {
          RCLCPP_WARN_STREAM(
            logger_,
            "No position interface found in command interfaces for joint '" <<
              it_commands.name << "'. Skipping sync!");
          continue;
        }
        size_t cmd_idx = std::distance(it_commands.interface_name_vec.begin(), cmd_it);
        // Find index in state interfaces
        auto state_it = std::find(
          it_states.interface_name_vec.begin(),
          it_states.interface_name_vec.end(),
          pos_cmd_name);
        if (state_it == it_states.interface_name_vec.end()) {
          RCLCPP_WARN_STREAM(
            logger_,
            "No position interface found in state interfaces for joint '" <<
              it_states.name << "'. Skipping sync!");
          continue;
        }
        size_t state_idx = std::distance(it_states.interface_name_vec.begin(), state_it);
        // Sync the value
        *it_commands.value_ptr_vec.at(cmd_idx) = *it_states.value_ptr_vec.at(state_idx);
        RCLCPP_INFO_STREAM(
          logger_, "Sync joint state to command (joint: " << it_states.name << ", " <<
            it_commands.interface_name_vec.at(cmd_idx).c_str() << ", " <<
            *it_commands.value_ptr_vec.at(cmd_idx) << " <- " <<
            it_states.interface_name_vec.at(state_idx).c_str() << ", " <<
            *it_states.value_ptr_vec.at(state_idx));
      }
    }
  }
}

void DynamixelHardware::ChangeDxlTorqueState()
{
  if (dxl_torque_status_ == REQUESTED_TO_ENABLE) {
    std::cout << "torque enable" << std::endl;
    dxl_comm_->DynamixelEnable(dxl_id_);
    dxl_comm_->DynamixelEnable(virtual_dxl_id_);
    SyncJointCommandWithStates();
  } else if (dxl_torque_status_ == REQUESTED_TO_DISABLE) {
    std::cout << "torque disable" << std::endl;
    dxl_comm_->DynamixelDisable(dxl_id_);
    dxl_comm_->DynamixelDisable(virtual_dxl_id_);
    SyncJointCommandWithStates();
  }

  dxl_torque_state_ = dxl_comm_->GetDxlTorqueState();
  for (auto single_torque_state : dxl_torque_state_) {
    if (single_torque_state.second == false) {
      dxl_torque_status_ = TORQUE_DISABLED;
      return;
    }
  }
  dxl_torque_status_ = TORQUE_ENABLED;
}

void DynamixelHardware::get_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response)
{
  uint8_t id = static_cast<uint8_t>(request->id);
  std::string name = request->item_name;

  if (dxl_comm_->InsertReadItemBuf(id, name) != DxlError::OK) {
    RCLCPP_ERROR_STREAM(logger_, "get_dxl_data_srv_callback InsertReadItemBuf");

    response->result = false;
    return;
  }
  double timeout_sec = request->timeout_sec;
  if (timeout_sec == 0.0) {
    timeout_sec = 1.0;
  }
  rclcpp::Time t_start = rclcpp::Clock().now();
  while (dxl_comm_->CheckReadItemBuf(id, name) == false) {
    if ((rclcpp::Clock().now() - t_start).seconds() > timeout_sec) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "get_dxl_data_srv_callback Timeout : " << (rclcpp::Clock().now() - t_start).seconds() );
      response->result = false;
      return;
    }
  }

  response->item_data = dxl_comm_->GetReadItemDataBuf(id, name);
  response->result = true;
}

void DynamixelHardware::set_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Response> response)
{
  uint8_t dxl_id = static_cast<uint8_t>(request->id);
  uint32_t dxl_data = static_cast<uint32_t>(request->item_data);
  if (dxl_comm_->InsertWriteItemBuf(dxl_id, request->item_name, dxl_data) == DxlError::OK) {
    response->result = true;
  } else {
    response->result = false;
  }
}

void DynamixelHardware::reboot_dxl_srv_callback(
  [[maybe_unused]] const std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Response> response)
{
  if (CommReset()) {
    response->result = true;
    RCLCPP_INFO_STREAM(logger_, "[reboot_dxl_srv_callback] SUCCESS");
  } else {
    response->result = false;
    RCLCPP_INFO_STREAM(logger_, "[reboot_dxl_srv_callback] FAIL");
  }
}

void DynamixelHardware::set_dxl_torque_srv_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (dxl_torque_status_ == TORQUE_ENABLED) {
      response->success = true;
      response->message = "Already enabled.";
      return;
    } else {
      dxl_torque_status_ = REQUESTED_TO_ENABLE;
    }
  } else {
    if (dxl_torque_status_ == TORQUE_DISABLED) {
      response->success = true;
      response->message = "Already disabled.";
      return;
    } else {
      dxl_torque_status_ = REQUESTED_TO_DISABLE;
    }
  }

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) {
    if (dxl_torque_status_ == TORQUE_ENABLED) {
      if (request->data) {
        response->success = true;
        response->message = "Success to enable.";
      } else {
        response->success = false;
        response->message = "Fail to enable.";
      }
      return;
    } else if (dxl_torque_status_ == TORQUE_DISABLED) {
      if (!request->data) {
        response->success = true;
        response->message = "Success to disable.";
      } else {
        response->success = false;
        response->message = "Fail to disable.";
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  response->success = false;
  response->message = "Fail to write requeset. main thread is not running.";
}

void DynamixelHardware::initRevoluteToPrismaticParam()
{
  if (info_.hardware_parameters.find("revolute_to_prismatic_dxl") !=
    info_.hardware_parameters.end())
  {
    conversion_dxl_name_ = info_.hardware_parameters.at("revolute_to_prismatic_dxl");
  }

  if (info_.hardware_parameters.find("revolute_to_prismatic_joint") !=
    info_.hardware_parameters.end())
  {
    conversion_joint_name_ = info_.hardware_parameters.at("revolute_to_prismatic_joint");
  }

  if (info_.hardware_parameters.find("prismatic_min") != info_.hardware_parameters.end()) {
    prismatic_min_ = std::stod(info_.hardware_parameters.at("prismatic_min"));
  }

  if (info_.hardware_parameters.find("prismatic_max") != info_.hardware_parameters.end()) {
    prismatic_max_ = std::stod(info_.hardware_parameters.at("prismatic_max"));
  }

  if (info_.hardware_parameters.find("revolute_min") != info_.hardware_parameters.end()) {
    revolute_min_ = std::stod(info_.hardware_parameters.at("revolute_min"));
  }

  if (info_.hardware_parameters.find("revolute_max") != info_.hardware_parameters.end()) {
    revolute_max_ = std::stod(info_.hardware_parameters.at("revolute_max"));
  }

  conversion_slope_ = (prismatic_max_ - prismatic_min_) / (revolute_max_ - revolute_min_);
  conversion_intercept_ = prismatic_min_ - conversion_slope_ * revolute_min_;
}

double DynamixelHardware::revoluteToPrismatic(double revolute_value)
{
  return conversion_slope_ * revolute_value + conversion_intercept_;
}

double DynamixelHardware::prismaticToRevolute(double prismatic_value)
{
  return (prismatic_value - conversion_intercept_) / conversion_slope_;
}

}  // namespace dynamixel_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_hardware_interface::DynamixelHardware,
  hardware_interface::SystemInterface
)
