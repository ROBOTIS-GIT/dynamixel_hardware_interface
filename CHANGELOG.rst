^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.9 (2025-06-24)
------------------
* Support ffw sensor model
* Contributors: Woojin Wie

1.4.8 (2025-06-23)
------------------
* Added new model for OMY to use virtual_dxl
* Added goal position synchronization before torque enable for OMY sync table feature
* Contributors: Woojin Wie

1.4.7 (2025-06-19)
------------------
* Added virtual_dxl and support for rcu
* Contributors: Woojin Wie

1.4.6 (2025-05-30)
------------------
* Changed dynamixel_sdk_TARGETS to dynamixel_sdk_LIBRARIES in target_link_libraries
* Contributors: Woojin Wie

1.4.5 (2025-05-30)
------------------
* Deprecate ament_include_dependency usage in CMakeLists.txt
* Contributors: Wonho Yun

1.4.4 (2025-05-28)
------------------
* Added proper command interface support with ROS2-Dynamixel interface mapping
* Improved error handling and robustness throughout the codebase
* Implemented per-device torque enable control (replacing global control)
* Added support for new sensor model (sensorxel_joy)
* Enhanced joint state-command synchronization
* Improved parameter initialization organization
* Added robust error handling for model file reading
* Contributors: Woojin Wie

1.4.3 (2025-04-10)
------------------
* Fixed build errors
* Contributors: Wonho Yun

1.4.2 (2025-04-05)
------------------
* Added OM-Y dynamixel model files
* Added a function to enable torque
* Fixed the configuration for OM-Y robots
* Contributors: Woojin Wie, Wonho Yun

1.4.1 (2025-03-31)
------------------
* Modified the Model File
* Contributors: Wonho Yun

1.4.0 (2025-03-20)
------------------
* Added Torque Constant Parameter to DXL Model Files
* Enhanced Transmission Command Calculation
* Unified Initialization Structure
* Support for Goal Current Control
* Contributors: Woojin Wie

1.3.0 (2025-02-17)
------------------
* Enhance Error Handling and Timeout Management
* Use GroupFastSyncRead and GroupFastBulkRead
* Remove deprecated parameter ros_update_freq_ to prevent stoi failure
* Contributors: Woojin Wie

1.2.0 (2025-01-17)
------------------
* Extend Bulk/Sync Selection Logic to Include Indirect Operations
* Enhance DXL item initialization by prioritizing 'Limit' parameters
* Contributors: Woojin Wie

1.1.0 (2024-12-27)
------------------
* Added new control table entries for Dynamixel X
* Contributors: Woojin Wie, Hye-Jong Kim

1.0.0 (2024-12-04)
------------------
* First release of dynamixel_hardware_interface package
* Contributors: Hye-Jong Kim, Sungho Woo
