# RosApp Documentation

This directory contains documentation for the RosApp project.

## Available Documentation

### Hardware Documentation

- **[PESD CAN ESD Protection](PESD_CAN_ESD_Protection.md)** - 详细说明PESD1CAN和PESD2CAN的区别 (Detailed explanation of the differences between PESD1CAN and PESD2CAN)
  - ESD protection devices comparison for CAN bus interfaces
  - Technical specifications
  - Pin configurations
  - Application guidelines

## Project Overview

This ROS2 application is designed for AGV (Automated Guided Vehicle) control and communication. The project includes:

- Custom turtlesim implementation with Modbus support
- AGV state machine for automated control
- Modbus communication interface for industrial automation
- Action servers for robot movement control

## Hardware Communication

The project supports multiple communication protocols:
- **Modbus TCP/RTU**: Primary protocol for AGV control
- **CAN Bus**: Optional interface for vehicle-level communication (requires appropriate ESD protection)

For CAN bus implementations, refer to the [PESD CAN ESD Protection](PESD_CAN_ESD_Protection.md) documentation for selecting appropriate protection components.
