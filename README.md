# RosApp - ROS2 AGV Control Application

A ROS2-based application for Automated Guided Vehicle (AGV) control and simulation.

## Project Structure

```
RosApp/
├── src/
│   └── my_ros_tutorials/
│       ├── my_turtlesim/          # Custom turtlesim with Modbus support
│       └── my_turtlesim_msgs/     # Custom message definitions
├── docs/                          # Documentation directory
│   ├── README.md                  # Documentation overview
│   └── PESD_CAN_ESD_Protection.md # Hardware ESD protection guide
├── turtle_config.txt              # Turtle configuration file
└── turtle_config1.txt             # Alternative turtle configuration
```

## Features

- Custom turtlesim implementation with industrial automation features
- Modbus TCP communication support for AGV control
- State machine for automated task execution
- Action servers for precise movement control
- Collision detection and path planning

## Documentation

For detailed documentation, please see the [docs directory](docs/):

- **[Hardware Documentation](docs/)**: ESD protection for CAN bus interfaces
  - [PESD1CAN vs PESD2CAN Comparison](docs/PESD_CAN_ESD_Protection.md) - 详细说明两种ESD保护器件的区别

## Building and Running

This is a ROS2 package. Build it using colcon:

```bash
colcon build --packages-select my_turtlesim my_turtlesim_msgs
source install/setup.bash
```

## Dependencies

- ROS2 (Humble or later recommended)
- Qt5 or Qt6
- libmodbus
- Standard ROS2 packages (rclcpp, geometry_msgs, etc.)

## Communication Protocols

- **Modbus TCP/RTU**: Primary protocol for industrial automation
- **CAN Bus**: Optional interface (with appropriate ESD protection)

For CAN bus hardware protection, see the [PESD CAN ESD Protection Guide](docs/PESD_CAN_ESD_Protection.md).

## License

Please refer to individual package licenses in their respective directories.
