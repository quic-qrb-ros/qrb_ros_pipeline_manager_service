# qrb_ros_pipeline_manager_service

A ROS 2 package that provides a flexible pipeline management service using a Node Container approach. This package allows users to dynamically start, stop, and manage other ROS nodes using service calls. It is particularly useful for scenarios where nodes need to be configured and controlled at runtime without restarting the entire ROS system.

## Features

- **Node Management**: Dynamically load, unload, and manage nodes within a ROS 2 environment.
- **Service-Based Control**: Use ROS 2 services to interact with the node container, making it easy to start and stop nodes programmatically.
- **Configurable Parameters**: Pass parameters such as topic names, image resolutions, and more during node instantiation.

## Prerequisites

Before using this package, ensure you have the following installed:

- ROS 2 Humble Hawksbill
- A properly configured ROS 2 workspace
- C++17 support

## Installation

Clone the repository into your ROS 2 workspace's `src` directory and build the workspace.

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/qrb_ros_pipeline_manager_service.git
cd ~/ros2_ws
colcon build --packages-select qrb_ros_pipeline_manager_service
```

Make sure to source your ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### 1. Start the Node Container

The Node Container is responsible for hosting and managing other nodes. Start the container using:

```bash
ros2 run qrb_ros_pipeline_manager_service node_container
```

### 2. Start the Service Node

The Service Node provides a service to manage other nodes (e.g., start and stop). Start the service node with:

```bash
ros2 run qrb_ros_pipeline_manager_service service_node
```

### 3. Managing Nodes

To start or stop a node using the provided service, you can use the following command:

```bash
ros2 service call /manage_nodes example_interfaces/srv/Trigger "{}"
```

Replace `example_interfaces/srv/Trigger` with the appropriate service definition as needed.

### Example: Starting a Camera Node

If you have a custom camera node that accepts parameters such as topic name and resolution, you can configure these parameters using a service call to dynamically launch the camera node within the container.

### Parameters

When creating service requests, you can specify parameters such as:

- `topic_name`: The name of the topic where images will be published.
- `width`: The width of the images in pixels.
- `height`: The height of the images in pixels.

Example service call:

```bash
ros2 service call /start_camera qrb_ros_pipeline_manager_service/srv/StartCamera "{topic_name: 'camera_image', width: 1280, height: 720}"
```

## Package Structure

```
qrb_ros_pipeline_manager_service/
│
├── include/
│   └── qrb_ros_pipeline_manager_service/
│       ├── node_container.hpp
│       └── service_node.hpp
│
├── src/
│   ├── node_container.cpp
│   ├── service_node.cpp
│   └── camera_node.cpp
│
├── srv/
│   └── StartCamera.srv
│
├── CMakeLists.txt
└── package.xml
```

- **include/**: Header files for the Node Container and Service Node.
- **src/**: Source files for the main components of the package.
- **srv/**: Service definition files.
- **CMakeLists.txt**: Build instructions for the package.
- **package.xml**: Package metadata.

## Contributing

We welcome contributions! To contribute to this project:

1. Fork the repository.
2. Create a new branch with your feature or bug fix.
3. Commit your changes and push them to your fork.
4. Create a pull request to the `main` branch of the original repository.

Make sure to follow the coding standards and write tests for new functionality. Please include a detailed description of your changes in the pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [rclcpp Components](https://docs.ros.org/en/humble/Tutorials/Composition.html)

## Support

If you encounter any issues or have questions, please open an issue in the GitHub repository or reach out to the maintainers.
