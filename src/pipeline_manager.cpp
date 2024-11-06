// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_pipeline_manager/pipeline_server.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto pipeline_server =
      std::make_shared<qrb_ros::pipeline_manager::PipelineServer>();
  executor.add_node(pipeline_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
