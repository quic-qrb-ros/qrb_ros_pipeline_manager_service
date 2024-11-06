// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef PIPELINE_SERVER_HPP_
#define PIPELINE_SERVER_HPP_

#include "ament_index_cpp/get_package_prefix.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include "qrb_ros_pipeline_manager_msgs/msg/node_info.hpp"
#include "qrb_ros_pipeline_manager_msgs/srv/pipeline.hpp"
#include "rclcpp_components/component_manager.hpp"

namespace qrb_ros {
namespace pipeline_manager {

struct PipelineContainer {
  int pipeline_id;
  std::string pipeline_namespace;
  pid_t pid;
  int node_num;
  std::vector<qrb_ros_pipeline_manager_msgs::msg::NodeInfo> node_info;
};

class PipelineServer : public rclcpp::Node {
public:
  PipelineServer();

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  void handle_service(
      const std::shared_ptr<qrb_ros_pipeline_manager_msgs::srv::Pipeline::Request>
          request,
      std::shared_ptr<qrb_ros_pipeline_manager_msgs::srv::Pipeline::Response> response);

  int create_container();

  bool destroy_container(int pipeline_id);

  bool load_node_to_container(
      PipelineContainer &target_container,
      const std::shared_ptr<qrb_ros_pipeline_manager_msgs::srv::Pipeline::Request>
          request);

  std::unordered_map<int, PipelineContainer>
      containers_; // Store container id and their process IDs

  rclcpp::Service<qrb_ros_pipeline_manager_msgs::srv::Pipeline>::SharedPtr service_;

  int pipeline_num;
};

} // namespace pipeline_manager
} // namespace qrb_ros

#endif
