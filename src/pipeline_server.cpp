// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_pipeline_manager/pipeline_server.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace qrb_ros {
namespace pipeline_manager {
#define ACTION_CREATE 0x101
#define ACTION_DESTROY 0x102
#define ACTION_LOAD 0x103

constexpr char PATH_OF_CONTAINER_LIB[] =
    "/lib/rclcpp_components/component_container_mt";
constexpr char NODE_NAME[] = "pipeline_manager";
constexpr char SERVICE_NAME[] = "/pipeline_manager_service";

PipelineServer::PipelineServer() : Node(NODE_NAME), pipeline_num(0) {
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;

  sub_options.callback_group = callback_group_;
  service_ = this->create_service<qrb_ros_pipeline_manager_msgs::srv::Pipeline>(
      SERVICE_NAME,
      std::bind(&PipelineServer::handle_service, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
}

void PipelineServer::handle_service(
    const std::shared_ptr<qrb_ros_pipeline_manager_msgs::srv::Pipeline::Request>
        request,
    std::shared_ptr<qrb_ros_pipeline_manager_msgs::srv::Pipeline::Response> response) {
  switch (request->action) {
  case ACTION_CREATE: {
    response->pipeline_id = create_container();
    response->success = (response->pipeline_id != -1);
    response->node_name = "null";
    response->output_topic_name = "null";
    break;
  }
  case ACTION_DESTROY: {
    response->success = destroy_container(request->pipeline_id);
    break;
  }
  case ACTION_LOAD: {
    auto it = containers_.find(request->pipeline_id);
    if (it == containers_.end())
      return;
    auto target_container = it->second;
    response->success = load_node_to_container(target_container, request);
    if(response->success != true)
	return;
    auto end_node = target_container.node_info.back();
    response->node_name =
        "/" + target_container.pipeline_namespace + "/" + end_node.node_name;
    response->output_topic_name = "/" + target_container.pipeline_namespace +
                                  "/" + end_node.pub_topic_name;
    break;
  }
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown action: %d", request->action);
    break;
  }
}

int PipelineServer::create_container() {
  PipelineContainer Container;
  std::string rclcpp_components_path =
      ament_index_cpp::get_package_prefix("rclcpp_components");
  std::string component_container_path =
      rclcpp_components_path + PATH_OF_CONTAINER_LIB;
  pipeline_num++;

  std::string node_container_remap_name =
      "__node:=container_" + std::to_string(pipeline_num);
  pid_t pid = fork();
  if (pid == 0) {
    int result = execl(component_container_path.c_str(),
                       "component_container_mt", "--ros-args", "--remap",
                       node_container_remap_name.c_str(), nullptr);
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to load component containers, error: %d", result);
    return -1;
  } else if (pid == -1) {
    pipeline_num--;
    perror("fork");
    exit(EXIT_FAILURE);
  } else if (pid > 0) {
    Container.pid = pid;
    Container.pipeline_id = pipeline_num;
    Container.pipeline_namespace =
        "container_" + std::to_string(Container.pipeline_id);
    Container.node_num = 0;
    containers_.emplace(Container.pipeline_id, Container);
    RCLCPP_INFO(this->get_logger(), "Container %d created successfully",
                Container.pipeline_id);
  }
  return Container.pipeline_id;
}

bool PipelineServer::destroy_container(int pipeline_id) {
  auto it = containers_.find(pipeline_id);
  if (it != containers_.end()) {
    kill(it->second.pid, SIGTERM);
    containers_.erase(pipeline_id);
    RCLCPP_INFO(this->get_logger(), "Container %d destroyed successfully",
                pipeline_id);
  } else {
    RCLCPP_WARN(this->get_logger(), "Container %d does not exist", pipeline_id);
    return false;
  }
  return true;
}

bool PipelineServer::load_node_to_container(
    PipelineContainer &target_container,
    const std::shared_ptr<qrb_ros_pipeline_manager_msgs::srv::Pipeline::Request>
        request) {
  std::string container_load_service =
      "/container_" + std::to_string(target_container.pipeline_id) +
      "/_container/load_node";
  auto client = this->create_client<composition_interfaces::srv::LoadNode>(
      container_load_service);

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for the component manager service to appear...");
  }

  if (request->nodes_info.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid Nodes info, exit");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Container %d loading node",
              target_container.pipeline_id);
  for (size_t i = 0; i < request->nodes_info.size(); ++i) {
    auto load_request =
        std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    load_request->package_name = request->nodes_info[i].package_name;
    load_request->plugin_name = request->nodes_info[i].plugin_name;
    load_request->node_name = request->nodes_info[i].node_name;
    load_request->node_namespace =
        "/container_" + std::to_string(target_container.pipeline_id);
    load_request->parameters = request->nodes_info[i].parameters;

    std::string pub_remap = request->nodes_info[i].pub_topic_name +
                        ":=" + request->nodes_info[i].pub_topic_name +
                        "_node_" + std::to_string(i) + "_pub";
    request->nodes_info[i].pub_topic_name = request->nodes_info[i].pub_topic_name +
                        "_node_" + std::to_string(i) + "_pub";

    load_request->remap_rules.push_back(pub_remap);

    if (target_container.node_num > 0) {
      auto end_node = target_container.node_info.back();
      std::string sub_remap = request->nodes_info[i].sub_topic_name +
                          ":=" + end_node.pub_topic_name;
      request->nodes_info[i].sub_topic_name = end_node.pub_topic_name;
      load_request->remap_rules.push_back(sub_remap);
    }

    auto result = client->async_send_request(std::move(load_request));
    auto status = result.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Service response timeout");
      return false;
    }
    auto load_response = result.get();

    if (!load_response->success)
      return false;
    target_container.node_info.push_back(request->nodes_info[i]);
    target_container.node_num++;
  }
  return true;
}

} // namespace pipeline_manager
} // namespace qrb_ros
