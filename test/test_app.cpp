//Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
//SPDX-License-Identifier: BSD-3-Clause-Clear

#include "rclcpp/rclcpp.hpp"
#include "qrb_ros_pipeline_manager_msgs/msg/node_info.hpp"
#include "qrb_ros_pipeline_manager_msgs/srv/pipeline.hpp"

#define SERVICE_NAME "/pipeline_manager_service"


class TestService : public rclcpp::Node
{
public:
  TestService();

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_{ nullptr };
  rclcpp::Client<qrb_ros_pipeline_manager_msgs::srv::Pipeline>::SharedPtr client_{ nullptr };
  int action;
  rclcpp::TimerBase::SharedPtr timer_{ nullptr };
  void timer_callback();
};


TestService::TestService() : Node("test_service") {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->declare_parameter("action", 0);
    action = this->get_parameter("action").as_int();
    client_ = this->create_client<qrb_ros_pipeline_manager_msgs::srv::Pipeline>(SERVICE_NAME, rmw_qos_profile_services_default, callback_group_);
    timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&TestService::timer_callback, this), callback_group_);
}

void TestService::timer_callback() {
  timer_->cancel();

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
  }

  auto request = std::make_unique<qrb_ros_pipeline_manager_msgs::srv::Pipeline::Request>();

  qrb_ros_pipeline_manager_msgs::msg::NodeInfo camera_node;
  camera_node.package_name = "qrb_ros_pipeline_manager";
  camera_node.plugin_name = "CameraNode";
  camera_node.node_name = "camera_node";
  camera_node.pub_topic_name = "raw_image";
  camera_node.sub_topic_name = "sub_topic_name";

  // Initialize Resize Node
  qrb_ros_pipeline_manager_msgs::msg::NodeInfo resize_node;
  resize_node.package_name = "qrb_ros_pipeline_manager";
  resize_node.plugin_name = "ImageResizeNode";
  resize_node.node_name = "image_resize_node";
  resize_node.pub_topic_name = "resized_image";
  resize_node.sub_topic_name = "raw_image";

  resize_node.parameters.push_back(rclcpp::Parameter("width", 720).to_parameter_msg());
  resize_node.parameters.push_back(rclcpp::Parameter("height", 640).to_parameter_msg());
  RCLCPP_INFO(this->get_logger(),"action = %d", action); 
  switch(action)
  {
    case 0:    
      request->action = action;
      break;
    case 1:
      request->action = action;
      request->pipeline_id = 1;
      break;
    case 2:
      request->action = action;
      request->pipeline_id = 1;
      request->nodes_info.push_back(camera_node);
      request->nodes_info.push_back(resize_node);
      break;
  }

  // Wait for the service to process the request
  auto future = client_->async_send_request(std::move(request));
  auto status = future.wait_for(std::chrono::seconds(5));
  if (status != std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(),"future status is timeout");
    return;
  }
  auto result = future.get();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TestService>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

