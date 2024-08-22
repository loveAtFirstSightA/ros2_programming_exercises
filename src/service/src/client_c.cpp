/*
 Copyright 2024 Google LLC

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <chrono>
#include <memory>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/addition.hpp"

namespace service
{
class ClientC : public rclcpp::Node
{
public:
    ClientC() : Node("client_c") 
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000),
            std::bind(&ClientC::timerCallback, this));
        // addition_client_ = this->create_client<custom_msgs::srv::Addition>("service_a");

        node_ = std::make_shared<rclcpp::Node>("client_cc");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        addition_client_ = node_->create_client<custom_msgs::srv::Addition>("service_a", rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);
    }

private:
    void timerCallback() 
    {
        static unsigned int count = 0;
        RCLCPP_INFO(this->get_logger(), "Timer event %d", count++);
        
        while (!addition_client_->wait_for_service(std::chrono::milliseconds(500))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Server not available, waiting again...");
        }

        static float a = 0.0, b = 0.0;
        auto request = std::make_shared<custom_msgs::srv::Addition::Request>();
        request->a = ++a;
        request->b = ++b;

        // 记录发送请求前的信息
        RCLCPP_INFO(node_->get_logger(), "Sending request to service '%s'", addition_client_->get_service_name());

        // 使用异步请求并非阻塞等待
        auto future_result = addition_client_->async_send_request(request);

        // 记录请求发送后的信息
        RCLCPP_INFO(node_->get_logger(), "Request sent. Waiting for response...");

        auto timeout = std::chrono::milliseconds(500);
        
        // 是否超时
        if (callback_group_executor_.spin_until_future_complete(future_result, timeout) == rclcpp::FutureReturnCode::TIMEOUT) {
            RCLCPP_ERROR(this->get_logger(), "Timeout set: %ld ms", timeout.count());
        }

        // 是否成功接收
        if (callback_group_executor_.spin_until_future_complete(future_result) != rclcpp::FutureReturnCode::SUCCESS) {
            // 请求失败，记录错误信息并移除挂起的请求
            RCLCPP_ERROR(node_->get_logger(), "Failed to complete the service request to '%s' within the timeout period.", addition_client_->get_service_name());
            addition_client_->remove_pending_request(future_result);
        } else {
            try {
                // 获取响应并记录成功信息
                auto response = future_result.get();
                RCLCPP_INFO(node_->get_logger(), "Received response from service '%s': sum = %f", addition_client_->get_service_name(), response->sum);

                // 在这里可以处理响应数据
                if (response->sum != 0) {
                    RCLCPP_INFO(this->get_logger(), "Handle response");
                }

            } catch (const std::exception& e) {
                // 记录异常信息
                RCLCPP_ERROR(node_->get_logger(), "Exception occurred while getting response: %s", e.what());
                addition_client_->remove_pending_request(future_result);
            }
        }
        RCLCPP_INFO(this->get_logger(), "End\n");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<custom_msgs::srv::Addition>::SharedPtr addition_client_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};
}  // namespace service


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<service::ClientC>());
    rclcpp::shutdown();
    return 0;
}
