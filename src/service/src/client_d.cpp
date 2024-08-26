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
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/addition.hpp"

namespace service
{
class ClientD : public rclcpp::Node
{
public:
    ClientD() : Node("client_d")
    {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
        addition_client_ = this->create_client<custom_msgs::srv::Addition>("service_a", rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);

        while (!addition_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Server not available, waiting again...");
        }

        RCLCPP_INFO(this->get_logger(), "Server available");

        auto request = std::make_shared<custom_msgs::srv::Addition::Request>();
        request->a = 1.0f;
        request->b = 2.0f;
        RCLCPP_INFO(this->get_logger(), "Sending request");
        auto future = addition_client_->async_send_request(request);
        if (future.wait_for(std::chrono::milliseconds(5000)) == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Timeout");
            return;
        }

        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %f", result->sum);
    }

private:

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Client<custom_msgs::srv::Addition>::SharedPtr addition_client_;
};
}  // namespace service

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<service::ClientD>());
    rclcpp::shutdown();
    return 0;
}

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<service::ClientD>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }
