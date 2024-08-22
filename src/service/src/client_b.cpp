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
class ClientB : public rclcpp::Node
{
public:
    ClientB() : Node("client_b") 
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),
            std::bind(&ClientB::timerCallback, this));
        addition_client_ = this->create_client<custom_msgs::srv::Addition>("service_a");
    }

private:
    void timerCallback() 
    {
        static unsigned int count = 0;
        RCLCPP_INFO(this->get_logger(), "Timer event %d", count ++);
        while (!addition_client_->wait_for_service(std::chrono::milliseconds(500))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Server not available, waiting again...");
        }
        static float a, b;
        auto request = std::make_shared<custom_msgs::srv::Addition::Request>();
        request->a = ++a;
        request->b = ++b;
        RCLCPP_INFO(this->get_logger(), "Sending request");

        // Call async_send_request() method
        using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::Addition>::SharedFuture;
        auto callback = [this] (ServiceResponseFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Sum: %f", response->sum);
            std::cout << std::endl;
        };
        // 发送异步请求
        auto future = addition_client_->async_send_request(request, callback);

        RCLCPP_INFO(this->get_logger(), "end");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<custom_msgs::srv::Addition>::SharedPtr addition_client_;
};
}  // namespace service


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<service::ClientB>());
    rclcpp::shutdown();
    return 0;
}
