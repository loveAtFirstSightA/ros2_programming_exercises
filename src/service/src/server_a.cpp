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

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/addition.hpp"

namespace service
{
class ServiceA : public rclcpp::Node
{
public:
    ServiceA() : Node("service_a") 
    {
        addition_server_ = this->create_service<custom_msgs::srv::Addition>(
            "service_a", std::bind(&ServiceA::additionServerCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void additionServerCallback(
        const std::shared_ptr<custom_msgs::srv::Addition::Request> request,
        std::shared_ptr<custom_msgs::srv::Addition::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request %f, %f", request->a, request->b);
        response->sum = request->a + request->b;
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(this->get_logger(), "response->sum %f \n", response->sum);
    }
    
    rclcpp::Service<custom_msgs::srv::Addition>::SharedPtr addition_server_;
};
}  // namespace service


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<service::ServiceA>());
    rclcpp::shutdown();
    return 0;
}
