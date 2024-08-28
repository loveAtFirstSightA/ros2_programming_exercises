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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace topic
{
class Subscriber : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a Subscriber object.
     *
     * Constructs a Subscriber object that subscribes to the "topic" topic
     * and calls the `subscriberCallback` function when a message is received.
     *
     * @throws None
     */
    Subscriber() : Node("subscriber")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "topic",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&Subscriber::subscriberCallback, this, std::placeholders::_1));
    }

private:
    /**
     * @brief This function is called whenever a message is received
     * 
     * It logs the message data and prints a newline character.
     * 
     * @param msg A pointer to the std_msgs/Float64 message received
     */
    void subscriberCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: [%f]", msg->data);
        std::cout << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;

};
}  // namespace topic

/**
 * @brief Main function that initializes the ROS2 node and spins it.
 *
 * This is the entry point of the program. It initializes the ROS2 context,
 * creates an instance of the `topic::Subscriber` class, and spins the node to
 * start the topic subscription.
 *
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 *
 * @return 0 if the program exits gracefully.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<topic::Subscriber>());
    rclcpp::shutdown();
    return 0;
}
