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
class Publisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the Publisher class.
     * 
     * This constructor initializes the ROS2 node and creates a timer and a
     * publisher for the topic "topic". The timer's callback is set to
     * `timerCallback()`, which publishes a message to the topic every second.
     * 
     * @return None
     */
    Publisher() : Node("publisher")
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Publisher::timerCallback, this));
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("topic",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    }

private:
    /**
     * @brief Callback function for the timer.
     * 
     * This function is called every second by the timer. It publishes a message
     * to the "topic" topic, incrementing the data field by 0.1 each time.
     * 
     * @return None
     */
    void timerCallback()
    {
        static float i = 0.0;
        i += 0.1;
        auto msg = std_msgs::msg::Float64();
        msg.data = i;
        RCLCPP_INFO(this->get_logger(), "Publishing message in topic: [%f]", msg.data);
        publisher_->publish(msg);
        std::cout << std::endl;    
    }
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};
}  // namespace topic

/**
 * @brief Main function of the node.
 *
 * This function initializes the ROS2 context, creates an instance of the
 * `topic::Publisher` class, and spins the node to start the topic publishing.
 *
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 *
 * @return 0 if the program exits gracefully.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<topic::Publisher>());
    rclcpp::shutdown();
    return 0;
}
