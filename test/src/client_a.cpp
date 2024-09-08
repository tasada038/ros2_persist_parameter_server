// Copyright (c) 2024 Takumi Asada
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


//------------------------------------------------------------------------------------
//----------------------------------- Include ----------------------------------------
//------------------------------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include <memory>  // For std::shared_ptr
#include <vector>  // For std::vector
#include <chrono>  // For std::chrono literals

using namespace std::chrono_literals;  // Enables 1s, 500ms, etc.


//------------------------------------------------------------------------------------
//----------------------------------- Class ------------------------------------------
//------------------------------------------------------------------------------------
class ClientANode : public rclcpp::Node
{
public:
    ClientANode()
    : Node("client_a_node")
    {
        // Create a parameter client to interact with the parameter server
        parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/parameter_server");

        // Wait for the parameter server to be available before proceeding
        parameters_client->wait_for_service();

        // Set up a periodic timer to update the parameter every 5 seconds
        timer_ = this->create_wall_timer(
            5s,  // Update every 5 seconds
            std::bind(&ClientANode::updateSomeIntParameter, this));
    }

/**
 * @brief Periodically updates the 'persistent.some_int' parameter by incrementing its value.
 */
    void updateSomeIntParameter()
    {
        // Fetch the current value of 'persistent.some_int'
        auto parameters_future = parameters_client->get_parameters(
            {"persistent.some_int"},
            std::bind(&ClientANode::callbackParamServer, this, std::placeholders::_1));
    }

/**
 * @brief Callback function to handle the result from the parameter server.
 *        This retrieves the current value of the parameter and increments it.
 * @param future Future object containing the fetched parameters from the server.
 */
    void callbackParamServer(std::shared_future<std::vector<rclcpp::Parameter>> future)
    {
        auto result = future.get();

        if (!result.empty()) {
            // Extract the current value of the 'persistent.some_int' parameter
            int64_t current_value = result.at(0).as_int();
            RCLCPP_INFO(this->get_logger(), "Current 'persistent.some_int': %ld", current_value);

            // Increment the value and update it back to the server
            int64_t new_value = current_value + 1;
            parameters_client->set_parameters({rclcpp::Parameter("persistent.some_int", new_value)});
            RCLCPP_INFO(this->get_logger(), "Updated 'persistent.some_int' to: %ld", new_value);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get 'persistent.some_int' parameter.");
        }
    }


private:
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;  ///< Client for interacting with the parameter server
    rclcpp::TimerBase::SharedPtr timer_;  ///< Timer to trigger the periodic parameter update
};


//------------------------------------------------------------------------------------
//----------------------------------- Main Function ----------------------------------
//------------------------------------------------------------------------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Spin the node to continuously run
    rclcpp::spin(std::make_shared<ClientANode>());

    rclcpp::shutdown();
    return 0;
}
