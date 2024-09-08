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
#include <string>  // For std::string

using namespace std::chrono_literals;  // Enables 1s, 500ms, etc.


//------------------------------------------------------------------------------------
//----------------------------------- Class ------------------------------------------
//------------------------------------------------------------------------------------
class ClientBNode : public rclcpp::Node
{
public:
    ClientBNode()
    : Node("client_b_node")
    {
        // Create a parameter client to interact with the parameter server
        parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/parameter_server");

        // Wait for the parameter server to be available before proceeding
        parameters_client->wait_for_service();

        // Set up a periodic timer to update parameters every 5 seconds
        timer_ = this->create_wall_timer(
            5s,  // Update every 5 seconds
            std::bind(&ClientBNode::updateParameters, this));
    }

/**
 * @brief Periodically updates multiple parameters by setting them to new values.
 */
    void updateParameters()
    {
        // Example new values for parameters (these can be set from API or other sources)
        double new_double_value = 3.14;
        std::vector<double> new_double_array_value = {1.1, 2.2, 3.3};

        // Create a vector of parameters to update
        std::vector<rclcpp::Parameter> parameters_to_update;
        parameters_to_update.emplace_back("persistent.double", new_double_value);
        parameters_to_update.emplace_back("persistent.double_array", new_double_array_value);

        // Update the parameters on the server
        parameters_client->set_parameters(parameters_to_update);
        RCLCPP_INFO(this->get_logger(), "Updated parameters: 'persistent.double' and 'persistent.double_array'");
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
    rclcpp::spin(std::make_shared<ClientBNode>());

    rclcpp::shutdown();
    return 0;
}
