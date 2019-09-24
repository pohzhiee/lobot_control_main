#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include "controller_manager/controller_manager.hpp"
// #include "my_robot.hpp"
// #include <controller_parameter_client/controller_parameter_client.hpp>
#include <ros2_control_robot/robot_hardware.hpp>
#include <parameter_server_interfaces/srv/get_controllers.hpp>

using parameter_server_interfaces::srv::GetControllers;
using rclcpp::Rate;
using rclcpp::executors::MultiThreadedExecutor;
using robot_hw_interface::MyRobotHardware;
using namespace std::chrono_literals;

void spin(std::shared_ptr<MultiThreadedExecutor> exe)
{
    exe->spin();
}

int main(int argc, char **argv)
{
    // do all the init stuff
    rclcpp::init(argc, argv);
    Rate r(300.0);
    bool active = true;

    // create my_robot instance
    auto robotName = "lobot";
    auto my_robot = std::make_shared<MyRobotHardware>();
    fprintf(stdout, "Finished initialising MyRobotHardware\n");
    my_robot->set_robot_name(robotName);

    // initialize the robot
    if (my_robot->init() != hardware_interface::HW_RET_OK)
    {
        fprintf(stderr, "failed to initialized hardware\n");
        return -1;
    }
    // MultiThreadedExecutor executor1;
    // executor1.add_node(master_node);
    auto executor =
        std::make_shared<MultiThreadedExecutor>();

    // start the controller manager with the robot hardware
    controller_manager::ControllerManager cm(my_robot, executor);
    // Load the controllers from the parameter server
    auto get_param_node = std::make_shared<rclcpp::Node>("lobot_control_main_get_param_node");
    auto param_client = get_param_node->create_client<GetControllers>("/GetControllers");
    param_client->wait_for_service(1s);

    // Get all the controllers to be added
    std::vector<std::pair<std::string, std::string>> controllers = {};
    unsigned int controllerSpinCount = 0;
    if (param_client->service_is_ready())
    {
        while (controllerSpinCount < 6)
        {
            auto req = std::make_shared<parameter_server_interfaces::srv::GetControllers::Request>();
            req->robot = robotName;
            auto resp = param_client->async_send_request(req);
            RCLCPP_INFO(get_param_node->get_logger(), "(main exec) Sending async request... ...");
            auto spin_status = rclcpp::spin_until_future_complete(get_param_node, resp, 10s);
            if (spin_status == rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                auto status = resp.wait_for(1s);
                if (status == std::future_status::ready)
                {
                    auto res = resp.get();
                    auto controllerNames = res->controllers;
                    auto controllerTypes = res->controller_types;
                    if (controllerNames.size() != controllerTypes.size())
                    {
                        RCLCPP_ERROR(get_param_node->get_logger(),
                                     "Number of controller names and types don't match, controller name count: %u, type count: %u",
                                     controllerNames.size(), controllerTypes.size());
                        return -1;
                    }
                    for (size_t i = 0; i < controllerNames.size(); i++)
                    {
                        auto name = controllerNames[i];
                        auto type = controllerTypes[i];
                        auto pair = std::make_pair(name, type);
                        controllers.push_back(pair);
                    }
                    // Break out of the retrying loop if getting controllers is successful
                    break;
                }
                else
                {
                    RCLCPP_ERROR(get_param_node->get_logger(), "GetControllers service failed to execute");
                    controllerSpinCount++;
                }
            }
            else
            {
                RCLCPP_ERROR(get_param_node->get_logger(), "GetControllers service failed to execute (spin failed)");
                controllerSpinCount++;
            }
        }
    }
    else
    {
        RCLCPP_ERROR(get_param_node->get_logger(), "GetControllers service failed to start, check that parameter server is launched");
    }
    for (auto &c : controllers)
    {
        RCLCPP_INFO(get_param_node->get_logger(), "Controller: %s (%s)", c.first.c_str(), c.second.c_str());
    }

    for (auto &pair : controllers)
    {
        cm.load_controller("ros_controllers", pair.second, pair.first);
    }

    // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
    auto future_handle = std::async(std::launch::async, spin, executor);

    // we can either configure each controller individually through its services
    // or we use the controller manager to configure every loaded controller
    unsigned int configureCount = 0;
    while (configureCount < 5)
    {
        // Note that while there is a retrying loop here, often when it runs things will fail because of conflicts in controller states
        // This is probably due to the controllers not taking into account previous state when configuring and etc.
        auto result = cm.configure();
        if (result != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
        {
            std::cerr << '[' << configureCount << ']' << " At least one controller failed to configure, retrying...";
            RCUTILS_LOG_ERROR("[%d] At least one controller failed to configure, retrying...", configureCount);
            configureCount++;
            cm.cleanup();
        }
        else
        {
            RCLCPP_INFO(get_param_node->get_logger(), "Controllers finished configuring");
            break;
        }
        std::this_thread::sleep_for(1s);
    }
    // and activate all controller
    unsigned int activateCount = 0;
    while (activateCount < 6)
    {
        // Note that while there is a retrying loop here, often when it runs things will fail because of conflicts in controller states
        // This is probably due to the controllers not taking into account previous state when activating and etc.
        if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
        {
            std::cerr << '[' << activateCount << ']' << " At least one controller failed to configure, retrying...";
            RCUTILS_LOG_ERROR("[%d] At least one controller failed to activate, retrying...", activateCount);
            activateCount++;
            cm.deactivate();
        }
        else
        {
            RCLCPP_INFO(get_param_node->get_logger(), "Controllers finished activating");
            break;
        }
    }
    hardware_interface::hardware_interface_ret_t ret;
    // Not sure about next line, but was done to make sure things finish before continuing into main loop
    rclcpp::spin_some(get_param_node);
    // Main Loop
    while (active && rclcpp::ok())
    {
        ret = my_robot->read();
        if (ret != hardware_interface::HW_RET_OK)
        {
            fprintf(stderr, "read failed!\n");
        }

        cm.update();

        ret = my_robot->write();
        if (ret != hardware_interface::HW_RET_OK)
        {
            fprintf(stderr, "write failed!\n");
        }
        r.sleep();
    }

    executor->cancel();
}
