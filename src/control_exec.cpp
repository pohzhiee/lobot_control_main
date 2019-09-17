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
#include <robot_hardware_interface/robot_hardware.hpp>
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
    std::this_thread::sleep_for(2s);

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
    if (param_client->service_is_ready())
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
                for(size_t i =0;i<controllerNames.size();i++){
                    auto name = controllerNames[i];
                    auto type = controllerTypes[i];
                    auto pair = std::make_pair(name, type);
                    controllers.push_back(pair);
                }
            }
            else
            {
                RCLCPP_ERROR(get_param_node->get_logger(), "GetControllers service failed to execute");
            }
        }
        else
        {
            RCLCPP_ERROR(get_param_node->get_logger(), "GetControllers service failed to execute (spin failed)");
        }
    }
    else
    {
        RCLCPP_ERROR(get_param_node->get_logger(), "GetControllers service failed to start, check that parameter server is launched");
    }
    for(auto &c : controllers){
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
    if (cm.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    {
        RCUTILS_LOG_ERROR("at least one controller failed to configure");
        return -1;
    }
    else{
        fprintf(stderr, "Controllers finished configuring\n");
        RCLCPP_INFO(get_param_node->get_logger(), "Controllers finished configuring");
    }
    // and activate all controller
    if (cm.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    {
        RCUTILS_LOG_ERROR("at least one controller failed to activate");
        return -1;
    }
    else{
        RCLCPP_INFO(get_param_node->get_logger(), "Controllers finished activating");
        fprintf(stderr, "Controllers finished activating\n");
    }
    // main loop
    hardware_interface::hardware_interface_ret_t ret;
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
