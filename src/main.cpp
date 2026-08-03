//
// Created by da on 03/08/23.
//
#include "System.h"
#include <memory>
#include <string>
#include <vector>
using namespace std;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    const string setting = (args.size() > 1) ? args[1] : string();
    auto sys = std::make_shared<System>(setting);
    sys->runRos();
    rclcpp::shutdown();
    return 0;
}
