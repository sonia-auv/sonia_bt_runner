#include <stdlib.h>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "sonia_bt_runner/MissionServer.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto server= std::make_shared<MissionServer>();
    server->init();
    
    /*rclcpp::Rate r(10);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server);
    executor.spin();*/
    rclcpp::spin(server);
    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}