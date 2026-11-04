#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "sonia_bt_runner/MissionServer.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<MissionServer>();
    server->init();
    rclcpp::spin(server);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}