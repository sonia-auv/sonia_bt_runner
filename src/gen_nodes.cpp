#include <stdlib.h>
#include <string>
#include <fstream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "sonia_bt_runner/SoniaNodes.hpp"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        return EXIT_FAILURE;
    }
    BT::BehaviorTreeFactory factory;
    rclcpp::init(argc,argv);
    auto node= std::make_shared<rclcpp::Node>("sonia_bt_runner");
    registerNodes(factory,node);

    std::string xml_models = BT::writeTreeNodesModelXML(factory, false);
    printf("%s\n", xml_models.c_str());
    std::ofstream myfile;
    myfile.open(argv[1]);
    myfile << xml_models.c_str();
    myfile.close();
    rclcpp::shutdown;
    return EXIT_SUCCESS;
}