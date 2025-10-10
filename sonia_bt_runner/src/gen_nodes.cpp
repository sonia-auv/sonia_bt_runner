#include <stdlib.h>
#include <string>
#include <fstream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "sonia_bt_runner/SoniaNodes.hpp"

int main(int argc, char *argv[])
{
    const char *ws = std::getenv("SONIA_WS");

    if(ws == NULL){
        std::cerr << "env var SONIA_WS not set" << std::endl;
        return EXIT_FAILURE;
    }

    std::string search_directory(ws);
    search_directory.append("/src/sonia_bt_runner/sonia_bt_missions/SoniaNodes.xml");

    BT::BehaviorTreeFactory factory;
    registerNodes(factory, nullptr);

    std::string xml_models = BT::writeTreeNodesModelXML(factory, false);
    printf("%s\n", xml_models.c_str());
    std::ofstream myfile;
    myfile.open(search_directory);
    myfile << xml_models.c_str();
    myfile.close();
    return EXIT_SUCCESS;
}