#include <stdlib.h>
#include <string>
#include <fstream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "ros/ros.h"

#include "sonia_bt_runner/SoniaNodes.hpp"

using namespace BT;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        return EXIT_FAILURE;
    }
    printf("LEN argc: %d\n", argc);
    BT::BehaviorTreeFactory factory;
    registerNodes(factory);
    
    // factory.registerNodeType<DropperActionNode>("DropperActionNode");

    std::string name = argv[1];
    // std::string file_path = name + ".xml";

    // Get the directory containing the executable
    const char *homeDir = getenv("HOME");
    // std::filesystem::path executablePath = std::string(homeDir) + "/ros2_sonia_ws/src/sonia_BehaviorTree/src/missions";

    // std::filesystem::path relativeFilePath = file_path;

    std::filesystem::path fullFilePath = name;

    ros::init(argc, argv, "sonia_bt_runner");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));


    auto tree = factory.createTreeFromFile(fullFilePath);
    BT::Groot2Publisher publisher(tree, 5555);
    publisher.setEnabled(true);


    // Helper function to print the tree.
    BT::printTreeRecursively(tree.rootNode());

    // The purpose of the observer is to save some statistics about the number of times
    // a certain node returns SUCCESS or FAILURE.
    // This is particularly useful to create unit tests and to check if
    // a certain set of transitions happened as expected
    BT::TreeObserver observer(tree);

    // Print the unique ID and the corresponding human readable path
    // Path is also expected to be unique.
    std::map<uint16_t, std::string> ordered_UID_to_path;
    for (const auto &[name, uid] : observer.pathToUID())
    {
        ordered_UID_to_path[uid] = name;
    }

    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        std::cout << uid << " -> " << name << std::endl;
    }
    NodeStatus result = NodeStatus::RUNNING;
    ros::Rate r(20);
    while(ros::ok() && result != NodeStatus::SUCCESS && result != NodeStatus::FAILURE)
    {
        // r.sleep();
        result = tree.tickOnce();
        ros::spinOnce();
        if (result == NodeStatus::SUCCESS)
            std::cout << "MISSION RESULT: " << result << std::endl;
    }


    // NodeStatus result = tree.tickWhileRunning();

    std::cout << "MISSION RESULT: " << result << std::endl;
    std::cout << "----------------" << std::endl;
    // print all the statistics
    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        const auto &stats = observer.getStatistics(uid);

        std::cout << "[" << name
                  << "] \tT/S/F:  " << stats.transitions_count
                  << "/" << stats.success_count
                  << "/" << stats.failure_count
                  << std::endl;
    }
    return EXIT_SUCCESS;
}