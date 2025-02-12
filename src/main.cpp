#include <stdlib.h>
#include <string>
#include <fstream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip> // for std::put_time
#include <sstream> // for std::stringstream

#include "sonia_bt_runner/SoniaNodes.hpp"

using namespace BT;

std::string getTimestamp()
{
    // Get the current time
    auto now = std::chrono::system_clock::now();

    // Convert it to time_t which holds the time in seconds since the epoch
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert to tm struct for local time
    std::tm *now_tm = std::localtime(&now_time_t);

    // Create a stringstream to format the timestamp
    std::stringstream ss;
    ss << std::put_time(now_tm, "%Y%m%d_%H%M%S"); // Format: YYYYMMDD_HHMMSS

    return ss.str();
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        return EXIT_FAILURE;
    }
    printf("LEN argc: %d\n", argc);
    BT::BehaviorTreeFactory factory;
    registerNodes(factory);

    std::string name = argv[1];
    

    std::string search_directory = "./";
    printf("L111");

    using std::filesystem::directory_iterator;
    printf("L2222");
    for (auto const &entry : directory_iterator(search_directory))
    {

        if (entry.path().extension() == ".xml")
        {
            factory.registerBehaviorTreeFromFile(entry.path().string());
            std::cout << "file"<<entry.path()<<std::endl;
        }
    }

    // Get the directory containing the executable
    const char *homeDir = getenv("HOME");
    // std::filesystem::path executablePath = std::string(homeDir) + "/ros2_sonia_ws/src/sonia_BehaviorTree/src/missions";

    // std::filesystem::path relativeFilePath = file_path;

    std::filesystem::path fullFilePath = name;

    rclcpp::init(argc,argv);
    auto node= std::make_shared<rclcpp::Node>("sonia_bt_runner");

    auto tree = factory.createTree(fullFilePath);
    BT::Groot2Publisher publisher(tree, 5555);
    publisher.setEnabled(true);

    std::string timestamp = getTimestamp();

    std::string filename = "runs/RUN_" + timestamp + ".txt";

    std::ofstream file(filename);

    // Helper function to print the tree.
    BT::printTreeRecursively(tree.rootNode(), file);

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
        file << uid << " -> " << name << "\n";
    }
    NodeStatus result = NodeStatus::RUNNING;

    while (rclcpp::ok() && result != NodeStatus::SUCCESS && result != NodeStatus::FAILURE)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        result = tree.tickOnce();
        rclcpp::spin_some(node);
    }

    std::cout << "MISSION RESULT: " << result << std::endl;
    file << "MISSION RESULT: " << result << "\n";
    std::cout << "----------------" << std::endl;
    file << "----------------" << "\n";
    // print all the statistics
    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        const auto &stats = observer.getStatistics(uid);

        std::cout << "[" << name
                  << "] \tT/S/F:  " << stats.transitions_count
                  << "/" << stats.success_count
                  << "/" << stats.failure_count
                  << std::endl;

        file << "[" << name
             << "] \tT/S/F:  " << stats.transitions_count
             << "/" << stats.success_count
             << "/" << stats.failure_count
             << "\n";
    }
    file.close();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}