#include <mutex>
#include <cassert>
#include <sstream>
#include <optional>

// #include "ObjectVerification.hpp"
#include "yaml-cpp/yaml.h"
#include "behaviortree_cpp/behavior_tree.h"

static std::optional<std::vector<std::string>> definedObjects = {};
// TODO: Maybe use a read-write lock it's better
static std::mutex definedObjectsGuard = {};

static std::vector<std::string> parseConfig()
{
    auto ws = getenv("SONIA_WS");

    assert(ws && "SONIA_WS should be defined");

    std::stringstream st;

    st << ws << "/src/proc_vision_ros2/models/robosub-2025-v0/data.yaml";

    auto configPath = st.str();
    std::vector<std::string> definedObjectsValue;
    auto configNode = YAML::LoadFile(configPath);
    auto configNodeNamesMap = configNode["names"].as<std::map<int, std::string>>();

    definedObjectsValue.reserve(configNodeNamesMap.size());

    for (auto &[_, nameValue] : configNodeNamesMap) {
	    definedObjectsValue.push_back(nameValue);
    }

    return definedObjectsValue;
}

std::optional<BT::NodeStatus>
verifyObject(std::string object)
{
    definedObjectsGuard.lock();

    if (!definedObjects.has_value()) {
	    definedObjects = {parseConfig()};
    }

    for (auto &definedObject : definedObjects.value()) {
	    if (definedObject == object) {
		    return {};
	    }
    }

    definedObjectsGuard.unlock();


    return BT::NodeStatus::FAILURE;
}
