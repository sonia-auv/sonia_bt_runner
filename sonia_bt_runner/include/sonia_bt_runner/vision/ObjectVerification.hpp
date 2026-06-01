#pragma once

#include <optional>
#include "behaviortree_cpp/behavior_tree.h"

std::optional<BT::NodeStatus>
verifyObject(std::string object);