#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

// clang-format off

// This is the list all the supported options:
//
// NOTE: By default if no mission options is passed, all the missions will be execute.
//
// Mission
//
// --gate              Run the gate mission
// --coin-flip         Run the coin flip mission
// --gate              Run the gate mission
// --trickshot         Run the trickshot mission
// --slalom            Run the slalom mission
// --bin               Run the bin mission
// --torpedo           Run the torpedo mission
// --octogon           Run the octogon mission
// --return-home       Run the return home mission
//
// Path
//
// --rescue            Run the rescue path (default)
// --repair            Run the repair path

// clang-format on

#define PARSE_OPTIONS_OPTION "option"
#define PARSE_OPTIONS_OPTION_PARAMS \
    PARSE_OPTIONS_OPTION, "Option is for customize a given run with the defined available options"
#define PARSE_OPTIONS_OPTION_TYPE std::string

#define PARSE_OPTIONS_HAS_COIN_FLIP "hasCoinFlip"
#define PARSE_OPTIONS_HAS_COIN_FLIP_PARAMS PARSE_OPTIONS_HAS_COIN_FLIP, "Run the coin flip mission"
#define PARSE_OPTIONS_HAS_COIN_FLIP_TYPE bool

#define PARSE_OPTIONS_HAS_GATE "hasGate"
#define PARSE_OPTIONS_HAS_GATE_PARAMS PARSE_OPTIONS_HAS_GATE, "Run the gate mission"
#define PARSE_OPTIONS_HAS_GATE_TYPE bool

#define PARSE_OPTIONS_HAS_TRICKSHOT "hasTrickshot"
#define PARSE_OPTIONS_HAS_TRICKSHOT_PARAMS PARSE_OPTIONS_HAS_TRICKSHOT, "Run the trickshot mission"
#define PARSE_OPTIONS_HAS_TRICKSHOT_TYPE bool

#define PARSE_OPTIONS_HAS_SLALOM "hasSlalom"
#define PARSE_OPTIONS_HAS_SLALOM_PARAMS PARSE_OPTIONS_HAS_SLALOM, "Run the slalom mission"
#define PARSE_OPTIONS_HAS_SLALOM_TYPE bool

#define PARSE_OPTIONS_HAS_BIN "hasBin"
#define PARSE_OPTIONS_HAS_BIN_PARAMS PARSE_OPTIONS_HAS_BIN, "Run the bin mission"
#define PARSE_OPTIONS_HAS_BIN_TYPE bool

#define PARSE_OPTIONS_HAS_TORPEDO "hasTorpedo"
#define PARSE_OPTIONS_HAS_TORPEDO_PARAMS PARSE_OPTIONS_HAS_TORPEDO, "Run the torpedo mission"
#define PARSE_OPTIONS_HAS_TORPEDO_TYPE bool

#define PARSE_OPTIONS_HAS_OCTOGON "hasOctogon"
#define PARSE_OPTIONS_HAS_OCTOGON_PARAMS PARSE_OPTIONS_HAS_OCTOGON, "Run the octogon mission"
#define PARSE_OPTIONS_HAS_OCTOGON_TYPE bool

#define PARSE_OPTIONS_HAS_RETURN_HOME "hasReturnHome"
#define PARSE_OPTIONS_HAS_RETURN_HOME_PARAMS PARSE_OPTIONS_HAS_RETURN_HOME, "Run the return home mission"
#define PARSE_OPTIONS_HAS_RETURN_HOME_TYPE bool

#define PARSE_OPTIONS_HAS_RESCUE "hasRescue"
#define PARSE_OPTIONS_HAS_RESCUE_PARAMS PARSE_OPTIONS_HAS_RESCUE, "Run the rescue path"
#define PARSE_OPTIONS_HAS_RESCUE_TYPE bool

#define PARSE_OPTIONS_HAS_REPAIR "hasRepair"
#define PARSE_OPTIONS_HAS_REPAIR_PARAMS PARSE_OPTIONS_HAS_REPAIR, "Run the repair path"
#define PARSE_OPTIONS_HAS_REPAIR_TYPE bool

namespace init
{
    class ParseOptions : public BT::SimpleActionNode
    {
        public:
        ParseOptions(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~ParseOptions() override = default;

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<PARSE_OPTIONS_OPTION_TYPE>(PARSE_OPTIONS_OPTION_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_COIN_FLIP_TYPE>(PARSE_OPTIONS_HAS_COIN_FLIP_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_GATE_TYPE>(PARSE_OPTIONS_HAS_GATE_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_TRICKSHOT_TYPE>(PARSE_OPTIONS_HAS_TRICKSHOT_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_SLALOM_TYPE>(PARSE_OPTIONS_HAS_SLALOM_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_BIN_TYPE>(PARSE_OPTIONS_HAS_BIN_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_TORPEDO_TYPE>(PARSE_OPTIONS_HAS_TORPEDO_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_OCTOGON_TYPE>(PARSE_OPTIONS_HAS_OCTOGON_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_RETURN_HOME_TYPE>(PARSE_OPTIONS_HAS_RETURN_HOME_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_RESCUE_TYPE>(PARSE_OPTIONS_HAS_RESCUE_PARAMS),
                    BT::OutputPort<PARSE_OPTIONS_HAS_REPAIR_TYPE>(PARSE_OPTIONS_HAS_REPAIR_PARAMS)};
        }

        private:
        BT::NodeStatus tick(BT::TreeNode &node);

        std::shared_ptr<rclcpp::Node> _ros_node;
    };
}  // namespace init
