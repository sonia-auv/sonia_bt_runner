#include "sonia_bt_runner/init/ParseOptions.hpp"

#include <functional>
#include <tuple>

using std::placeholders::_1;

namespace init
{
    ParseOptions::ParseOptions(const std::string &name, const BT::NodeConfig &config,
                               std::shared_ptr<rclcpp::Node> node)
        : BT::SimpleActionNode(name, std::bind(&ParseOptions::tick, this, _1), config), _ros_node(node)
    {}

    BT::NodeStatus ParseOptions::tick(BT::TreeNode &node)
    {
        (void)node;

#define OPTION_FLAGS_NONE (0)

// Mission options
#define OPTION_FLAGS_COIN_FLIP (1 << 0)
#define OPTION_FLAGS_GATE (1 << 1)
#define OPTION_FLAGS_TRICKSHOT (1 << 2)
#define OPTION_FLAGS_SLALOM (1 << 3)
#define OPTION_FLAGS_BIN (1 << 4)
#define OPTION_FLAGS_TORPEDO (1 << 5)
#define OPTION_FLAGS_OCTOGON (1 << 6)
#define OPTION_FLAGS_RETURN_HOME (1 << 7)

#define OPTION_FLAGS_MISSION_ALL ((1 << 7) - 1)

// Path options
#define OPTION_FLAGS_RESCUE (1 << 8)
#define OPTION_FLAGS_REPAIR (1 << 9)

        int option_flags = OPTION_FLAGS_NONE;
        constexpr int path_options_mask = OPTION_FLAGS_RESCUE | OPTION_FLAGS_REPAIR;
        std::tuple<const char *, int, std::function<void(bool)>> options[] = {
            {  "--coin-flip",   OPTION_FLAGS_COIN_FLIP,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_COIN_FLIP_TYPE>(PARSE_OPTIONS_HAS_COIN_FLIP, value); }},
            {       "--gate",        OPTION_FLAGS_GATE,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_GATE_TYPE>(PARSE_OPTIONS_HAS_GATE, value); }          },
            {  "--trickshot",   OPTION_FLAGS_TRICKSHOT,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_TRICKSHOT_TYPE>(PARSE_OPTIONS_HAS_TRICKSHOT, value); }},
            {     "--slalom",      OPTION_FLAGS_SLALOM,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_SLALOM_TYPE>(PARSE_OPTIONS_HAS_SLALOM, value); }      },
            {        "--bin",         OPTION_FLAGS_BIN,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_BIN_TYPE>(PARSE_OPTIONS_HAS_BIN, value); }            },
            {    "--torpedo",     OPTION_FLAGS_TORPEDO,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_TORPEDO_TYPE>(PARSE_OPTIONS_HAS_TORPEDO, value); }    },
            {    "--octogon",     OPTION_FLAGS_OCTOGON,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_OCTOGON_TYPE>(PARSE_OPTIONS_HAS_OCTOGON, value); }    },
            {"--return-home", OPTION_FLAGS_RETURN_HOME,
             [this](bool value) {
             setOutput<PARSE_OPTIONS_HAS_RETURN_HOME_TYPE>(PARSE_OPTIONS_HAS_RETURN_HOME, value);
             }                                                                                                      },
            {     "--rescue",      OPTION_FLAGS_RESCUE,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_RESCUE_TYPE>(PARSE_OPTIONS_HAS_RESCUE, value); }      },
            {     "--repair",      OPTION_FLAGS_REPAIR,
             [this](bool value) { setOutput<PARSE_OPTIONS_HAS_REPAIR_TYPE>(PARSE_OPTIONS_HAS_REPAIR, value); }      }
        };
        const auto option_input = getInput<PARSE_OPTIONS_OPTION_TYPE>(PARSE_OPTIONS_OPTION).value();
        size_t begin = 0;
        size_t end = option_input.find(" ");

        while (end != std::string::npos)
        {
            auto sub = option_input.substr(begin, end - begin);

            for (const auto &option : options)
            {
                if (!strcmp(sub.c_str(), std::get<0>(option)))
                {
                    option_flags |= std::get<1>(option);
                }
            }

            begin += end + 1;
            end = option_input.find(" ", begin);
        }

        if (!(option_flags & path_options_mask))
        {
            option_flags |= OPTION_FLAGS_RESCUE /* default */;
        }

        if ((option_flags & ~path_options_mask) == OPTION_FLAGS_NONE)
        {
            option_flags |= OPTION_FLAGS_MISSION_ALL;
        }

        for (const auto &option : options)
        {
            std::get<2>(option)((option_flags & std::get<1>(option)) != 0);
        }

#undef OPTION_FLAGS_NONE
#undef OPTION_FLAGS_COIN_FLIP
#undef OPTION_FLAGS_GATE
#undef OPTION_FLAGS_TRICKSHOT
#undef OPTION_FLAGS_SLALOM
#undef OPTION_FLAGS_BIN
#undef OPTION_FLAGS_TORPEDO
#undef OPTION_FLAGS_OCTOGON
#undef OPTION_FLAGS_RETURN_HOME
#undef OPTION_FLAGS_MISSION_ALL
#undef OPTION_FLAGS_RESCUE
#undef OPTION_FLAGS_REPAIR

        return BT::NodeStatus::SUCCESS;
    }
}  // namespace init
