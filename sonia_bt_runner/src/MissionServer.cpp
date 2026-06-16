#include "sonia_bt_runner/MissionServer.hpp"

using std::filesystem::directory_iterator;
using namespace std::placeholders;

MissionServer::MissionServer()
    : Node("Mission_server")
    {
        //set mission path
        const char *ws = std::getenv("SONIA_WS");
        _search_directory.assign(ws);
        _search_directory.append("/src/sonia_bt_runner/sonia_bt_missions/mission/");

        _pub_status = this->create_publisher<std_msgs::msg::String>("/mission_server/status_report",1);
        _pub_node_status = this->create_publisher<sonia_common_ros2::msg::NodeStatus>("/system_monitor/node_status",1);
        _fetch_missions_srv = this->create_service<sonia_common_ros2::srv::MissionListService>("/mission_server/mission_list", std::bind(&MissionServer::grabMissionList, this, _1, _2));

        _timer_node_status = this->create_wall_timer(500ms, std::bind(&MissionServer::publishStatus, this));

        _server = rclcpp_action::create_server<MissionControl>(
                    this,
                    "MissionControl",
                    std::bind(&MissionServer::handleGoal, this, _1,_2),
                    std::bind(&MissionServer::handleCancel, this, _1),
                    std::bind(&MissionServer::handleAccept, this, _1));
        
        _node_status.node_name = this->get_name();
        _node_status.quality = sonia_common_ros2::msg::NodeStatus::Q_OK;
        _node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_INITIALIZING;
        
        RCLCPP_INFO(this->get_logger(), "Mission Server up running");
    }
    
    void MissionServer::init(){
        registerNodes(_factory, this->shared_from_this());
        _node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_IDLE;    
    }

	std::shared_ptr<MissionControl::Result> MissionServer::launchMission(const std::shared_ptr<GoalHandle> goal, std_msgs::msg::String &rep) {
		auto res = std::make_shared<MissionControl::Result>();
		auto thread_handler = [&]() -> void {
			RCLCPP_INFO(this->get_logger(), "Mission launched"); 

			rep.data= "Mission launched....";
			_pub_status->publish(rep);

			while (!BT::isStatusCompleted(_result))
			{
				try {
					_result = _tree.tickExactlyOnce();
				} catch (const std::exception& e) {
					RCLCPP_INFO(this->get_logger(), "An exception occured during the execution of the mission: %s", e.what());
					res->success = false;
					_result = NodeStatus::FAILURE;

					break;
				}

				if(goal->is_canceling()) {
					res->success = false;
					goal->canceled(res);
					clearFactory("Mission Cancelled");
					return;
				}

				_tree.sleep(std::chrono::milliseconds(_TICK_SLEEP_TIME));
			}

			RCLCPP_INFO(this->get_logger(), "MISSION RESULT: %s", BT::toStr(_result).c_str());
			RCLCPP_INFO(this->get_logger(), "----------------");
		};

		std::thread mission_thread(thread_handler);

		mission_thread.join();

		return res;
    }

    void MissionServer::execute(const std::shared_ptr<GoalHandle> goal){
        std_msgs::msg::String rep;
        _result=NodeStatus::RUNNING;
        Tracker trac(_tree, goal);
		auto res = launchMission(goal, rep);

        res->success = (_result == NodeStatus::SUCCESS);
        goal->succeed(res);

        RCLCPP_INFO(this->get_logger(), "Mission completed");
        rep.data= "Mission completed....";
        _pub_status->publish(rep);

        _node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_IDLE;
    }

    rclcpp_action::GoalResponse MissionServer::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal){
        (void)uuid;
        _node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_RUNNING;
        RCLCPP_INFO(this->get_logger(), "Received goal request with mission : %s", goal->mission.c_str());
        _name =goal->mission;
        std::string temp_file;

        try
        {
            for (auto const &entry : directory_iterator(_search_directory))
            {
                if (entry.path().extension() == ".xml")
                {
                    temp_file = entry.path().string();
                    _factory.registerBehaviorTreeFromFile(temp_file);
                    std::cout << "file: "<<entry.path()<<std::endl;
                }
            }
            
            std::filesystem::path fullFilePath(_name);
            _tree = _factory.createTree(fullFilePath);
            _tree.initialize();
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        catch(const BT::RuntimeError& e)
        {
            std::string err= e.what();
            clearFactory("File : "+temp_file+" contains : "+ err);
            return rclcpp_action::GoalResponse::REJECT;
        }
        catch(const std::exception& e)
        {
            std::string err= e.what();
            clearFactory("Loaded mission Error : "+ err);
            return rclcpp_action::GoalResponse::REJECT;
        }   
    }
    
    rclcpp_action::CancelResponse MissionServer::handleCancel(const std::shared_ptr<GoalHandle> goal_handle){
        (void)goal_handle;
        if (_result != NodeStatus::RUNNING){
            RCLCPP_INFO(this->get_logger(), "Cancel request rejected: NO MISSION RUNNING");
            return rclcpp_action::CancelResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Cancel request accepted: MISSION CANCELLED");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MissionServer::handleAccept(const std::shared_ptr<GoalHandle> goal_handle){
        std::thread{std::bind(&MissionServer::execute, this, _1), goal_handle}.detach();
    }

    void MissionServer::generateMissionList() {
        _mission_list.clear();
        for (auto const &entry : directory_iterator(_search_directory))
        {
            tinyxml2::XMLDocument document;
            if (entry.path().extension() == ".xml")
            {
                document.LoadFile(entry.path().string().c_str());
                tinyxml2::XMLElement* root_element = document.RootElement();

                for(tinyxml2::XMLElement* element = root_element->FirstChildElement("BehaviorTree"); element!= nullptr; element = element->NextSiblingElement("BehaviorTree")){
                    const char* id = element->Attribute("ID");
                    _mission_list.push_back(id);
                }
            }
        }     
    }

    void MissionServer::clearFactory(const std::string log){
        std_msgs::msg::String rep;
        rep.data=log;
        _pub_status->publish(rep);
        RCLCPP_INFO(this->get_logger(), "%s", log.c_str());
        _factory.clearRegisteredBehaviorTrees();
        _node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_IDLE;
    }
    void MissionServer::grabMissionList(const std::shared_ptr<sonia_common_ros2::srv::MissionListService::Request> request, std::shared_ptr<sonia_common_ros2::srv::MissionListService::Response> response){
        (void)request;
        generateMissionList();
        response->missions = _mission_list;
    }
    void MissionServer::publishStatus(){
        _node_status.stamp = this->now();
        _pub_node_status->publish(_node_status);
    }
