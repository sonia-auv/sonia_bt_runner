#include "sonia_bt_runner/MissionServer.hpp"

using std::filesystem::directory_iterator;
using namespace std::placeholders;

MissionServer::MissionServer()
    : Node("Mission_server")
    {
        const char *ws = std::getenv("SONIA_WS");
        search_directory.assign(ws);
        search_directory.append("/src/sonia_bt_runner/sonia_bt_missions/mission/");

        pub_status_ = this->create_publisher<std_msgs::msg::String>("/mission_server/status_report",1);
        pub_node_status_ = this->create_publisher<sonia_common_ros2::msg::NodeStatus>("/system_monitor/node_status",1);
        fetch_missions_srv_ = this->create_service<sonia_common_ros2::srv::MissionListService>("/mission_server/mission_list", std::bind(&MissionServer::grabMissionList, this, _1, _2));

        _timerNodeStatus = this->create_wall_timer(500ms, std::bind(&MissionServer::publishStatus, this));
        server_ = rclcpp_action::create_server<MissionControl>(
                    this,
                    "MissionControl",
                    std::bind(&MissionServer::handleGoal, this, _1,_2),
                    std::bind(&MissionServer::handleCancel, this, _1),
                    std::bind(&MissionServer::handleAccept, this, _1));

        node_status.quality = sonia_common_ros2::msg::NodeStatus::LVL_OK;
        node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_IDLE;
        
        RCLCPP_INFO(this->get_logger(), "Mission Server up running");
    }
    
    void MissionServer::init(){
        registerNodes(factory_, this->shared_from_this());    
    }

    void MissionServer::execute(const std::shared_ptr<GoalHandle> goal){
        auto res = std::make_shared<MissionControl::Result>();
        std_msgs::msg::String rep;
        result_=NodeStatus::RUNNING;
        Tracker trac(tree_, goal); 
       
        RCLCPP_INFO(this->get_logger(), "Mission launched"); 
        rep.data= "Mission launched....";
        pub_status_->publish(rep);  
        
        while (!BT::isStatusCompleted(result_))
        { 
            result_ = tree_.tickExactlyOnce(); 
          
            if(goal->is_canceling()){
                res->success = false;
                goal->canceled(res);
                node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_IDLE;
                clearFactory("Mission Cancelled");
                return;
            }
            tree_.sleep(std::chrono::milliseconds(_TICK_SLEEP_TIME));
        }

        RCLCPP_INFO(this->get_logger(), "MISSION RESULT: %s", BT::toStr(result_).c_str());
        RCLCPP_INFO(this->get_logger(), "----------------");

        res->success = (result_ == NodeStatus::SUCCESS);
        goal->succeed(res);

        RCLCPP_INFO(this->get_logger(), "Mission completed");
        rep.data= "Mission completed....";
        pub_status_->publish(rep);

        node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_IDLE;
    }

    rclcpp_action::GoalResponse MissionServer::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal){
        (void)uuid;
        node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_RUNNING;
        RCLCPP_INFO(this->get_logger(), "Received goal request with mission : %s", goal->mission.c_str());
        name_ =goal->mission;
        std::string temp_file;

        try
        {
            for (auto const &entry : directory_iterator(search_directory))
            {
                if (entry.path().extension() == ".xml")
                {
                    temp_file = entry.path().string();
                    factory_.registerBehaviorTreeFromFile(temp_file);
                    std::cout << "file: "<<entry.path()<<std::endl;
                }
            }
            
            std::filesystem::path fullFilePath(name_);
            tree_ = factory_.createTree(fullFilePath);
            tree_.initialize();
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
        if (result_ != NodeStatus::RUNNING){
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
        mission_list.clear();
        for (auto const &entry : directory_iterator(search_directory))
        {
            tinyxml2::XMLDocument document;
            if (entry.path().extension() == ".xml")
            {
                document.LoadFile(entry.path().string().c_str());
                tinyxml2::XMLElement* root_element = document.RootElement();

                for(tinyxml2::XMLElement* element = root_element->FirstChildElement("BehaviorTree"); element!= nullptr; element = element->NextSiblingElement("BehaviorTree")){
                    const char* id = element->Attribute("ID");
                    mission_list.push_back(id);
                }
            }
        }     
    }

    void MissionServer::clearFactory(const std::string log){
        std_msgs::msg::String rep;
        rep.data=log;
        pub_status_->publish(rep);
        RCLCPP_INFO(this->get_logger(), "%s", log.c_str());
        factory_.clearRegisteredBehaviorTrees();
    }
    void MissionServer::grabMissionList(const std::shared_ptr<sonia_common_ros2::srv::MissionListService::Request> request, std::shared_ptr<sonia_common_ros2::srv::MissionListService::Response> response){
        (void)request;
        generateMissionList();
        response->missions = mission_list;
    }
    void MissionServer::publishStatus(){
        node_status.node_name = this->get_name();
        node_status.stamp = this->get_clock().get()->now();
        pub_node_status_->publish(node_status);
    }