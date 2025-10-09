#include "sonia_bt_runner/MissionServer.hpp"

using namespace std::placeholders;

MissionServer::MissionServer()
    : Node("Mission_server")
    {
        const char *ws = std::getenv("SONIA_WS");
        search_directory.assign(ws);
        search_directory.append("/src/sonia_bt_runner/sonia_bt_missions/mission/");

        server_ = rclcpp_action::create_server<MissionControl>(
                    this,
                    "MissionControl",
                    std::bind(&MissionServer::handleGoal, this, _1,_2),
                    std::bind(&MissionServer::handleCancel, this, _1),
                    std::bind(&MissionServer::handleAccept, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Mission Server up running");
    } 
    
    MissionServer::~MissionServer()
    {}
    void MissionServer::init(){
        registerNodes(factory_, this->shared_from_this());    
    }

    void MissionServer::execute(const std::shared_ptr<GoalHandle> goal){
        auto res = std::make_shared<MissionControl::Result>();
        result_=NodeStatus::RUNNING;
        Tracker trac(tree_, goal); 
       
        RCLCPP_INFO(this->get_logger(), "Mission launched");   
        while (!BT::isStatusCompleted(result_))
        { 
            result_ = tree_.tickExactlyOnce(); 
          
            if(goal->is_canceling()){
                res->success = false;
                goal->canceled(res);
                clearFactory("Mission Cancelled");
                return;
            }
            tree_.sleep(std::chrono::milliseconds(_TICK_SLEEP_TIME));
        }

        RCLCPP_INFO(this->get_logger(), "MISSION RESULT: %s", BT::toStr(result_).c_str());
        RCLCPP_INFO(this->get_logger(), "----------------");

        res->success = (result_ == NodeStatus::SUCCESS) ? true: false;
        goal->succeed(res);

        RCLCPP_INFO(this->get_logger(), "Mission completed");
    }

    rclcpp_action::GoalResponse MissionServer::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal){
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with mission : %s", goal->mission.c_str());
        name_ =goal->mission;
        std::string temp_file="";

        using std::filesystem::directory_iterator;
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
            tree_.rootBlackboard().reset();
            tree_ = factory_.createTree(fullFilePath);
            tree_.initialize();
            //tree_.rootNode();
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        catch(const BT::RuntimeError e)
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
    void MissionServer::clearFactory(const std::string log){
        RCLCPP_INFO(this->get_logger(), "%s", log.c_str());
        factory_.clearRegisteredBehaviorTrees();
    }