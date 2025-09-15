#include "sonia_bt_runner/MissionServer.hpp"
#include "sonia_bt_runner/Tracker.hpp"
#include <functional>
using namespace std::placeholders;

MissionServer::MissionServer()
    : Node("Mission_server")
    {
        const char *ws = std::getenv("SONIA_WS");
        search_directory.assign(ws);
        search_directory.append("/src/sonia_bt_missions/mission/");

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
    void MissionServer::execute(const std::shared_ptr<GoalHandle> goal)
    {
        auto res = std::make_shared<MissionControl::Result>();

        NodeStatus result_=NodeStatus::RUNNING;
        Tracker trac(tree_, goal); 
       
        RCLCPP_INFO(this->get_logger(), "Begin tree");     
        while (!BT::isStatusCompleted(result_))
        {
            result_ = tree_.tickOnce();
            tree_.sleep(std::chrono::milliseconds(66));         
        }
        RCLCPP_INFO(this->get_logger(), "MISSION RESULT: %s", BT::toStr(result_));
        RCLCPP_INFO(this->get_logger(), "----------------");
        
        res->success=true;
        goal->succeed(res);

        RCLCPP_INFO(this->get_logger(), "completed the tree");
    }

    rclcpp_action::GoalResponse MissionServer::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request with mission %s", goal->mission.c_str());
        (void)uuid;

        name_ =goal->mission;

        using std::filesystem::directory_iterator;
    
        for (auto const &entry : directory_iterator(search_directory))
        {
            if (entry.path().extension() == ".xml")
            {
                factory_.registerBehaviorTreeFromFile(entry.path().string());
                std::cout << "file: "<<entry.path()<<std::endl;
            }
        }
        try
        {
            std::filesystem::path fullFilePath(name_);
            tree_ = factory_.createTree(fullFilePath);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "Loaded mission Error: ", e.what());
            factory_.clearRegisteredBehaviorTrees();
            tree_.~Tree();
            return rclcpp_action::GoalResponse::REJECT;
        }
          
    }
    rclcpp_action::CancelResponse MissionServer::handleCancel(const std::shared_ptr<GoalHandle> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void MissionServer::handleAccept(const std::shared_ptr<GoalHandle> goal_handle){
        std::thread{std::bind(&MissionServer::execute, this, _1), goal_handle}.detach();
    }