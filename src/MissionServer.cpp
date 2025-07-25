#include "sonia_bt_runner/MissionServer.hpp"
#include <functional>
using namespace std::placeholders;

MissionServer::MissionServer()
    : Node("Mission_server")
    {
        server_ = rclcpp_action::create_server<MissionControl>(
                    this,
                    "MissionControl",
                    std::bind(&MissionServer::handleGoal, this, _1,_2),
                    std::bind(&MissionServer::handleCancel, this, _1),
                    std::bind(&MissionServer::handleAccept, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Action Server up running");
    } 
    
    MissionServer::~MissionServer()
    {}
    void MissionServer::init(){
        registerNodes(factory_, this->shared_from_this());
        
    }
    void MissionServer::execute(const std::shared_ptr<GoalHandle> goal)
    {    
         /*std::map<uint16_t, std::string> ordered_UID_to_path;
        for (const auto &[name, uid] : obs.pathToUID())
        {
            ordered_UID_to_path[uid] = name;
        }

        for (const auto &[uid, name] : ordered_UID_to_path)
        {
            std::cout << uid << " -> " << name << std::endl;
        }*/
        
        auto res = std::make_shared<MissionControl::Result>();
        

        BT::Groot2Publisher publisher(tree_, 5555);
        publisher.setEnabled(true);

        //Logging log(tree_);
        RCLCPP_INFO(this->get_logger(), "Begin tree");
        NodeStatus result_=NodeStatus::RUNNING;
        while (result_ != NodeStatus::SUCCESS && result_ != NodeStatus::FAILURE)
        {
            result_ = tree_.tickOnce();
            std::cout << "ticking..... " << result_ << std::endl;

        }
        std::cout << "MISSION RESULT: " << result_ << std::endl;
    
        std::cout << "----------------" << std::endl;
    
        res->success=true;
        goal->succeed(res);

        
        RCLCPP_INFO(this->get_logger(), "completed the tree");

    }
    rclcpp_action::GoalResponse MissionServer::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MissionControl::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request with mission %", goal->mission);
        (void)uuid;

        std::string search_directory = "/home/sonia2/ros2_sonia_ws/src/sonia_bt_missions/mission/";
        //std::string search_directory = "/home/sawali/ros_sonia_ws2/src/sonia_bt_missions/mission/";
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
  
        std::filesystem::path fullFilePath(name_);
        tree_ = factory_.createTree(fullFilePath);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse MissionServer::handleCancel(const std::shared_ptr<GoalHandle> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void MissionServer::handleAccept(const std::shared_ptr<GoalHandle> goal_handle){
        std::thread{std::bind(&MissionServer::execute, this, _1), goal_handle}.detach();
    }