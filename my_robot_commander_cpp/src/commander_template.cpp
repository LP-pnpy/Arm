#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h> 
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <thread>  
#include <memory>  
#include <tf2/LinearMath/Quaternion.h>           // tf2 头文件
#include <geometry_msgs/msg/pose_stamped.hpp>    // PoseStamped 头文件
#include <geometry_msgs/msg/pose.hpp>            // Pose 头文件
#include <moveit_msgs/msg/robot_trajectory.hpp>  // 轨迹消息头文件

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders; 

class Commander{
public:
    Commander(std::shared_ptr<rclcpp::Node> node){
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "grapper");

        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open", 10, 
            [this](const Bool::SharedPtr msg) { 
                this->OpenGripperCallback(*msg); 
            }
        );

        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command", 10, 
            [this](const FloatArray::SharedPtr msg) { 
                this->JointCmdCallback(*msg); 
            }
        );

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command", 10,
            [this](const PoseCmd::SharedPtr msg){
                this->poseCmdCallback(*msg);
            }
        );
    }

    void goToNamedTarget(const std::string &name){
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints){
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }
    
    void goToPoseTarget(double x, double y, double z, 
                        double roll, double pitch, double yaw, bool cartesian_path=false){
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if(!cartesian_path){
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double eef_step = 0.01;         // 步长 1cm
            double jump_threshold = 0.0;    // 禁用跳跃检测（对于单一小位移，设为0即可）
            
            double fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            // 5. 判断结果并执行
            if (fraction == 1.0) {
                RCLCPP_INFO(node_->get_logger(), "笛卡尔路径规划成功！正在执行...");
                arm_->execute(trajectory);
            } else {
                RCLCPP_WARN(node_->get_logger(), "笛卡尔路径规划失败，完成度：%.2f%%", fraction * 100.0);
                // 如果 fraction < 1.0，通常是因为向下移动会碰撞到底座、地面或超出工作空间
                }
            }
        }
    void openGripper(){
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("open");
        planAndExecute(gripper_);
    }

    void closeGripper(){
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("close");
        planAndExecute(gripper_);
    }

private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface){
        MoveGroupInterface::Plan plan;
        bool success = {interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS};

        if(success){interface->execute(plan);}
    }

    void OpenGripperCallback(const Bool &msg){
        if(msg.data){
            openGripper();
        }else{
            closeGripper();
        }
    }

    void JointCmdCallback(const FloatArray &msg){
        auto joints = msg.data;
        if(joints.size() == 6){
            goToJointTarget(joints);
        }
    }

    void poseCmdCallback(const PoseCmd &msg){
        goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}