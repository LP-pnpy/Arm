#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h> 
#include <thread>  
#include <memory>  

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    
    // 使用单线程执行器并在后台线程 spin，防止阻塞主线程
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){ executor.spin(); });

    // 创建 MoveGroupInterface，指定规划组为 "arm"
    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    
    //setMaxVelocituScalingFactor -> setMaxVelocityScalingFactor
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // 设置起始状态为当前状态
    arm.setStartStateToCurrentState();
    
    // 设置目标为预定义的命名姿态 "pose_1"
    arm.setNamedTarget("pose_1");

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success1){
        RCLCPP_INFO(node->get_logger(), "规划成功，正在执行...");
        arm.execute(plan1);
    } else {
        RCLCPP_ERROR(node->get_logger(), "规划失败！请检查目标姿态是否可达或有碰撞。");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}