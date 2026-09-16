#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h> 
#include <thread>  
#include <memory>  
#include <tf2/LinearMath/Quaternion.h>           // tf2 头文件
#include <geometry_msgs/msg/pose_stamped.hpp>    // PoseStamped 头文件
#include <geometry_msgs/msg/pose.hpp>            // Pose 头文件
#include <moveit_msgs/msg/robot_trajectory.hpp>  // 轨迹消息头文件

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

    /*******************************Goal Name**********************************/

    // // 设置起始状态为当前状态
    // arm.setStartStateToCurrentState();
    
    // // 设置目标为预定义的命名姿态 "pose_1"
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if(success1){
    //     RCLCPP_INFO(node->get_logger(), "plan1规划成功,正在执行...");
    //     arm.execute(plan1);
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "plan1规划失败!请检查目标姿态是否可达或有碰撞。");
    // }

    // // 设置起始状态为当前状态
    // arm.setStartStateToCurrentState();
    
    // // 设置目标为预定义的命名姿态 "home"
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // if(success2){
    //     RCLCPP_INFO(node->get_logger(), "plan2规划成功,正在执行...");
    //     arm.execute(plan2);
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "plan2规划失败!请检查目标姿态是否可达或有碰撞。");
    // }

    /*-------------------------------------------------------------------------*/
    //  joint Goal

    // std::vector<double> joints = { 1.5, 0.5, 0.0, 0.0, 0.0, 0.0};
    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if(success1){
    //     RCLCPP_INFO(node->get_logger(), "plan1规划成功,正在执行...");
    //     arm.execute(plan1);
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "plan1规划失败!请检查目标姿态是否可达或有碰撞。");
    // }

    /*-------------------------------------------------------------------------*/

    // Pose Goal

    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success1){
        RCLCPP_INFO(node->get_logger(), "plan1规划成功,正在执行...");
        arm.execute(plan1);
    } else {
        RCLCPP_ERROR(node->get_logger(), "plan1规划失败!请检查目标姿态是否可达或有碰撞。");
    }

    //  Cartian Path

    // 1. 获取当前位姿
    geometry_msgs::msg::Pose pose1  = arm.getCurrentPose().pose;
    
    // 2. 沿 Z 轴向下移动 0.2 米 (注意：这是在世界坐标系下移动)
    pose1 .position.z -= 0.2; 

    // 3. 放入路径点容器
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose1);

    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.y += 0.2;
    waypoints.push_back(pose2);

    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.y -= 0.2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);
    

    // 4. 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;         // 步长 1cm
    double jump_threshold = 0.0;    // 禁用跳跃检测（对于单一小位移，设为0即可）
    
    double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // 5. 判断结果并执行
    if (fraction == 1.0) {
        RCLCPP_INFO(node->get_logger(), "笛卡尔路径规划成功！正在执行...");
        arm.execute(trajectory);
    } else {
        RCLCPP_WARN(node->get_logger(), "笛卡尔路径规划失败，完成度：%.2f%%", fraction * 100.0);
        // 如果 fraction < 1.0，通常是因为向下移动会碰撞到底座、地面或超出工作空间
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}