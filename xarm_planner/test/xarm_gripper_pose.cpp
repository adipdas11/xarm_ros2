#include "xarm_planner/xarm_planner.h"
#include <std_msgs/msg/float64.hpp>
#include <signal.h>

std::shared_ptr<rclcpp::Node> node_;
std::shared_ptr<xarm_planner::XArmPlanner> planner_;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_gripper_pose_subscriber] Ctrl-C caught, exit process...\n");
    rclcpp::shutdown();
    exit(0);
}

void gripper_pose_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double joint_target = msg->data;
    RCLCPP_INFO(node_->get_logger(), "Received gripper target: %f", joint_target);

    std::vector<double> target_joint_positions(6, joint_target);  // all joints set to same value

    if (planner_->planJointTarget(target_joint_positions)) {
        planner_->executePath();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Gripper planning failed");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    node_ = rclcpp::Node::make_shared("xarm_gripper_pose_subscriber", node_options);
    RCLCPP_INFO(node_->get_logger(), "xarm_gripper_pose_subscriber node started");

    signal(SIGINT, exit_sig_handler);

    std::string robot_type;
    node_->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type + "_gripper";

    RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node_->get_namespace(), group_name.c_str());

    planner_ = std::make_shared<xarm_planner::XArmPlanner>(node_, group_name);

    auto subscription = node_->create_subscription<std_msgs::msg::Float64>(
        "/gripper_pose", 10, gripper_pose_callback);

    rclcpp::spin(node_);
    RCLCPP_INFO(node_->get_logger(), "Shutting down xarm_gripper_pose_subscriber");
    rclcpp::shutdown();
    return 0;
}
