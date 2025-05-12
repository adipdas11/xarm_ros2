#include "xarm_planner/xarm_planner.h"
#include <geometry_msgs/msg/pose.hpp>
#include <signal.h>

std::shared_ptr<rclcpp::Node> node_;
std::shared_ptr<xarm_planner::XArmPlanner> planner_;
geometry_msgs::msg::Pose current_target_pose_;
bool pose_received_ = false;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_pose_subscriber] Ctrl-C caught, exit process...\n");
    rclcpp::shutdown();
    exit(0);
}

void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "Received new target pose");
    current_target_pose_ = *msg;
    pose_received_ = true;

    if (planner_->planPoseTarget(current_target_pose_)) {
        planner_->executePath();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Planning failed");
    }

    pose_received_ = false;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    node_ = rclcpp::Node::make_shared("xarm_pose_subscriber", node_options);
    RCLCPP_INFO(node_->get_logger(), "xarm_pose_subscriber node started");

    signal(SIGINT, exit_sig_handler);

    int dof;
    node_->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node_->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string prefix;
    node_->get_parameter_or("prefix", prefix, std::string(""));


    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);

    if (!prefix.empty()) {
        group_name = prefix + group_name;
    }

    planner_ = std::make_shared<xarm_planner::XArmPlanner>(node_, group_name);

    auto subscription = node_->create_subscription<geometry_msgs::msg::Pose>(
        "/xarm_target_pose", 10, target_pose_callback);

    rclcpp::spin(node_);
    RCLCPP_INFO(node_->get_logger(), "Shutting down xarm_pose_subscriber");
    rclcpp::shutdown();
    return 0;
}
