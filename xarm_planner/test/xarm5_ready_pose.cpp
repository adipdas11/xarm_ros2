/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
*
* Software License Agreement (BSD License)
*
* Author: Vinman <vinman.cub@gmail.com>
============================================================================
*/
#include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_combined_test] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    // --- ROS 2 init ---
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("xarm_combined_test", node_options);

    RCLCPP_INFO(node->get_logger(), "xarm_combined_test start");
    signal(SIGINT, exit_sig_handler);

    // --- parameters for arm group ---
    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));

    std::string arm_group = robot_type;
    if (robot_type == "xarm" || robot_type == "lite") {
    arm_group += std::to_string(dof);
    }
    if (!prefix.empty()) {
    arm_group = prefix + arm_group;
    }

    // build gripper group name, e.g. "xarm_gripper" or "prefixxarm_gripper"
    std::string grip_group = robot_type + "_gripper";
    if (!prefix.empty()) {
    grip_group = prefix + grip_group;
    }

    RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());
    RCLCPP_INFO(node->get_logger(), "arm group: %s, gripper group: %s",
                arm_group.c_str(), grip_group.c_str());

    // --- planners ---
    xarm_planner::XArmPlanner arm_planner(node, arm_group);
    xarm_planner::XArmPlanner grip_planner(node, grip_group);

    // --- prepare targets ---
    // open-gripper joints (6-joint gripper)
    std::vector<double> open_joints(6, 0.0);

    // target pose
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.position.x = 0.37;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.35;
    // target_pose.orientation.x = 1.0;
    // target_pose.orientation.y = 0;
    // target_pose.orientation.z = 0;
    // target_pose.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.37;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.30;
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 0;

    // --- execute sequence once ---
    if (rclcpp::ok()) {
        // 1) open gripper
        RCLCPP_INFO(node->get_logger(), "Opening gripper...");
        grip_planner.planJointTarget(open_joints);
        grip_planner.executePath();

        // 2) move arm to target pose
        RCLCPP_INFO(node->get_logger(), "Moving arm to target pose...");
        arm_planner.planPoseTarget(target_pose);
        arm_planner.executePath();
    }

    RCLCPP_INFO(node->get_logger(), "xarm_combined_test over");
    return 0;
}
