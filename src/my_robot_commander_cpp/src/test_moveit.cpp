#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include <cmath>
#include <string>
#include <vector>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// ============================================================
// Helper function: normalize quaternion
// ============================================================

void normalizeQuaternion(geometry_msgs::msg::Quaternion &q)
{
    double norm = std::sqrt(
        q.x * q.x +
        q.y * q.y +
        q.z * q.z +
        q.w * q.w
    );

    if (norm > 1e-8)
    {
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
        q.w /= norm;
    }
}

// ============================================================
// Helper function: plan and execute with error handling
// ============================================================

bool planAndExecute(
    MoveGroupInterface &group,
    rclcpp::Logger logger,
    const std::string &motion_name
)
{
    MoveGroupInterface::Plan plan;

    RCLCPP_INFO(logger, "Planning: %s", motion_name.c_str());

    auto result = group.plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(logger, "Plan successful: %s", motion_name.c_str());

        auto exec_result = group.execute(plan);

        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Execution successful: %s", motion_name.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(logger, "Execution failed: %s", motion_name.c_str());
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed: %s", motion_name.c_str());
        return false;
    }
}

// ============================================================
// Helper function: control gripper by sending trajectory directly
// ============================================================

bool controlGripperDirect(
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher,
    rclcpp::Logger logger,
    double left_position,
    double right_position,
    const std::string &action_name
)
{
    RCLCPP_INFO(logger, "Controlling gripper: %s", action_name.c_str());

    // Create trajectory message
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};

    // Create a single point trajectory
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {left_position, right_position};
    point.velocities = {0.0, 0.0};
    point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(500));

    trajectory.points.push_back(point);

    // Publish the trajectory
    publisher->publish(trajectory);

    RCLCPP_INFO(logger, "Gripper trajectory published: %s [%.3f, %.3f]", 
                action_name.c_str(), left_position, right_position);

    // Wait for execution
    rclcpp::sleep_for(std::chrono::milliseconds(600));

    return true;
}

// ============================================================
// Main program
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "test_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

    // Create publisher for gripper trajectory
    auto gripper_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/gripper_controller/joint_trajectory", 10);

    // MoveIt groups
    auto arm = MoveGroupInterface(node, "arm");
    auto gripper = MoveGroupInterface(node, "gripper");

    // Configure arm
    arm.setMaxVelocityScalingFactor(0.8);
    arm.setMaxAccelerationScalingFactor(0.8);
    arm.setPlanningTime(10.0);
    arm.setEndEffectorLink("hand_link");
    arm.setPoseReferenceFrame("base_link");

    // Configure gripper
    gripper.setMaxVelocityScalingFactor(0.5);
    gripper.setMaxAccelerationScalingFactor(0.5);
    gripper.setPlanningTime(5.0);

    RCLCPP_INFO(node->get_logger(), "========== STARTING PICK AND PLACE SEQUENCE ==========");

    // ============================================================
    // STEP 1: Move to HOME position
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 1] Moving to HOME position...");
    
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");
    
    if (!planAndExecute(arm, node->get_logger(), "Move to home"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to move to home, continuing anyway...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 2: Open gripper (DIRECT CONTROL)
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 2] Opening gripper...");
    
    controlGripperDirect(gripper_pub, node->get_logger(), 0.0, 0.0, "Open gripper");

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 3: APPROACH object (move to position above object)
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 3] Approaching object...");

    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header.frame_id = "base_link";
    approach_pose.header.stamp = node->now();

    approach_pose.pose.position.x = 0.950;
    approach_pose.pose.position.y = 0.290;
    approach_pose.pose.position.z = 0.250;

    approach_pose.pose.orientation.x = 0.733;
    approach_pose.pose.orientation.y = -0.679;
    approach_pose.pose.orientation.z = 0.009;
    approach_pose.pose.orientation.w = -0.037;

    normalizeQuaternion(approach_pose.pose.orientation);

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(approach_pose, "hand_link");

    if (!planAndExecute(arm, node->get_logger(), "Approach object"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to approach object, continuing...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 4: PICK - Descend to object
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 4] Descending to object (PICK)...");

    geometry_msgs::msg::PoseStamped pick_pose = approach_pose;
    pick_pose.pose.position.z = 0.168;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(pick_pose, "hand_link");

    if (!planAndExecute(arm, node->get_logger(), "Descend to object"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to descend to object, continuing...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // ============================================================
    // STEP 5: Close gripper (DIRECT CONTROL)
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 5] Closing gripper to grasp object...");

    controlGripperDirect(gripper_pub, node->get_logger(), 0.04, -0.04, "Close gripper");

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 6: LIFT object - Move upward
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 6] Lifting object...");

    geometry_msgs::msg::PoseStamped lift_pose = pick_pose;
    lift_pose.pose.position.z += 0.25;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(lift_pose, "hand_link");

    if (!planAndExecute(arm, node->get_logger(), "Lift object"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to lift object, continuing...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 7: MOVE TO PLACE LOCATION
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 7] Moving to place location...");

    geometry_msgs::msg::PoseStamped place_approach_pose;
    place_approach_pose.header.frame_id = "base_link";
    place_approach_pose.header.stamp = node->now();

    place_approach_pose.pose.position.x = -0.400;
    place_approach_pose.pose.position.y = 0.500;
    place_approach_pose.pose.position.z = 0.350;

    place_approach_pose.pose.orientation.x = 0.733;
    place_approach_pose.pose.orientation.y = -0.679;
    place_approach_pose.pose.orientation.z = 0.009;
    place_approach_pose.pose.orientation.w = -0.037;

    normalizeQuaternion(place_approach_pose.pose.orientation);

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(place_approach_pose, "hand_link");

    if (!planAndExecute(arm, node->get_logger(), "Move to place location"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to move to place location, continuing...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 8: DESCEND to place surface
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 8] Descending to place surface...");

    geometry_msgs::msg::PoseStamped place_pose = place_approach_pose;
    place_pose.pose.position.z = 0.100;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(place_pose, "hand_link");

    if (!planAndExecute(arm, node->get_logger(), "Descend to place surface"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to descend, continuing...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // ============================================================
    // STEP 9: OPEN gripper (DIRECT CONTROL)
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 9] Opening gripper to release object...");

    controlGripperDirect(gripper_pub, node->get_logger(), 0.0, 0.0, "Open gripper");

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 10: RETRACT - Move away from placed object
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 10] Retracting arm...");

    geometry_msgs::msg::PoseStamped retract_pose = place_approach_pose;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(retract_pose, "hand_link");

    if (!planAndExecute(arm, node->get_logger(), "Retract arm"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to retract, continuing...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // STEP 11: Return to HOME
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n[STEP 11] Returning to HOME position...");

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    if (!planAndExecute(arm, node->get_logger(), "Return to home"))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to return home, continuing...");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // Cleanup and shutdown
    // ============================================================

    RCLCPP_INFO(node->get_logger(), "\n========== PICK AND PLACE SEQUENCE COMPLETE ==========\n");

    arm.clearPoseTargets();

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return 0;
}