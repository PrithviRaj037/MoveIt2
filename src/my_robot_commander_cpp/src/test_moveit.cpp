#include <rclcpp/rclcpp.hpp>  // Main ROS 2 C++ library for nodes, logging, publishers, sleep, etc.
#include <moveit/move_group_interface/move_group_interface.hpp>  // MoveIt interface used for arm motion planning.
#include <trajectory_msgs/msg/joint_trajectory.hpp>  // Message type used to send a trajectory to gripper joints.
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>  // One point inside a joint trajectory message.

#include <geometry_msgs/msg/pose_stamped.hpp>  // Pose with position, orientation, frame_id, and timestamp.

#include <chrono>  // Used for time durations, for example milliseconds.
#include <memory>  // Used for smart pointers such as shared_ptr.
#include <thread>  // Used to run the ROS executor in a background thread.
#include <cmath>  // Used for math functions such as sqrt.
#include <string>  // Used for std::string.
#include <vector>  // Used for vector/list data structures.

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;  // Short name for MoveIt's planning interface.

// ============================================================
// Helper function: normalize quaternion
// ============================================================

void normalizeQuaternion(geometry_msgs::msg::Quaternion &q)  // Function receives a quaternion by reference and modifies it.
{
    double norm = std::sqrt(  // Calculate the length/magnitude of the quaternion.
        q.x * q.x +  // Add x squared.
        q.y * q.y +  // Add y squared.
        q.z * q.z +  // Add z squared.
        q.w * q.w  // Add w squared.
    );

    if (norm > 1e-8)  // Check that norm is not almost zero to avoid division by zero.
    {
        q.x /= norm;  // Normalize x component.
        q.y /= norm;  // Normalize y component.
        q.z /= norm;  // Normalize z component.
        q.w /= norm;  // Normalize w component.
    }
}

// ============================================================
// Helper function: plan and execute with error handling
// ============================================================

bool planAndExecute(  // Function returns true if both planning and execution succeed.
    MoveGroupInterface &group,  // MoveIt planning group, for example arm.
    rclcpp::Logger logger,  // Logger used to print terminal messages.
    const std::string &motion_name  // Name of the motion for readable logs.
)
{
    MoveGroupInterface::Plan plan;  // Variable where MoveIt stores the planned trajectory.

    RCLCPP_INFO(logger, "Planning: %s", motion_name.c_str());  // Print which motion is being planned.

    auto result = group.plan(plan);  // Ask MoveIt to compute a motion plan.

    if (result == moveit::core::MoveItErrorCode::SUCCESS)  // Check if planning succeeded.
    {
        RCLCPP_INFO(logger, "Plan successful: %s", motion_name.c_str());  // Print planning success.

        auto exec_result = group.execute(plan);  // Send the planned trajectory to the controller for execution.

        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)  // Check if execution succeeded.
        {
            RCLCPP_INFO(logger, "Execution successful: %s", motion_name.c_str());  // Print execution success.
            return true;  // Return success to the caller.
        }
        else  // Execution failed.
        {
            RCLCPP_ERROR(logger, "Execution failed: %s", motion_name.c_str());  // Print execution failure.
            return false;  // Return failure to the caller.
        }
    }
    else  // Planning failed.
    {
        RCLCPP_ERROR(logger, "Planning failed: %s", motion_name.c_str());  // Print planning failure.
        return false;  // Return failure to the caller.
    }
}

// ============================================================
// Helper function: control gripper by sending trajectory directly
// ============================================================

bool controlGripperDirect(  // Function sends direct joint position commands to the gripper.
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher,  // Publisher used to send gripper trajectory.
    rclcpp::Logger logger,  // Logger used to print terminal messages.
    double left_position,  // Target position for left gripper finger.
    double right_position,  // Target position for right gripper finger.
    const std::string &action_name  // Name of action, for example open or close gripper.
)
{
    RCLCPP_INFO(logger, "Controlling gripper: %s", action_name.c_str());  // Print gripper action name.

    trajectory_msgs::msg::JointTrajectory trajectory;  // Create trajectory message for gripper.
    trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};  // Select both gripper joints.

    trajectory_msgs::msg::JointTrajectoryPoint point;  // Create one target point for the gripper trajectory.
    point.positions = {left_position, right_position};  // Set target finger positions.
    point.velocities = {0.0, 0.0};  // Set final velocities to zero so fingers stop at target.
    point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(500));  // Reach target in 0.5 seconds.

    trajectory.points.push_back(point);  // Add the target point to the trajectory message.

    publisher->publish(trajectory);  // Publish trajectory to the gripper controller topic.

    RCLCPP_INFO(logger, "Gripper trajectory published: %s [%.3f, %.3f]",
                action_name.c_str(), left_position, right_position);  // Print the sent gripper positions.

    rclcpp::sleep_for(std::chrono::milliseconds(600));  // Wait slightly longer than motion duration.

    return true;  // Return true because the command was published.
}

// ============================================================
// Main program
// ============================================================

int main(int argc, char **argv)  // Main function starts when the program runs.
{
    rclcpp::init(argc, argv);  // Initialize ROS 2.

    auto node = std::make_shared<rclcpp::Node>(  // Create a ROS 2 node object.
        "test_moveit",  // Name of the ROS 2 node.
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)  // Allow parameters passed from launch files.
    );

    rclcpp::executors::SingleThreadedExecutor executor;  // Create executor to process ROS callbacks.
    executor.add_node(node);  // Add this node to the executor.

    std::thread spinner([&executor]() {  // Start a separate thread for spinning.
        executor.spin();  // Keep processing ROS callbacks in the background.
    });

    auto gripper_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(  // Create publisher for gripper commands.
        "/gripper_controller/joint_trajectory", 10);  // Topic name and queue size.

    auto arm = MoveGroupInterface(node, "arm");  // Create MoveIt interface for the arm planning group.
    auto gripper = MoveGroupInterface(node, "gripper");  // Create MoveIt interface for gripper group, although direct control is used later.

    arm.setMaxVelocityScalingFactor(0.8);  // Use 80 percent of maximum allowed arm velocity.
    arm.setMaxAccelerationScalingFactor(0.8);  // Use 80 percent of maximum allowed arm acceleration.
    arm.setPlanningTime(10.0);  // Give MoveIt up to 10 seconds to find a plan.
    arm.setEndEffectorLink("hand_link");  // Tell MoveIt that hand_link is the end-effector.
    arm.setPoseReferenceFrame("base_link");  // Target poses are defined relative to base_link.

    gripper.setMaxVelocityScalingFactor(0.5);  // Configure gripper velocity scaling if MoveIt gripper group is used.
    gripper.setMaxAccelerationScalingFactor(0.5);  // Configure gripper acceleration scaling if MoveIt gripper group is used.
    gripper.setPlanningTime(5.0);  // Give gripper planning up to 5 seconds if used.

    RCLCPP_INFO(node->get_logger(), "========== STARTING PICK AND PLACE SEQUENCE ==========");  // Print sequence start.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 1] Moving to HOME position...");  // Print Step 1 message.

    arm.setStartStateToCurrentState();  // Start planning from current robot state.
    arm.setNamedTarget("home");  // Set predefined MoveIt/SRDF target named home.

    if (!planAndExecute(arm, node->get_logger(), "Move to home"))  // Plan and execute home motion.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to move to home, continuing anyway...");  // Warn if home motion failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait 0.5 seconds before next step.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 2] Opening gripper...");  // Print Step 2 message.

    controlGripperDirect(gripper_pub, node->get_logger(), 0.0, 0.0, "Open gripper");  // Open gripper with direct joint command.

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait for gripper to open.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 3] Approaching object...");  // Print Step 3 message.

    geometry_msgs::msg::PoseStamped approach_pose;  // Create pose target above/near the object.
    approach_pose.header.frame_id = "base_link";  // Pose is expressed relative to base_link.
    approach_pose.header.stamp = node->now();  // Add current timestamp.

    approach_pose.pose.position.x = 0.950;  // Set target x position.
    approach_pose.pose.position.y = 0.290;  // Set target y position.
    approach_pose.pose.position.z = 0.250;  // Set target z height above object.

    approach_pose.pose.orientation.x = 0.733;  // Set quaternion x orientation.
    approach_pose.pose.orientation.y = -0.679;  // Set quaternion y orientation.
    approach_pose.pose.orientation.z = 0.009;  // Set quaternion z orientation.
    approach_pose.pose.orientation.w = -0.037;  // Set quaternion w orientation.

    normalizeQuaternion(approach_pose.pose.orientation);  // Normalize quaternion before sending to MoveIt.

    arm.setStartStateToCurrentState();  // Plan from current robot state.
    arm.setPoseTarget(approach_pose, "hand_link");  // Set target pose for hand_link.

    if (!planAndExecute(arm, node->get_logger(), "Approach object"))  // Plan and execute approach motion.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to approach object, continuing...");  // Warn if approach failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait before descending.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 4] Descending to object (PICK)...");  // Print Step 4 message.

    geometry_msgs::msg::PoseStamped pick_pose = approach_pose;  // Copy approach pose to create pick pose.
    pick_pose.pose.position.z = 0.168;  // Lower z position to reach the object.

    arm.setStartStateToCurrentState();  // Plan from current robot state.
    arm.setPoseTarget(pick_pose, "hand_link");  // Set lower pick pose as target.

    if (!planAndExecute(arm, node->get_logger(), "Descend to object"))  // Plan and execute descent.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to descend to object, continuing...");  // Warn if descent failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(300));  // Short wait before closing gripper.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 5] Closing gripper to grasp object...");  // Print Step 5 message.

    controlGripperDirect(gripper_pub, node->get_logger(), 0.04, -0.04, "Close gripper");  // Close gripper by moving fingers inward.

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait for gripper to close.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 6] Lifting object...");  // Print Step 6 message.

    geometry_msgs::msg::PoseStamped lift_pose = pick_pose;  // Copy pick pose to create lift pose.
    lift_pose.pose.position.z += 0.25;  // Lift upward by 0.25 meters.

    arm.setStartStateToCurrentState();  // Plan from current robot state.
    arm.setPoseTarget(lift_pose, "hand_link");  // Set lifted pose as target for hand_link.

    if (!planAndExecute(arm, node->get_logger(), "Lift object"))  // Plan and execute lift motion.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to lift object, continuing...");  // Warn if lift failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait after lifting.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 7] Moving to place location...");  // Print Step 7 message.

    geometry_msgs::msg::PoseStamped place_approach_pose;  // Create pose above the place location.
    place_approach_pose.header.frame_id = "base_link";  // Pose is expressed relative to base_link.
    place_approach_pose.header.stamp = node->now();  // Add current timestamp.

    place_approach_pose.pose.position.x = -0.400;  // Set place approach x position.
    place_approach_pose.pose.position.y = 0.500;  // Set place approach y position.
    place_approach_pose.pose.position.z = 0.350;  // Set safe height above place surface.

    place_approach_pose.pose.orientation.x = 0.733;  // Set quaternion x orientation.
    place_approach_pose.pose.orientation.y = -0.679;  // Set quaternion y orientation.
    place_approach_pose.pose.orientation.z = 0.009;  // Set quaternion z orientation.
    place_approach_pose.pose.orientation.w = -0.037;  // Set quaternion w orientation.

    normalizeQuaternion(place_approach_pose.pose.orientation);  // Normalize quaternion before planning.

    arm.setStartStateToCurrentState();  // Plan from current robot state.
    arm.setPoseTarget(place_approach_pose, "hand_link");  // Set place approach pose as target.

    if (!planAndExecute(arm, node->get_logger(), "Move to place location"))  // Plan and execute move to place area.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to move to place location, continuing...");  // Warn if motion failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait before descending to place surface.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 8] Descending to place surface...");  // Print Step 8 message.

    geometry_msgs::msg::PoseStamped place_pose = place_approach_pose;  // Copy place approach pose.
    place_pose.pose.position.z = 0.100;  // Lower z position to place/release height.

    arm.setStartStateToCurrentState();  // Plan from current robot state.
    arm.setPoseTarget(place_pose, "hand_link");  // Set lower place pose as target.

    if (!planAndExecute(arm, node->get_logger(), "Descend to place surface"))  // Plan and execute descent to place surface.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to descend, continuing...");  // Warn if descent failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(300));  // Short wait before opening gripper.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 9] Opening gripper to release object...");  // Print Step 9 message.

    controlGripperDirect(gripper_pub, node->get_logger(), 0.0, 0.0, "Open gripper");  // Open gripper to release object.

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait for gripper to open.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 10] Retracting arm...");  // Print Step 10 message.

    geometry_msgs::msg::PoseStamped retract_pose = place_approach_pose;  // Use high place approach pose as retract pose.

    arm.setStartStateToCurrentState();  // Plan from current robot state.
    arm.setPoseTarget(retract_pose, "hand_link");  // Move hand_link away/up from placed object.

    if (!planAndExecute(arm, node->get_logger(), "Retract arm"))  // Plan and execute retract motion.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to retract, continuing...");  // Warn if retract failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait after retracting.

    RCLCPP_INFO(node->get_logger(), "\n[STEP 11] Returning to HOME position...");  // Print Step 11 message.

    arm.setStartStateToCurrentState();  // Plan from current robot state.
    arm.setNamedTarget("home");  // Set target to predefined home pose.

    if (!planAndExecute(arm, node->get_logger(), "Return to home"))  // Plan and execute return home.
    {
        RCLCPP_WARN(node->get_logger(), "Failed to return home, continuing...");  // Warn if return home failed.
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));  // Wait before shutdown.

    RCLCPP_INFO(node->get_logger(), "\n========== PICK AND PLACE SEQUENCE COMPLETE ==========\n");  // Print sequence complete.

    arm.clearPoseTargets();  // Clear any remaining pose targets from MoveIt.

    executor.cancel();  // Stop the executor spinning.
    spinner.join();  // Wait for background thread to finish safely.
    rclcpp::shutdown();  // Shut down ROS 2.

    return 0;  // End program successfully.
}
