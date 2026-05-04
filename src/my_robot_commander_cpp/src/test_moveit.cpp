#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // // Named goal

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");
    
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");
    
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success2) {
    //     arm.execute(plan2);
    // }

    // --------------------------------------------------------------------------------

    // // Joint Goal

    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    // --------------------------------------------------------------------------------

    // Pose Goal

    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.7;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1) {
        arm.execute(plan1);

        // 🔽 CLOSE GRIPPER HERE
        gripper.setStartStateToCurrentState();
        gripper.setNamedTarget("gripper_closed");

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        if (gripper.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            gripper.execute(gripper_plan);
    }
    }

        // Cartesian Path 

        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
        pose1.position.z += -0.2;
        waypoints.push_back(pose1);
        geometry_msgs::msg::Pose pose2 = pose1;
        pose2.position.y += 0.2;
        waypoints.push_back(pose2); 
        geometry_msgs::msg::Pose pose3 = pose2;
        pose3.position.y += -0.2;
        pose3.position.z += 0.2;
        waypoints.push_back(pose3);

        rclcpp::sleep_for(std::chrono::seconds(1));

    // =========================
    // 3. ROTATE BASE
    // =========================
    std::vector<double> current_joints = arm.getCurrentJointValues();

// Rotate ONLY base joint
    current_joints[0] -= 1.57;   // or += 1.57

// Set new target
    arm.setJointValueTarget(current_joints);

    

    moveit::planning_interface::MoveGroupInterface::Plan rotate_plan;
    if (arm.plan(rotate_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        arm.execute(rotate_plan);
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // =========================
    // 4. OPEN GRIPPER
    // =========================
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("gripper_open");

    moveit::planning_interface::MoveGroupInterface::Plan open_plan;
    if (gripper.plan(open_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        gripper.execute(open_plan);
    }

        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

        if (fraction == 1 ) {
            arm.execute(trajectory);
        }

    rclcpp::shutdown();
    spinner.join();
    return 0;
 }