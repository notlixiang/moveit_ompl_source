#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <msg_package/grasp.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <geometric_shapes/shape_operations.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <boost/timer.hpp>
#include <trac_ik/trac_ik.hpp>

std::vector<double> home = {0.48122, -1.2618, -1.5896, 1.7762, -1.352, -1.2236};
std::vector<double> place = {-0.437, -1, -1.62, 1.3625, -1.1244, -0.6764};

std::vector<double> observe_up_right = {-0.3126, -1.729, -1.1581, 1.823, -1.74623, -1.9182};
std::vector<double> observe_down_right = {-0.3157, -2.0473, -1.9597, 2.941, -1.7498, -1.9286};
std::vector<double> observe_up_left = {0.87972, -1.7767, -1.0199, 1.5692, -1.2163, -0.8315};
std::vector<double> observe_down_left = {0.882, -2.073, -1.947, 2.8352, -1.178, -0.85450};
std::vector<double> observe_up_middle = {0.337, -1.51, -1.400, 1.853, -1.42, -1.354};
std::vector<double> observe_down_middle = {0.279, -1.905, -2.215, 3.071, -1.451, -1.412};

std::vector<double> pre_down_right = {-0.47122, -2.6468, -1.91, 4.538, -2.0767, -1.67};
std::vector<double> pre_up_right = {-0.66, -1.2241, -2.326, 3.568, -2.2556, -1.646};
std::vector<double> pre_down_left = {0.925, -2.638, -1.988, 4.617, -0.681, -1.652};
std::vector<double> pre_up_left = {0.7982, -1.759, -1.906, 3.63, -0.847, -1.541};
std::vector<double> pre_down_middle = {0.3650, -2.564, -2.351, 4.7527, -1.247, -1.5026};
std::vector<double> pre_up_middle = {0.6812, -0.9714, -2.445, 3.4338, -0.915, -1.5814};

std::vector<double> observe_desk_left = {0.3782760798931122, -1.807453457509176, -1.470379654561178,
                                         -1.1602729002581995, 1.560141682624817, 1.9563981294631958};
std::vector<double> observe_desk_right = {-0.11585075059999639, -1.7541731039630335, -1.6129143873797815,
                                          -1.0659402052508753, 1.6896229982376099, 1.4781898260116577};

double up_level_z_up = 0.5;
double up_level_z_down = 0.34;
double down_level_z_up = -0.08;
double down_level_z_down = -0.26;
double x_up = 0.64;
double x_down = -0.64;
double y_up = -1.05;
double y_down = -1.4;

double desk_x_down = -0.4;
double desk_y_up = -0.8;
double desk_z_down = 0.152;
double desk_x_up = 0.5;
double desk_y_down = -1.5;
double desk_z_up = 0.32;

double cartesian_distance = 0.1;
bool add_colllsion_ = false;

enum M_robot_state {
    s_home, s_place, s_grasp_up, s_grasp_down,
    s_up_right_observe, s_up_left_observe, s_up_middle_observe,
    s_down_right_observe, s_down_left_observe, s_down_middle_observe,
    s_observe_desk_left, s_observe_desk_right
};
M_robot_state m_robot_state;

enum M_agv_state {
    unknown, huojia, desk
};
M_agv_state m_agv_state;

ros::Publisher planning_scene_diff_publisher;
ros::ServiceClient planning_scene_diff_client;

Eigen::Matrix<double, 4, 4> camera_to_depth;
Eigen::Matrix<double, 4, 4> ee_to_camera;

Eigen::Matrix<double, 4, 4> up_right_base_to_ee;
Eigen::Matrix<double, 4, 4> down_right_base_to_ee;
Eigen::Matrix<double, 4, 4> up_left_base_to_ee;
Eigen::Matrix<double, 4, 4> down_left_base_to_ee;
Eigen::Matrix<double, 4, 4> down_middle_base_to_ee;
Eigen::Matrix<double, 4, 4> up_middle_base_to_ee;

Eigen::Matrix<double, 4, 4> desk_right_base_to_ee;
Eigen::Matrix<double, 4, 4> desk_left_base_to_ee;

geometry_msgs::Pose current_pose;

bool pose_plan(std::string planning_group, std::string reframe, geometry_msgs::Pose target_pose, int pos) {
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    move_group.setMaxVelocityScalingFactor(1);
    move_group.setPoseReferenceFrame(reframe);
    move_group.setPoseTarget(target_pose);

//    if(pos == 0)
//        move_group.setPlannerId("RRTConnectkConfigDefault");
//    else if(pos == 1)
//        move_group.setPlannerId("URPRMkConfigDefault");
//    else if(pos == 2)
//        move_group.setPlannerId("DRPRMkConfigDefault");
//    else if(pos == 3)
//        move_group.setPlannerId("UMPRMkConfigDefault");
//    else if(pos == 4)
//        move_group.setPlannerId("DMPRMkConfigDefault");
//    else if(pos == 5)
//        move_group.setPlannerId("ULPRMkConfigDefault");
//    else if(pos == 6)
//        move_group.setPlannerId("DLPRMkConfigDefault");
//    else

    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlannerId("RRTstarReformkConfigDefault");
    move_group.setPlanningTime(0.5);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("pose_planning......\n");
    boost::timer timer;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << "************************" << std::endl;
    std::cout << "pose planning cost time: " << timer.elapsed() << std::endl;
    std::cout << "************************" << std::endl;
    ROS_INFO("Visualizing plan pose (pose goal) %s", success ? "" : "FAILED");
    if (success) {
        move_group.execute(my_plan);
        return 1;
    } else {
        ROS_ERROR("pose PLAN FAILED!!!");
        return 0;
    }
}

bool trac_ik_solver(std::vector<double> current_q, geometry_msgs::Pose pose_, std::vector<double> &result_q) {
    KDL::Frame end_effector_pose;
    Eigen::Quaterniond q_end_effector_pose(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y,
                                           pose_.orientation.z);
    Eigen::Matrix<double, 3, 3> r_end_effector_pose;
    r_end_effector_pose = q_end_effector_pose.toRotationMatrix();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            end_effector_pose.M(i, j) = r_end_effector_pose(i, j);
        }
    }
    end_effector_pose.p(0) = pose_.position.x;
    end_effector_pose.p(1) = pose_.position.y;
    end_effector_pose.p(2) = pose_.position.z;

    std::string chain_start, chain_end, urdf_param;
    double timeout;

    chain_start = "base";
    chain_end = "gripper";
    urdf_param = "/robot_description";
    timeout = 0.005;
    double eps = 1e-5;

    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

    KDL::Chain chain;
    KDL::JntArray ll, ul;

    bool valid = tracik_solver.getKDLChain(chain);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return 0;
    }

    if (current_q.size() != chain.getNrOfJoints()) {
        ROS_ERROR("joint number error!");
        return 0;
    }

    valid = tracik_solver.getKDLLimits(ll, ul);
    if (!valid) {
        ROS_ERROR("There were no valid KDL joint limits found");
        return 0;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());
    ROS_INFO ("Using %d joints", chain.getNrOfJoints());

    KDL::JntArray current(chain.getNrOfJoints());
    for (int i = 0; i < chain.getNrOfJoints(); i++)
        current(i) = current_q[i];

    KDL::JntArray result;
    int rc;
    rc = tracik_solver.CartToJnt(current, end_effector_pose, result);
    if (rc >= 0) {
        result_q.clear();
        for (int i = 0; i < chain.getNrOfJoints(); i++)
            result_q.push_back(result(i));
        return 1;
    } else {
        ROS_ERROR("no ik solution!");
        return 0;
    }
}

bool Joint_plan(std::string planning_group, std::string reframe, std::vector<double> joint_group_positions, int pos) {
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    move_group.setMaxVelocityScalingFactor(1);
    move_group.setPoseReferenceFrame(reframe);
    move_group.setJointValueTarget(joint_group_positions);

//    if(pos == 0)
//        move_group.setPlannerId("RRTConnectkConfigDefault");
//    else if(pos == 1)
//        move_group.setPlannerId("URPRMkConfigDefault");
//    else if(pos == 2)
//        move_group.setPlannerId("DRPRMkConfigDefault");
//    else if(pos == 3)
//        move_group.setPlannerId("UMPRMkConfigDefault");
//    else if(pos == 4)
//        move_group.setPlannerId("DMPRMkConfigDefault");
//    else if(pos == 5)
//        move_group.setPlannerId("ULPRMkConfigDefault");
//    else if(pos == 6)
//        move_group.setPlannerId("DLPRMkConfigDefault");
//    else
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlannerId("RRTstarReformkConfigDefault");
    move_group.setPlanningTime(0.5);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    boost::timer timer;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << "************************" << std::endl;
    std::cout << "Joint planning cost time: " << timer.elapsed() << std::endl;
    std::cout << "************************" << std::endl;
    ROS_INFO("Visualizing plan Joint (pose goal) %s", success ? "" : "FAILED");
    if (success) {
        move_group.execute(my_plan);
        return 1;
    } else {
        ROS_ERROR("JOINT PLAN FAILED!!!");
        return 0;
    }
}

bool pose_to_joint_plan(std::string planning_group, std::string reframe, geometry_msgs::Pose target_pose) {
    if (target_pose.position.z > 0) {
        if (target_pose.position.x >= -0.2 && target_pose.position.x <= 0.2) {
            std::vector<double> result;
            if (trac_ik_solver(pre_up_middle, target_pose, result)) {
                for (int i = 0; i < 6; i++)
                    std::cout << result[i] << "   ";
                std::cout << std::endl;
                bool execute_result = Joint_plan(planning_group, "base", result, 3);
                if (execute_result)
                    return 1;
                else
                    return 0;
            } else
                return 0;
        } else if (target_pose.position.x < -0.2) {
            std::vector<double> result;
            if (trac_ik_solver(pre_up_right, target_pose, result)) {
                for (int i = 0; i < 6; i++)
                    std::cout << result[i] << "   ";
                std::cout << std::endl;
                bool execute_result = Joint_plan(planning_group, "base", result, 1);
                if (execute_result)
                    return 1;
                else
                    return 0;
            } else
                return 0;
        } else if (target_pose.position.x > 0.2) {
            std::vector<double> result;
            if (trac_ik_solver(pre_up_left, target_pose, result)) {
                for (int i = 0; i < 6; i++)
                    std::cout << result[i] << "   ";
                std::cout << std::endl;
                bool execute_result = Joint_plan(planning_group, "base", result, 5);
                if (execute_result)
                    return 1;
                else
                    return 0;
            } else
                return 0;
        } else
            return 0;
    } else {
        if (target_pose.position.x >= -0.2 && target_pose.position.x <= 0.2) {
            std::vector<double> result;
            if (trac_ik_solver(pre_down_middle, target_pose, result)) {
                for (int i = 0; i < 6; i++)
                    std::cout << result[i] << "   ";
                std::cout << std::endl;
                bool execute_result = Joint_plan(planning_group, "base", result, 4);
                if (execute_result)
                    return 1;
                else
                    return 0;
            } else
                return 0;
        } else if (target_pose.position.x < -0.2) {
            std::vector<double> result;
            if (trac_ik_solver(pre_down_right, target_pose, result)) {
                for (int i = 0; i < 6; i++)
                    std::cout << result[i] << "   ";
                std::cout << std::endl;
                bool execute_result = Joint_plan(planning_group, "base", result, 2);
                if (execute_result)
                    return 1;
                else
                    return 0;
            } else
                return 0;
        } else if (target_pose.position.x > 0.2) {
            std::vector<double> result;
            if (trac_ik_solver(pre_down_left, target_pose, result)) {
                for (int i = 0; i < 6; i++)
                    std::cout << result[i] << "   ";
                std::cout << std::endl;
                bool execute_result = Joint_plan(planning_group, "base", result, 6);
                if (execute_result)
                    return 1;
                else
                    return 0;
            } else
                return 0;
        } else
            return 0;
    }
}

bool C_plan(std::string planning_group, std::string reframe, std::vector<geometry_msgs::Pose> waypoints) {
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    move_group.setMaxVelocityScalingFactor(1);
    move_group.setPoseReferenceFrame(reframe);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    boost::timer timer;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan C (cartesian path) (%.2f%% acheived)", fraction * 100.0);
    std::cout << "************************" << std::endl;
    std::cout << "cartesian planning cost time: " << timer.elapsed() << std::endl;
    std::cout << "************************" << std::endl;
    if (fraction >= 0.99) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        move_group.execute(my_plan);
        return 1;
    } else {
        ROS_ERROR("cartesian PLAN FAILED!!!");
        return 0;
    }
}

geometry_msgs::Pose getCurrentPose(std::string planning_group) {
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    move_group.setPoseReferenceFrame("base");
    geometry_msgs::PoseStamped currentPoseStamped = move_group.getCurrentPose();
    return currentPoseStamped.pose;
}

bool move_to_observe_up_right() {
    ROS_INFO("move_to_observe_up_right");
    std::string PLANNING_GROUP = "arm";
    bool execute_result;
    if (m_robot_state == s_place)
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_up_right, 1);
    else
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_up_right, 0);
    if (execute_result) {
        ROS_INFO("move to observe_up_right succeed");
        m_robot_state = s_up_right_observe;
        return true;
    } else {
        ROS_ERROR("move to observe_up_right fail");
        return false;
    }
}

bool move_to_observe_down_right() {
    ROS_INFO("move_to_observe_down_right");
    std::string PLANNING_GROUP = "arm";
    bool execute_result;
    if (m_robot_state == s_place)
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_down_right, 2);
    else
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_down_right, 0);
    if (execute_result) {
        ROS_INFO("move to observe_down_right succeed");
        m_robot_state = s_down_right_observe;
        return true;
    } else {
        ROS_ERROR("move to observe_down_right fail");
        return false;
    }
}

bool move_to_observe_up_left() {
    ROS_INFO("move_to_observe_up_left");
    std::string PLANNING_GROUP = "arm";
    bool execute_result;
    if (m_robot_state == s_place)
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_up_left, 5);
    else
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_up_left, 0);
    if (execute_result) {
        ROS_INFO("move to observe_up_left succeed");
        m_robot_state = s_up_left_observe;
        return true;
    } else {
        ROS_ERROR("move to observe_up_left fail");
        return false;
    }
}

bool move_to_observe_down_left() {
    ROS_INFO("move_to_observe_down_left");
    std::string PLANNING_GROUP = "arm";
    bool execute_result;
    if (m_robot_state == s_place)
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_down_left, 6);
    else
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_down_left, 0);
    if (execute_result) {
        ROS_INFO("move to observe_down_left succeed");
        m_robot_state = s_down_left_observe;
        return true;
    } else {
        ROS_ERROR("move to observe_down fail");
        return false;
    }
}

bool move_to_observe_down_middle() {
    ROS_INFO("move_to_observe_down_middle");
    std::string PLANNING_GROUP = "arm";
    bool execute_result;
    if (m_robot_state == s_place)
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_down_middle, 4);
    else
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_down_middle, 0);
    if (execute_result) {
        ROS_INFO("move to observe_down_middle succeed");
        m_robot_state = s_down_middle_observe;
        return true;
    } else {
        ROS_ERROR("move to observe_down_middle fail");
        return false;
    }
}

bool move_to_observe_up_middle() {
    ROS_INFO("move_to_observe_up_middle");
    std::string PLANNING_GROUP = "arm";
    bool execute_result;
    if (m_robot_state == s_place)
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_up_middle, 3);
    else
        execute_result = Joint_plan(PLANNING_GROUP, "base", observe_up_middle, 0);
    if (execute_result) {
        ROS_INFO("move to observe_up_middle succeed");
        m_robot_state = s_up_middle_observe;
        return true;
    } else {
        ROS_ERROR("move to observe_up_middle fail");
        return false;
    }
}

bool move_to_home() {
    ROS_INFO("move_to_home");
    std::string PLANNING_GROUP = "arm";
    bool execute_result = Joint_plan(PLANNING_GROUP, "base", home, 0);
    if (execute_result) {
        ROS_INFO("move to home succeed");
        m_robot_state = s_home;
        return true;
    } else {
        ROS_ERROR("move to home fail");
        return false;
    }
}

bool move_to_observe_desk_left() {
    ROS_INFO("move_to_desk_left_observe");
    std::string PLANNING_GROUP = "arm";
    bool execute_result = Joint_plan(PLANNING_GROUP, "base", observe_desk_left, 0);
    if (execute_result) {
        ROS_INFO("move to observe desk_left succeed");
        m_robot_state = s_observe_desk_left;
        return true;
    } else {
        ROS_ERROR("move to observe desk_left fail");
        return false;
    }
}

bool move_to_observe_desk_right() {
    ROS_INFO("move_to_desk_right_observe");
    std::string PLANNING_GROUP = "arm";
    bool execute_result = Joint_plan(PLANNING_GROUP, "base", observe_desk_right, 0);
    if (execute_result) {
        ROS_INFO("move to observe desk_right succeed");
        m_robot_state = s_observe_desk_right;
        return true;
    } else {
        ROS_ERROR("move to observe desk_right fail");
        return false;
    }
}

bool move_to_place_with_plan() {
    ROS_INFO("move_to_place_with_plan");
    std::string PLANNING_GROUP = "arm";
    bool execute_result = Joint_plan(PLANNING_GROUP, "base", place, 0);
    if (execute_result) {
        ROS_INFO("move to place succeed");
        m_robot_state = s_place;
        return true;
    } else {
        ROS_ERROR("plan to place fail");
        return false;
    }
}

bool is_obj_pose_valid(geometry_msgs::Pose pose_) {
    if (((m_robot_state == s_up_left_observe) || (m_robot_state == s_up_right_observe) ||
         (m_robot_state == s_up_middle_observe)) && (m_agv_state == huojia)) {
        if (pose_.position.z > up_level_z_up || pose_.position.z < up_level_z_down) {
            ROS_ERROR("obj's pose Z is %f ,error!", pose_.position.z);
            return false;
        }
        if (pose_.position.x > x_up || pose_.position.x < x_down) {
            ROS_ERROR("obj's pose X is %f ,error!", pose_.position.x);
            return false;
        }
        if (pose_.position.y > y_up || pose_.position.y < y_down) {
            ROS_ERROR("obj's pose Y is %f ,error!", pose_.position.y);
            return false;
        }

        return true;
    } else if (((m_robot_state == s_down_left_observe) || (m_robot_state == s_down_right_observe) ||
                (m_robot_state == s_down_middle_observe)) && (m_agv_state == huojia)) {
        if (pose_.position.z > down_level_z_up || pose_.position.z < down_level_z_down) {
            ROS_ERROR("obj's pose Z is %f ,error!", pose_.position.z);
            return false;
        }
        if (pose_.position.x > x_up || pose_.position.x < x_down) {
            ROS_ERROR("obj's pose X is %f ,error!", pose_.position.x);
            return false;
        }
        if (pose_.position.y > y_up || pose_.position.y < y_down) {
            ROS_ERROR("obj's pose Y is %f ,error!", pose_.position.y);
            return false;
        }

        return true;
    } else if (((m_robot_state == s_observe_desk_left) || (m_robot_state == s_observe_desk_right)) &&
               (m_agv_state == desk)) {
        if (pose_.position.z > desk_z_up || pose_.position.z < desk_z_down) {
            ROS_ERROR("obj's pose Z is %f ,error!", pose_.position.z);
            return false;
        }
        if (pose_.position.x > desk_x_up || pose_.position.x < desk_x_down) {
            ROS_ERROR("obj's pose X is %f ,error!", pose_.position.x);
            return false;
        }
        if (pose_.position.y > desk_y_up || pose_.position.y < desk_y_down) {
            ROS_ERROR("obj's pose Y is %f ,error!", pose_.position.y);
            return false;
        }

        return true;
    } else {
        return false;
    }
}

int getCurrentRegion(geometry_msgs::Pose pose_) {
    if (pose_.position.z > 0) {
        if (pose_.position.x >= -0.2 && pose_.position.x <= 0.2) {
            return 3;
        } else if (pose_.position.x < -0.2) {
            return 1;
        } else if (pose_.position.x > 0.2) {
            return 5;
        } else
            return 0;
    } else {
        if (pose_.position.x >= -0.2 && pose_.position.x <= 0.2) {
            return 4;
        } else if (pose_.position.x < -0.2) {
            return 2;
        } else if (pose_.position.x > 0.2) {
            return 6;
        } else
            return 0;
    }
}

void add_gripper_collision_object() {
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "gripper";
    attached_object.object.header.frame_id = "gripper";
    attached_object.object.id = "box";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0;
    pose.position.y = 0.05;
    pose.position.z = 0.08;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.08;
    primitive.dimensions[1] = 0.08;
    primitive.dimensions[2] = 0.15;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);

    moveit_msgs::CollisionObject remove_object;
    remove_object.id = "box";
    remove_object.header.frame_id = "gripper";
    remove_object.operation = remove_object.REMOVE;

    ROS_INFO("Attaching the object to the hand and removing it from the world.");
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    //planning_scene_diff_publisher.publish(planning_scene);
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

void remove_gripper_collision_object() {
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "gripper";
    attached_object.object.header.frame_id = "gripper";
    attached_object.object.id = "box";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0;
    pose.position.y = 0.05;
    pose.position.z = 0.08;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.08;
    primitive.dimensions[1] = 0.08;
    primitive.dimensions[2] = 0.15;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "box";
    detach_object.link_name = "gripper";
    detach_object.object.operation = attached_object.object.REMOVE;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);

    moveit_msgs::CollisionObject remove_object;
    remove_object.id = "box";
    remove_object.header.frame_id = "gripper";
    remove_object.operation = remove_object.REMOVE;

    ROS_INFO("Removing the object from the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    //planning_scene_diff_publisher.publish(planning_scene);
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

void add_desk() {
    moveit_msgs::CollisionObject col_sphere_msg;
    col_sphere_msg.id = "desk";
    col_sphere_msg.header.frame_id = "/world";
    col_sphere_msg.operation = moveit_msgs::CollisionObject::ADD;

    shapes::Mesh *m = shapes::createMeshFromResource("package://ur_description/meshes/desk1.stl");
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
    col_sphere_msg.meshes.resize(1);
    col_sphere_msg.meshes[0] = co_mesh;
    col_sphere_msg.mesh_poses.resize(1);
    col_sphere_msg.mesh_poses[0].position.x = 0;
    col_sphere_msg.mesh_poses[0].position.y = -1.05;
    col_sphere_msg.mesh_poses[0].position.z = 0;
    col_sphere_msg.mesh_poses[0].orientation.w = 1.0;
    col_sphere_msg.mesh_poses[0].orientation.x = 0.0;
    col_sphere_msg.mesh_poses[0].orientation.y = 0.0;
    col_sphere_msg.mesh_poses[0].orientation.z = 0.0;

    col_sphere_msg.meshes.push_back(co_mesh);
    col_sphere_msg.mesh_poses.push_back(col_sphere_msg.mesh_poses[0]);
    col_sphere_msg.operation = col_sphere_msg.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(col_sphere_msg);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

void remove_desk() {
    moveit_msgs::CollisionObject col_sphere_msg;
    col_sphere_msg.id = "desk";
    col_sphere_msg.header.frame_id = "/world";
    col_sphere_msg.operation = moveit_msgs::CollisionObject::REMOVE;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(col_sphere_msg);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

void add_huojia() {
    moveit_msgs::CollisionObject col_sphere_msg;
    col_sphere_msg.id = "huojia";
    col_sphere_msg.header.frame_id = "/world";
    col_sphere_msg.operation = moveit_msgs::CollisionObject::ADD;

    shapes::Mesh *m = shapes::createMeshFromResource("package://ur_description/meshes/huojia1.stl");
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
    col_sphere_msg.meshes.resize(1);
    col_sphere_msg.meshes[0] = co_mesh;
    col_sphere_msg.mesh_poses.resize(1);
    col_sphere_msg.mesh_poses[0].position.x = 0;
    col_sphere_msg.mesh_poses[0].position.y = -1.05;
    col_sphere_msg.mesh_poses[0].position.z = 0.0;
    col_sphere_msg.mesh_poses[0].orientation.w = 1.0;
    col_sphere_msg.mesh_poses[0].orientation.x = 0.0;
    col_sphere_msg.mesh_poses[0].orientation.y = 0.0;
    col_sphere_msg.mesh_poses[0].orientation.z = 0.0;

    col_sphere_msg.meshes.push_back(co_mesh);
    col_sphere_msg.mesh_poses.push_back(col_sphere_msg.mesh_poses[0]);
    col_sphere_msg.operation = col_sphere_msg.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(col_sphere_msg);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

void remove_huojia() {
    moveit_msgs::CollisionObject col_sphere_msg;
    col_sphere_msg.id = "huojia";
    col_sphere_msg.header.frame_id = "/world";
    col_sphere_msg.operation = moveit_msgs::CollisionObject::REMOVE;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(col_sphere_msg);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

void add_collision_object(geometry_msgs::Pose pose_) {
    moveit_msgs::AttachedCollisionObject object1, object2;
    object1.link_name = "base";
    object1.object.header.frame_id = "base";
    object1.object.id = "langan1";
    object2.link_name = "base";
    object2.object.header.frame_id = "base";
    object2.object.id = "langan2";

    geometry_msgs::Pose pose1, pose2;
    pose1.orientation.w = 1.0;
    pose1.position.x = pose_.position.x + 0.2;
    if (pose_.position.z > 0) {
        pose1.position.y = -1.37;
        pose1.position.z = 0.35;
    } else if (pose_.position.z < 0) {
        pose1.position.y = -1.37;
        pose1.position.z = -0.15;
    }

    pose2.orientation.w = 1.0;
    pose2.position.x = pose_.position.x - 0.2;
    pose2.position.y = -1.15;
    if (pose_.position.z > 0) {
        pose2.position.y = -1.37;
        pose2.position.z = 0.35;
    } else if (pose_.position.z < 0) {
        pose2.position.y = -1.37;
        pose2.position.z = -0.15;
    }

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.15;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.25;

    object1.object.primitives.push_back(primitive);
    object1.object.primitive_poses.push_back(pose1);
    object1.object.operation = object1.object.ADD;

    object2.object.primitives.push_back(primitive);
    object2.object.primitive_poses.push_back(pose2);
    object2.object.operation = object2.object.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object1.object);
    planning_scene.world.collision_objects.push_back(object2.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}

void remove_collision_object() {
    moveit_msgs::AttachedCollisionObject object1, object2;
    object1.link_name = "base";
    object1.object.header.frame_id = "base";
    object1.object.id = "langan1";
    object2.link_name = "base";
    object2.object.header.frame_id = "base";
    object2.object.id = "langan2";

    object1.object.operation = object1.object.REMOVE;
    object2.object.operation = object2.object.REMOVE;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object1.object);
    planning_scene.world.collision_objects.push_back(object2.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
}
