/**
 * @file lincontrol.cpp
 * @brief Linear controller implementation for robot
 * @author Rainhard Wipp
 * @version 1.0
 */


#include "lincontrol.hpp"

// build problems cause compiler does not recognize M_PI from <cmath>
#define M_PI 3.14159265358979323846

// Initialize static constants
const double LinearController::v_max = 0.3;
const double LinearController::w_max = 3.0;
const double LinearController::goal_distance_threshold = 0.05;

// Constructor
LinearController::LinearController() 
    : k_roh(1.0), k_alpha(1.5), k_beta(-0.5),
      goal_pose({0.0, 0.0, 0.0}),
      robot_pose({0.0, 0.0, 0.0}),
      start_moving(false),
      goal_reached(false) {
}

// Set target position
void LinearController::setTargetPosition(double x, double y, double theta) {
    goal_pose.x = x;
    goal_pose.y = y;
    goal_pose.theta = theta;
    goal_reached = false;  // Reset goal reached flag when new target is set
}

// Update robot pose
void LinearController::updateRobotPose(double x, double y, double theta) {
    robot_pose.x = x;
    robot_pose.y = y;
    robot_pose.theta = theta;
    
    // Check if goal is reached
    if (start_moving) {
        double dx = goal_pose.x - robot_pose.x;
        double dy = goal_pose.y - robot_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance <= goal_distance_threshold) {
            goal_reached = true;
        }
    }
}

// Set control parameters
void LinearController::setControlParameters(double k_roh, double k_alpha, double k_beta) {
    this->k_roh = k_roh;
    this->k_alpha = k_alpha;
    this->k_beta = k_beta;
}

// Start control
void LinearController::start() {
    start_moving = true;
}

// Stop control
void LinearController::stop() {
    start_moving = false;
}

// +/- Pi
double LinearController::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Compute control output
ControlOutput LinearController::compute_control() {
    ControlOutput output = {0.0, 0.0};
    
    if (!start_moving) {
        return output;
    }

    // Abstand und Winkel zum Ziel
    double dx = goal_pose.x - robot_pose.x;
    double dy = goal_pose.y - robot_pose.y;
    double roh = std::sqrt(dx * dx + dy * dy);
    
    double alpha = normalize_angle(std::atan2(dy, dx) - robot_pose.theta);
    double beta = normalize_angle(goal_pose.theta - robot_pose.theta - alpha);

    // Steuerung
    double v = k_roh * roh;
    double w = k_alpha * alpha + k_beta * beta;

    // Limitierung
    v = std::max(-v_max, std::min(v, v_max));
    w = std::max(-w_max, std::min(w, w_max));

    output.v = v;
    output.w = w;

    return output;
}

// Get goal reached flag
bool LinearController::isGoalReached() const {
    return goal_reached;
}

// Reset goal reached flag
void LinearController::resetGoalReached() {
    goal_reached = false;
}
