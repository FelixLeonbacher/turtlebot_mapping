#ifndef LINCONTROL_H
#define LINCONTROL_H

#include <cmath>

// Ist-/ Zielpose
struct Pose {
    double x;
    double y;
    double theta;
};

// Control output
struct ControlOutput {
    double v;    // linear velocity
    double w;    // angular velocity
};

class LinearController {
private:
    // Control parameters
    double k_roh;
    double k_alpha;
    double k_beta;
    
    // Goal pose
    Pose goal_pose;
    
    // Current robot pose
    Pose robot_pose;
    
    // Control state
    bool start_moving;
    bool goal_reached;
    
    // Velocity limits
    static const double v_max;
    static const double w_max;
    
    // Goal reached threshold
    static const double goal_distance_threshold;
    
public:
    LinearController();
    
    // Set target position
    void setTargetPosition(double x, double y, double theta = 0.0);
    
    // Update robot pose
    void updateRobotPose(double x, double y, double theta);
    
    // Set control parameters
    void setControlParameters(double k_roh, double k_alpha, double k_beta);
    
    // Start/stop control
    void start();
    void stop();
    
    // Angle normalization
    double normalize_angle(double angle);
    
    // Control computation
    ControlOutput compute_control();
    
    // Get goal reached flag
    bool isGoalReached() const;
    
    // Reset goal reached flag
    void resetGoalReached();
};

#endif // LINCONTROL_H
