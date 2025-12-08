#pragma once

const int lidar_max = 2048;  // maximale Speicherkapazität für LiDar Scan

struct SharedPose {
    double x;
    double y;
    double theta;
};

// --- define the SharedData ---
struct SharedData {
    SharedPose goal_pose;
    SharedPose current_pose;

    // lidar scan
    float lidar_scan[lidar_max]; 
    int lidar_count;
    float lidar_angle_min;
    float lidar_angle_inc;
    float lidar_range_min;
    float lidar_range_max;

    //flags
    int goal_valid;
    int pose_valid;  
    int network_ok;
    int scan_valid;
    int goal_reached;


    //stop flag
    int stop;
};


// declare ID and pointer
extern int shm_id;   // kernel ID of the shared memory segment
extern SharedData* g_shm; // pointer to the structure in shared memory



// initialize shared memory
void ipc_init();

// clean up shared memory
void ipc_cleanup();


