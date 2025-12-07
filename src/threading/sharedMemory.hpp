#pragma once

#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore>


const int lidar_max = 2048;  // maximale Speicherkapazität für LiDar Scan

struct Pose {
    double x;
    double y;
    double theta;
};

// --- define the SharedData ---
struct SharedData {
    Pose goal_pose;
    Pose current_pose;
    float lidar_scan[lidar_max];  //ggf. anpassen
    int lidar_count;

    //flags
    int goal_valid;
    int pose_valid;  
    int network_ok;
    int scan_valid;
};


// declare ID and pointer
extern int shm_id;   // kernel ID of the shared memory segment
extern SharedData* g_shm; // pointer to the structure in shared memory



// initialize shared memory
void ipc_init();

// clean up shared memory
void ipc_cleanup();


