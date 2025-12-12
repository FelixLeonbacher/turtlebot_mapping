/**
 * @author Merle Rehpeen
 * @version 1.0
 */

#pragma once
#include <mutex>
#include <semaphore>

// Maximum number of LiDAR ranges stored in shared memory
const int lidar_max = 2048;  

/// @brief  Simple pose representation shared between threads
struct SharedPose {
    double x;
    double y;
    double theta;
};

/// @brief POD struct stored entirely in shared memory
///
/// Contains current robot state, LiDAR scan data and various flags used for synchronization between threads
struct SharedData {
    SharedPose goal_pose;           //< Latest goal pose requested by the user
    SharedPose current_pose;        //< Latest pose from odometry

    // LiDAR scan data
    float lidar_scan[lidar_max];    //< Fixed-size buffer for LiDAR ranges
    int lidar_count;                //< Number of valid entries in lidar_scan
    float lidar_angle_min;
    float lidar_angle_inc;
    float lidar_range_min;
    float lidar_range_max;

    // Satus flags
    int goal_valid;                 //< 1 if new goal is available for the controller
    int pose_valid;                 //< 1 if current_pose is valid
    int network_ok;                 //< >0 if network init worked, <0 on error
    int scan_valid;                 //< 1 if a new LiDAR scan is available
    int goal_reached;               //< 1 if a goal is reached

    // Global stop flag for shutting down all threads.
    int stop;

    // Monotonically increasing goal sequence number to detect new goal
    int goal_seq;


};

/// Pointer to the SharedData block in shared memory
extern SharedData* g_shm; 

/// Global mutex protecting all accesses to g_shm
extern std::mutex g_shm_mutex;

/// Semaphore signalled when a new LiDAR scan is available
extern std::binary_semaphore g_scan_sem; 

/// Semaphore signalles when a new LiDAR scan is available
extern std::binary_semaphore g_goal_sem; 


/// @brief Initialize shared memory (create or open).
///
/// @param creator If true, a new shared memory segment is created and zeroed.
///                If false, an existing mapping is opened.
void ipc_init(bool creator);

/// @brief Detach and clean up shared memory resources
void ipc_cleanup();

/// @brief Request a global stop for all worker threads.
///
/// Sets g_shm->stop to 1 and releases semaphores so that sleeping
/// threads can wake up, check the flag and exit.
void request_global_stop();

/// @brief  Check whether a global stop was requested
bool is_stop_requested();


/// @brief Check whether a valid pose is available in shared memory.
///
/// @return true if g_shm is non-null and pose_valid != 0.
bool has_valid_pose();

