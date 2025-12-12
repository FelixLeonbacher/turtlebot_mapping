#include "threads.hpp"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>

#include "sharedMemory.hpp"
#include "connection/connection.hpp"
#include "core/geometry.hpp"
#include "pathing/lincontrol.hpp"
#include "config.hpp"

extern Config g_config;

/// @brief Controller thread.
///
/// Waits for new goals signalled via g_goal_sem, reads the current
/// robot pose from shared memory and uses a LinearController to compute
/// velocity commands. Commands are sent to the robot via TCP.
void controller_thread() {

    // Configure numeric output format
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[Controller] Thread started.\n";

    // IP + Port for command interface
    const std::string ip       = g_config.connection.ip;
    const int port_cmd = g_config.connection.port_cmd;

    // Controller loop period and max number of iterations per goal
    const double dt_s = 0.1;
    const int dt_ms = static_cast<int>(dt_s * 1000);
    const std::chrono::milliseconds loop_delay(dt_ms);
    const int max_steps = 2000;

    LinearController controller;

    while (true) {

        // check stop flag 
        if (is_stop_requested()) {
            std::cout << "[Controller] Stop requested, exiting.\n";
            break;
        }

        // Wait until a new goal is available (goal_thread releases g_goal_sem)
        g_goal_sem.acquire();

        // check stop flag
        if (is_stop_requested()) {
            std::cout << "[Controller] Stop requested, exiting.\n";
            break;
        }

        // Read goal and current pose atomically from shared memory
        core::Pose2D goal{};
        core::Pose2D pose{};
        int c_goal_seq = 0;

        {
            std::lock_guard<std::mutex> lock(g_shm_mutex);

            goal.x = g_shm->goal_pose.x;
            goal.y = g_shm->goal_pose.y;
            goal.theta = g_shm->goal_pose.theta;

            pose.x = g_shm->current_pose.x;
            pose.y = g_shm->current_pose.y;
            pose.theta = g_shm->current_pose.theta;

            c_goal_seq = g_shm->goal_seq;

        }

        std::cout << "[INFO] New goal: "
        << "x=" << goal.x << "  y=" << goal.y
        << "  theta=" << goal.theta <<  " (seq=" << c_goal_seq << ")\n";

        // Initialize controller with the target pose
        controller.setTargetPosition(goal.x, goal.y, goal.theta);
        controller.start();

        int step = 0;

        while(!controller.isGoalReached() && step < max_steps) {

            // check stop flag
            if (is_stop_requested()) {
                std::cout << "\n[Controller] Stop requested during control loop, aborting.\n";
                break;
            }

            // Abort if a new goal was issued in parallel
            {
                std::lock_guard<std::mutex> lock(g_shm_mutex);
                if (g_shm->goal_seq != c_goal_seq) {
                    std::cout << "\n[Controller] New goal detected (seq changed from " << c_goal_seq << " to " << g_shm->goal_seq << "). Aborting current goal.\n";
                    break;
                }
            }

            // Check whether a valid pose is available
            if (!has_valid_pose()) {
                std::this_thread::sleep_for(loop_delay);
                continue;
            }

            // Read latest pose from shared memory
            {
                std::lock_guard<std::mutex> lock(g_shm_mutex);
                pose.x = g_shm->current_pose.x;
                pose.y = g_shm->current_pose.y;
                pose.theta = g_shm->current_pose.theta;
            }

            // Update controller with current pose
            controller.updateRobotPose(pose.x, pose.y, pose.theta);

            // Compute control output (v, w)
            ControlOutput u = controller.compute_control();

            std::cout << "Step " << step
                          << " | Pose=(" << pose.x << ", " << pose.y << ", " << pose.theta << ")"
                          << " | v=" << u.v << " w=" << u.w << "\r";
            std::cout.flush();


            // cbuild and send velocity command via TCP
            std::string cmd = connection::buildTaggedControlMessageFromControlOutput(u);

            try {
                connection::sendMessage(ip, port_cmd, cmd);
            }
            catch (const std::exception& e) {
                std::cerr << "\n[Controller][ERROR] sendMessage: " << e.what() << "\n";
            }

            std::this_thread::sleep_for(loop_delay);
            ++step;

        }

        std::cout << "\n";

        // Send stop command after goal reached, timeout or global stop
        if (is_stop_requested()) {
            std::cout << "[Controller] Global stop, sending final stop.\n";
        } else if (controller.isGoalReached()) {
            std::cout << "[Controller] Goal reached! Sending stop.\n";
        } else {
            std::cout << "[Controller] Max steps reached. Sending stop.\n";
        }

        std::string stop = connection::buildTaggedControlMessage(0.0, 0.0);
        try {
            connection::sendMessage(ip, port_cmd, stop);
        }
        catch (...) {}
            // ignore errors while stopping


        // Mark goal as processed if sequence number did not change
        {
            std::lock_guard<std::mutex> lock(g_shm_mutex);
            if (g_shm->goal_seq == c_goal_seq) {
                g_shm->goal_valid = 0;
            }

        }

        if (is_stop_requested()) {
            break;  
        }
    }   
}