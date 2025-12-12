#include "threads.hpp"

#include <iostream>
#include <string>
#include <sstream>

#include "sharedMemory.hpp"

/// @brief Goal input thread.
///
/// Reads target poses (x, y, theta) from stdin and writes them to shared memory.
/// Each new goal increments g_shm->goal_seq and releases g_goal_sem so that
/// the controller thread can start working on it. Typing 'q' triggers a global stop.
void goal_thread() {
    std::cout << "\n=================================================\n";
    std::cout << "Enter goal as:  x  y  theta(rad)\n";
    std::cout << "Example:        0.3  0.0  1.57\n";
    std::cout << "Or type 'q' to quit.\n> ";

    while (true) {

        if (is_stop_requested()) {
            std::cout << "[Goal] Global stop requested. Exiting goal thread.\n";
            break;
        }

        std::cout << "\n[Goal] New goal (x y theta) or 'q': ";
        std::string line;

        if (!std::getline(std::cin, line)) {
            std::cout << "\n[Goal] Input closed. Exiting goal thread.\n";
            request_global_stop();
            break;
        }

        if (line == "q" || line == "Q") {
            std::cout << "[Goal] Quit requested. Exiting goal thread.\n";
            request_global_stop();
            break;
        }

        // Parse x, y, theta from input line
        float gx = 0.0f;
        float gy = 0.0f;
        float gtheta = 0.0f;

        {
            std::istringstream iss(line);
            if(!(iss >> gx >> gy >> gtheta)) {
                std::cout << "[Goal][WARN] Could not parse input. Please use: x y theta\n";
                continue;
            }

        }

        std::cout << "[Goal] New goal: x=" << gx << " y=" << gy << " theta=" << gtheta << "\n";

        // Write goal to shared memory and bump goal sequence counter
        {
            std::lock_guard<std::mutex> lock(g_shm_mutex);

            g_shm->goal_pose.x = gx;
            g_shm->goal_pose.y = gy;
            g_shm->goal_pose.theta = gtheta;

            g_shm->goal_valid = 1;    // new goal avaialble

            g_shm->goal_seq += 1;  // bump goal squence counter

            g_shm->goal_reached = 0;
        }

        // Notify controller thread that a new goal is available
        g_goal_sem.release();
    }

    std::cout << "[Goal] thread finished.\n";

}