#include "threads.hpp"

#include <iostream>
#include <string>
#include <sstream>

#include "sharedMemory.hpp"

void goal_thread() {
    std::cout << "\n=================================================\n";
    std::cout << "Enter goal as:  x  y  theta(rad)\n";
    std::cout << "Example:        0.3  0.0  1.57\n";
    std::cout << "Or type 'q' to quit.\n> ";

    while (true) {
        std::cout << "\n[Goal] New goal (x y theta) or 'q': ";
        std::string line;

        if (!std::getline(std::cin, line)) {
            std::cout << "\n[Goal] Input closed. Exiting goal thread.\n";
            break;
        }

        if (line == "q" || line == "Q") {
            std::cout << "[Goal] Quit requested. Exiting goal thread.\n";
            break;
        }

        // x y theta aus Eingabe parsen
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

        //In shared memory schreiben
        // TODO: mit semaphore schützen
        g_shm->goal_pose.x = gx;
        g_shm->goal_pose.y = gy;
        g_shm->goal_pose.theta = gtheta;

        g_shm->goal_valid = 1;    // neues Ziel ist available
    }

    std::cout << "[Goal] thread finished.\n";

}