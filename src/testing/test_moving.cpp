#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>

#include "connection/connection.hpp"
#include "core/geometry.hpp"
#include "core/parser.hpp"
#include "pathing/lincontrol.hpp"
#include "core/geometry.hpp"

// safety for MSVC or MinGW
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main()
{
    const std::string ip = "192.168.100.54";
    const int port_odom = 9998;
    const int port_cmd  = 9999;

    const double dt_s  = 0.1;
    const int dt_ms    = static_cast<int>(dt_s * 1000);
    const int max_steps = 2000; // safety

    // Goal pose
    const double goal_x = 0.20;
    const double goal_y = 0.00;
    const double goal_theta = 0.0; // M_PI / 2.0;

    try {
        // ----------------------------------------------------
        // Init networking
        // ----------------------------------------------------
        connection::init();
        std::cout << "[INFO] Winsock initialized\n";

        // ----------------------------------------------------
        // Controller setup
        // ----------------------------------------------------
        LinearController controller;
        controller.setTargetPosition(goal_x, goal_y, goal_theta);
        controller.start();

        std::cout << "[INFO] Moving to goal: ("
                  << goal_x << ", " << goal_y
                  << ", theta=" << goal_theta << ")\n";

        // ----------------------------------------------------
        // Main loop until controller reaches goal
        // ----------------------------------------------------
        int step = 0;
        while (!controller.isGoalReached() && step < max_steps)
        {
            // ---------------------- 1) READ ODOM ----------------------
            std::string odomMsg;
            try {
                odomMsg = connection::readTaggedMessage(ip, port_odom);
            }
            catch (const std::exception& e) {
                std::cerr << "[ERROR] reading odom: " << e.what() << "\n";
                ++step;
                continue;
            }

            // ---------------------- 2) PARSE ODOM ----------------------
            core::Pose2D pose{};
            if (!parseOdomToPose2D(odomMsg, pose)) {
                std::cerr << "[WARN] parseOdomToPose2D failed\n";
                ++step;
                continue;
            }

            // ---------------------- 3) UPDATE CONTROLLER ----------------------
            controller.updateRobotPose(pose.x, pose.y, pose.theta);

            // ---------------------- 4) COMPUTE CONTROL ----------------------
            ControlOutput u = controller.compute_control();

            std::cout << std::fixed << std::setprecision(3)
                      << "Step " << step
                      << " | Pose=(" << pose.x << ", " << pose.y << ", " << pose.theta << ")"
                      << " | v=" << u.v << "  w=" << u.w << "\n";

            // ---------------------- 5) BUILD COMMAND MESSAGE ----------------------
            std::string cmd = connection::buildTaggedControlMessageFromControlOutput(u);

            // ---------------------- 6) SEND TO ROBOT ----------------------
            try {
                connection::sendMessage(ip, port_cmd, cmd);
            }
            catch (const std::exception& e) {
                std::cerr << "[ERROR] send cmd: " << e.what() << "\n";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
            ++step;
        }

        // ----------------------------------------------------
        // EXIT / STOP COMMAND
        // ----------------------------------------------------
        if (controller.isGoalReached()) {
            std::cout << "\n[INFO] Goal reached! Stopping robot.\n";
        } else {
            std::cout << "\n[WARN] Max steps reached without reaching goal.\n";
        }

        // Send one final stop
        std::string stopMsg =
            connection::buildTaggedControlMessage(0.0, 0.0);

        try {
            connection::sendMessage(ip, port_cmd, stopMsg);
        } catch (...) {}

        // ----------------------------------------------------
        // Shutdown networking
        // ----------------------------------------------------
        connection::shutdown();
        std::cout << "[INFO] Finished. Winsock closed.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
    }

    return 0;
}
