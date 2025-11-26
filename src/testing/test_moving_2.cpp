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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main()
{
    const std::string ip       = "192.168.100.54";
    const int         port_odom = 9998;
    const int         port_cmd  = 9999;

    const double dt_s  = 0.1;
    const int    dt_ms = int(dt_s * 1000);
    const int    max_steps = 2000; // per-goal safety cutoff

    try {
        // --------------------- Init Networking ---------------------
        connection::init();
        std::cout << "[INFO] Winsock initialized.\n";

        std::cout << std::fixed << std::setprecision(3);

        LinearController controller;

        // ================================
        //         MAIN INTERACTIVE LOOP
        // ================================
        while (true)
        {
            std::cout << "\n=================================================\n";
            std::cout << "Enter goal as:  x  y  theta(rad)\n";
            std::cout << "Example:        0.3  0.0  1.57\n";
            std::cout << "Or type 'q' to quit.\n> ";

            std::string line;
            if (!std::getline(std::cin, line)) {
                std::cout << "\n[INFO] Input closed. Exiting.\n";
                break;
            }

            if (line == "q" || line == "Q")
                break;

            // ------------------ PARSE GOAL INTO Pose2D ------------------
            core::Pose2D goal_pose{};
            {
                std::istringstream iss(line);
                if (!(iss >> goal_pose.x >> goal_pose.y >> goal_pose.theta)) {
                    std::cout << "[WARN] Could not parse input. Try again.\n";
                    continue;
                }
            }

            std::cout << "[INFO] New goal: "
                      << "x=" << goal_pose.x << "  y=" << goal_pose.y
                      << "  theta=" << goal_pose.theta << "\n";

            // ------------------ CONTROLLER SETUP ------------------
            controller.setTargetPosition(goal_pose.x,
                                         goal_pose.y,
                                         goal_pose.theta);
            controller.start();

            // ------------------ CONTROL LOOP ------------------
            int step = 0;
            while (!controller.isGoalReached() && step < max_steps)
            {
                // 1) Receive odometry
                std::string odomMsg;
                try {
                    odomMsg = connection::readTaggedMessage(ip, port_odom);
                } catch (const std::exception& e) {
                    std::cerr << "[ERROR] Reading odom: " << e.what() << "\n";
                    break;
                }

                // 2) Parse Pose2D
                core::Pose2D pose{};
                if (!parseOdomToPose2D(odomMsg, pose)) {
                    std::cerr << "[WARN] parseOdomToPose2D failed.\n";
                    continue;
                }

                // 3) Update controller with current robot pose
                controller.updateRobotPose(pose.x, pose.y, pose.theta);

                // 4) Compute control command
                ControlOutput u = controller.compute_control();

                std::cout << "Step " << step
                          << " | Pose=(" << pose.x << ", " << pose.y << ", " << pose.theta << ")"
                          << " | v=" << u.v << " w=" << u.w << "\r";
                std::cout.flush();

                // 5) Send control to robot
                std::string cmd = connection::buildTaggedControlMessageFromControlOutput(u);
                try {
                    connection::sendMessage(ip, port_cmd, cmd);
                } catch (const std::exception& e) {
                    std::cerr << "\n[ERROR] sendMessage: " << e.what() << "\n";
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
                ++step;
            }

            std::cout << "\n";

            // ------------------ FINISH / STOP ------------------
            if (controller.isGoalReached())
                std::cout << "[INFO] Goal reached! Sending stop.\n";
            else
                std::cout << "[WARN] Max steps reached. Sending stop.\n";

            std::string stop = connection::buildTaggedControlMessage(0.0, 0.0);
            try {
                connection::sendMessage(ip, port_cmd, stop);
            } catch (...) {}
        }

        // ------------------ SHUTDOWN ------------------
        connection::shutdown();
        std::cout << "[INFO] Winsock shutdown. Exiting.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
    }

    return 0;
}
