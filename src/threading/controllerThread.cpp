#include "threads.hpp"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>
#include <semaphore>

#include "sharedMemory.hpp"
#include "connection/connection.hpp"
#include "core/geometry.hpp"
#include "pathing/lincontrol.hpp"

#include "config.hpp"
extern Config g_config;


void controller_thread() {
    // Anzahl Nachkommastellen für cout wird gesetzt
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[Controller] Thread started.\n";

    //IP + Port für Command-Interface
    const std::string ip       = g_config.connection.ip;
    const int port_cmd = g_config.connection.port_cmd;

    const double dt_s = 0.1;
    const int dt_ms = static_cast<int>(dt_s * 1000);
    const std::chrono::milliseconds loop_delay(dt_ms);
    const int max_steps = 2000;

    LinearController controller;

    while (true) {

        // stop_flag checken
        if (is_stop_requested()) {
            std::cout << "[Controller] Stop requested, exiting.\n";
            break;
        }
        // ============================
        // Auf ein gültiges Ziel warten
        // ============================
        g_goal_sem.acquire();


        // stop_flag checken
        if (is_stop_requested()) {
            std::cout << "[Controller] Stop requested, exiting.\n";
            break;
        }


        // Startpose und Goalpose aus shared memory holen
        core::Pose2D goal{};
        core::Pose2D pose{};

        {
            std::lock_guard<std::mutex> lock(g_shm_mutex);

            goal.x = g_shm->goal_pose.x;
            goal.y = g_shm->goal_pose.y;
            goal.theta = g_shm->goal_pose.theta;

            pose.x = g_shm->current_pose.x;
            pose.y = g_shm->current_pose.y;
            pose.theta = g_shm->current_pose.theta;

        }

        std::cout << "[INFO] New goal: "
        << "x=" << goal.x << "  y=" << goal.y
        << "  theta=" << goal.theta << "\n";

        // ============================
        // Controller Setup
        // ============================

        controller.setTargetPosition(goal.x, goal.y, goal.theta);
        controller.start();

        int step = 0;

        while(!controller.isGoalReached() && step < max_steps) {

            // stop flag checken
            if (is_stop_requested()) {
                std::cout << "\n[Controller] Stop requested during control loop, aborting.\n";
                break;
            }

            // aktuelle pose aus shared memory holen
            if (g_shm->pose_valid == 0) {
                std::this_thread::sleep_for(loop_delay);
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(g_shm_mutex);
                pose.x = g_shm->current_pose.x;
                pose.y = g_shm->current_pose.y;
                pose.theta = g_shm->current_pose.theta;
            }

            // controller aktuelle Pose ünbergeben
            controller.updateRobotPose(pose.x, pose.y, pose.theta);

            // Controll-Output berechnen
            ControlOutput u = controller.compute_control();

            std::cout << "Step " << step
                          << " | Pose=(" << pose.x << ", " << pose.y << ", " << pose.theta << ")"
                          << " | v=" << u.v << " w=" << u.w << "\r";
            std::cout.flush();


            // command Message
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

        // ============================
        // Stop-Kommando schicken
        // ============================
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

        // Ziel als erreicht markieren

        {
            std::lock_guard<std::mutex> lock(g_shm_mutex);
            g_shm->goal_valid = 0;

        }

        if (is_stop_requested()) {
            break;  
        }





    }
    
}