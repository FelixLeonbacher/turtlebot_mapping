#include <iostream>
#include <thread>

#include "sharedMemory.hpp"
#include "threads.hpp"
#include "config.hpp"

Config g_config;

int main()
{

    // ============================
    // CONFIG laden
    // ============================
    if (!loadConfig("../src/config.json", g_config)) {
        std::cerr << "[MAIN][ERROR] Failed to load config. Exiting.\n";
        return 1;
    }


    // ============================
    // SHARED MEMORY initialise
    // ============================
    ipc_init(true);

    if (g_shm != nullptr) {
        std::lock_guard<std::mutex> lock(g_shm_mutex);
        g_shm->network_ok = 0;
    }

    // ============================
    // START NETWORKCHECKER THREAD
    // ============================


    std::thread netThread(network_thread);

    // wait, until networkchecker is ready
    std::cout << "[MAIN] Waiting for network thread to finish...\n" << std::endl;

    netThread.join();

    
    // ============================
    // GOAL THREAD 
    // ============================
    std::thread goalThread(goal_thread);

    // ============================
    // SENSOR THREAD
    // ============================
    std::thread sensorThread(sensor_thread);

    // ============================
    // MAPPING THREAD 
    // ============================
    std::thread mappingThread(mapping_thread);
    // ============================
    // CONTROLLER THREAD 
    // ============================
    std::thread controllerThread(controller_thread);



    sensorThread.join();
    goalThread.join();
    mappingThread.join();
    controllerThread.join();

    std::cout << "All threads finished. Exiting program.\n";

    // ============================
    // NETWORK SHUTDOWN
    // ============================
    network_shutdown();
    // ============================
    // SHARED MEMORY CLEAN UP
    // ============================
    ipc_cleanup();

    return 0;
}