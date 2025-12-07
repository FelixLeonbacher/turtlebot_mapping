#include <iostream>
#include <thread>

#include "sharedMemory.hpp"
#include "threads.hpp"


int main()
{
    // ============================
    // SHARED MEMORY initialise
    // ============================
    ipc_init();

    if (g_shm != nullptr) {
        g_shm->network_ok = 0;
    }

    // ============================
    // START NETWORKCHECKER THREAD
    // ============================

    std::cout << "[MAIN] Starting network checker thread...\n" << std::endl;

    std::thread netThread(network_thread);

    // wait, until networkchecker is ready
    std::cout << "[MAIN] Waiting for network thread to finish...\n" << std::endl;

    netThread.join();

    // ============================
    // SENSOR THREAD
    // ============================
    std::thread sensorThread(sensor_thread);

    // ============================
    // MAPPING THREAD 
    // ============================

    // ============================
    // CONTROLLER THREAD 
    // ============================

    // ============================
    // GOAL THREAD 
    // ============================

    // ============================
    // NETWORK SHUTDOWN
    // ============================
    network_shutdown();
    // ============================
    // SHARED MEMORY CLEAN UP
    // ============================
    ipc_cleanup;

    return 0;
}