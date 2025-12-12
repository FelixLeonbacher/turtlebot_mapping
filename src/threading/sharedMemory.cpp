#include "sharedMemory.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <semaphore>
#include <iostream>
#include <windows.h>


std::mutex g_shm_mutex;
std::binary_semaphore g_scan_sem(0);
std::binary_semaphore g_goal_sem(0);

/// Handle to the windows file mapping object for shared memory
static HANDLE g_hMapFile = nullptr;     

/// Global pointer to the shared memory region
SharedData* g_shm = nullptr;

/// of the shared memory mapping
const char* shm_name = "turtlebot_shared_mem";


void ipc_init(bool creator)
{
    if (creator) {
        // Create shared memory mapping in RAM
        g_hMapFile = CreateFileMappingA(
            INVALID_HANDLE_VALUE,          // no actual file -> memory-only
            nullptr,                       // default security
            PAGE_READWRITE,                // read + write
            0,                             // 
            sizeof(SharedData),            // 
            shm_name                       // mapping name
        );

        if(!g_hMapFile) {
            std::cerr << "CreateFileMappingA failed, error code = " << GetLastError() << '\n';
            std::exit(EXIT_FAILURE);
            
        }

        
        // Map the shared memory into the process address space
        void* addr = MapViewOfFile(
            g_hMapFile,
            FILE_MAP_ALL_ACCESS,
            0,
            0,
            sizeof(SharedData)
        );

        if (!addr) {
            std::cerr << "MapViewOfFile failed, error code = " << GetLastError() << '\n';
            CloseHandle(g_hMapFile);
            g_hMapFile = nullptr;
            std::exit(EXIT_FAILURE);
        }

        // cast pointer to shared memory
        g_shm = static_cast<SharedData*>(addr);

        // initialize all fiels to zero
        std::memset(g_shm, 0, sizeof(SharedData));
    
    } else {

        // Attach to an existing shared memory mapping
        g_hMapFile = OpenFileMappingA(
            FILE_MAP_ALL_ACCESS,
            FALSE,
            shm_name
        );

        if (!g_hMapFile) {
            std::cerr << "OpenFileMappingA failed, error = " << GetLastError() << '\n';
            std::exit(EXIT_FAILURE);
        }

        // Map the shared memory into the process address space
        void* addr = MapViewOfFile(
            g_hMapFile,
            FILE_MAP_ALL_ACCESS,
            0,
            0,
            sizeof(SharedData)
        );

        if (!addr) {
            std::cerr << "MapViewOfFile failed, error code = " << GetLastError() << '\n';
            CloseHandle(g_hMapFile);
            g_hMapFile = nullptr;
            std::exit(EXIT_FAILURE);
        }

        // cast pointer to shared memory
        g_shm = static_cast<SharedData*>(addr);

    }

    


}

void ipc_cleanup() {

    // Detach view of the shared memory region
    if (g_shm) {
        UnmapViewOfFile(g_shm);
        g_shm = nullptr;
    }
    // Close the Windows handle for the mapping
    if (g_hMapFile) {
        CloseHandle(g_hMapFile);
        g_hMapFile = nullptr;
    }
}


// --- helper functions used by all thread ---
void request_global_stop()
{
    {
        std::lock_guard<std::mutex> lock(g_shm_mutex);
        g_shm->stop= 1;   // global stop request
    }

    // Wake up threads that might be blocked on semaphores
    g_goal_sem.release();   
    g_scan_sem.release();  

}

bool is_stop_requested()
{
    std::lock_guard<std::mutex> lock(g_shm_mutex);
    return (g_shm && g_shm->stop != 0);
}

bool has_valid_pose()
{
    std::lock_guard<std::mutex> lock(g_shm_mutex);
    return (g_shm && g_shm->pose_valid != 0);
}


