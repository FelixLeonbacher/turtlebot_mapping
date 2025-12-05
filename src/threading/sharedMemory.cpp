#include "sharedMemory.hpp"
#include <cstdio>
#include <cstdlib>


int shm_id = -1;
SharedData* g_shm = nullptr;


// --- initialize shared memory ---
void ipc_init() 
{
    key_t key = 1234;   // key for the shared memory segment

    // --- create System-V shared memory ---

    // find or create shared memory segment
    shm_id = shmget(key, sizeof(SharedData), IPC_CREAT | 0666);   // 0666: permissions, everyone can read and write
    if (shm_id == -1) {
        perror("shmget");
        std::exit(EXIT_FAILURE);
    }

    // attach shared memory to my process
    char* addr = (char*) shmat(shm_id, nullptr, 0);  // choose a suitable address
    if (addr == (char*) -1) {
        perror("shmat");
        std::exit(EXIT_FAILURE);
    }

    g_shm = reinterpret_cast<SharedData*>(addr);  // cast pointer to SharedData*
}


// --- at the end of the program, clean up shared memory ---
void ipc_cleanup()
{
    if (g_shm != nullptr) {
        shmdt(g_shm);           // detach shared memory segment from address space
        g_shm = nullptr;
    }
    if (shm_id != -1) {
        shmctl(shm_id, IPC_RMID, nullptr);    // delete shared memory segment
        shm_id = -1;
    }
}
