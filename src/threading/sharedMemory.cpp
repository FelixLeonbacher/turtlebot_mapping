#include "sharedMemory.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>


#ifdef USE_SYSTEMV_SHM

#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore>





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
    char* addr = (char*) shmat(shm_id, nullptr, 0);  
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

#else

// === einfache Thread-Test-Variante (globales Objekt) ===
static SharedData g_local_shm;
SharedData* g_shm = &g_local_shm;

void ipc_init() {
    std::memset(&g_local_shm, 0, sizeof(g_local_shm));
    std::printf("[SHM] Using local SharedData instance (no System-V).\n");
}

void ipc_cleanup() {
    g_shm = &g_local_shm;
}

#endif
