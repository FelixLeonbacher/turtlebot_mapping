#pragma once

#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore>



// --- define the SharedData ---
struct SharedData {
    
};


// declare ID and pointer
extern int shm_id;   // kernel ID of the shared memory segment
extern SharedData* g_shm; // pointer to the structure in shared memory



// initialize shared memory
void ipc_init();

// clean up shared memory
void ipc_cleanup();


