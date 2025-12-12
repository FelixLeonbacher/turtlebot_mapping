#include "sharedMemory.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <semaphore>
#include <iostream>

std::mutex g_shm_mutex;
std::binary_semaphore g_scan_sem(0);
std::binary_semaphore g_goal_sem(0);



// Gemeinsamen Speicherbereich unter Windows anlegen (Shared memory)
// Zeiger (g_shm) darauf merken, um auf Daten zugreifen zu können
// Räumt zum Schluss wieder auf

#ifdef _WIN32
#include <windows.h>

static HANDLE g_hMapFile = nullptr;     // Referenz für mein Shared-memory Objekt
SharedData* g_shm = nullptr;

// Namen für shared memory festlegen
const char* shm_name = "turtlebot_shared_mem";


// --- initialize shared memory ---
void ipc_init(bool creator)
{
    if (creator) {
        // Shared Memory erzeugen mit File Mapping 
        g_hMapFile = CreateFileMappingA(
            INVALID_HANDLE_VALUE,          // keine echte Datei auf Festplatte -> Daten liegen nur im RAM
            nullptr,                       // Default Security
            PAGE_READWRITE,                // Lesen + Schreiben erlaubt
            0,                             // Größe vom shared memory Block bestimmen
            sizeof(SharedData),            // 
            shm_name                       // Mapping Name
        );

        if(!g_hMapFile) {
            std::cerr << "CreateFileMappingA failed, error code = " << GetLastError() << '\n';
            std::exit(EXIT_FAILURE);
            
        }

        
        // attach shared memory to my process
        // return pointer auf den Speicher in meinem Prozess
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

        // alles auf null setzen
        std::memset(g_shm, 0, sizeof(SharedData));
    
    } else {

        // wenn Mapping bereits existiert nut noch öffnen
        g_hMapFile = OpenFileMappingA(
            FILE_MAP_ALL_ACCESS,
            FALSE,
            shm_name
        );

        if (!g_hMapFile) {
            std::cerr << "OpenFileMappingA failed, error = " << GetLastError() << '\n';
            std::exit(EXIT_FAILURE);
        }

        // attach shared memory to my process
        // return pointer auf den Speicher in meinem Prozess
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
    // detach shared memory segment from address space
    if (g_shm) {
        UnmapViewOfFile(g_shm);
        g_shm = nullptr;
    }
    // Handle wieder schließen
    if (g_hMapFile) {
        CloseHandle(g_hMapFile);
        g_hMapFile = nullptr;
    }
}


// --- Hilfsfunktionen ---
void request_global_stop()
{
    {
        std::lock_guard<std::mutex> lock(g_shm_mutex);
        g_shm->stop= 1;   // 1 = Stopp
    }

    // Threads aufwecken, die evtl. gerade auf Semaphoren schlafen
    g_goal_sem.release();   
    g_scan_sem.release();  

}

bool is_stop_requested()
{
    std::lock_guard<std::mutex> lock(g_shm_mutex);
    return g_shm->stop != 0;
}

bool has_valid_pose()
{
    std::lock_guard<std::mutex> lock(g_shm_mutex);
    return (g_shm && g_shm->pose_valid != 0);
}



#endif


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

/**#else

// === einfache Thread-Test-Variante (globales Objekt) ===
static SharedData g_local_shm;
SharedData* g_shm = &g_local_shm;

void ipc_init() {
    std::memset(&g_local_shm, 0, sizeof(g_local_shm));
    std::printf("[SHM] Using local SharedData instance (no System-V).\n");
}

void ipc_cleanup() {
    g_shm = &g_local_shm;
}**/

#endif
