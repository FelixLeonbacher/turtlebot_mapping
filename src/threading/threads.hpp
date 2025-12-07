#pragma once

#include <string>
#include "sharedMemory.hpp"

// Network_thread
void network_thread();
void network_shutdown();

// Sensor_thread
void sensor_thread();

// Mapping_thread
void mapping_thread();