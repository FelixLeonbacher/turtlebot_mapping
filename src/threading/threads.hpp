#pragma once

/**
 * @file threads.hpp
 * @brief thread function declarations
 * @author Merle Rehpeen
 * @version 1.0
 */

#include <string>
#include "sharedMemory.hpp"
#include "config.hpp"


// Network_thread
void network_thread();
void network_shutdown();

// Sensor_thread
void sensor_thread();

// Mapping_thread
void mapping_thread();

// Goal_thread
void goal_thread();

// controller_thread
void controller_thread();