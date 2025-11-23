#pragma once
#include <string>

/**
 * @brief Simple TCP connection helper for Windows.
 * Provides initialization and message reading with START/END tags.
 *
 * Usage:
 *   connection::init();
 *   std::string msg = connection::readTaggedMessage("192.168.100.54", 9997);
 *   connection::shutdown();
 */
namespace connection
{
    /// @brief Initialize Winsock (call once in main)
    void init();

    /// @brief Cleanup Winsock (call once before exit)
    void shutdown();

    /**
     * @brief Connect to IP:port and read one complete tagged message.
     * Message must be framed by:
     *   ---START---   ...   ___END___
     *
     * @param ip   Target IP (e.g. "192.168.100.54")
     * @param port Target TCP port (e.g. 9997 or 9998)
     * @return std::string Raw message including tags
     *
     * @throws std::runtime_error on failure
     */
    std::string readTaggedMessage(const std::string& ip, int port);
}

