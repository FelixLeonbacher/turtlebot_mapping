#pragma once
#include <string>
#include "../pathing/lincontrol.hpp"

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


    /**
     * @brief Send a raw message (no START/END added automatically).
     *
     * Opens socket, sends full message, closes socket.
     *
     * @param ip   Target IP
     * @param port Target port
     * @param msg  Message to send
     *
     * @return The message that was transmitted
     *
     * @throws std::runtime_error on socket or send errors
     */
    std::string sendMessage(const std::string& ip, int port, const std::string& msg);


    /**
     * @brief Build a tagged command message:
     *   ---START---{"linear": v, "angular": w}___END___
     *
     * @return Fully framed message (ready for sendMessage)
     */
    std::string buildTaggedControlMessage(double linear, double angular);


    /**
     * @brief Convenience wrapper for ControlOutput struct
     *
     * @param u  ControlOutput with members u.v (linear) and u.w (angular)
     */
    std::string buildTaggedControlMessageFromControlOutput(const ControlOutput& u);
}

