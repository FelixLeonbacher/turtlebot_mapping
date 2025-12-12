/**
 * @author Philipp Riegler
 * @brief Simple TCP connection helper for Windows.
 * Provides initialization and message reading with START/END tags.
 *
 * Usage:
 *   connection::init();
 *   std::string msg = connection::readTaggedMessage("192.168.100.54", 9997);
 *   connection::shutdown();
 */

#include "connection.hpp"

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>


#include <stdexcept>
#include <sstream>

// Link Winsock automatically when using MSVC
#pragma comment(lib, "Ws2_32.lib")

namespace connection
{
    void init()
    {
        WSADATA wsa; // Windows Sockets API 
        if (WSAStartup(MAKEWORD(2,2), &wsa) != 0)
            throw std::runtime_error("WSAStartup() failed");
    }

    void shutdown()
    {
        WSACleanup();
    }

    std::string readTaggedMessage(const std::string& ip, int port)
    {
        const std::string START_TAG = "---START---";
        const std::string END_TAG   = "___END___";

        // 1. Create socket
        SOCKET sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET)
            throw std::runtime_error("socket() failed");

        // 2. Setup sockaddr
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
            closesocket(sock);
            throw std::runtime_error("inet_pton() failed");
        }

        // 3. Connect
        if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
            closesocket(sock);
            throw std::runtime_error("connect() failed");
        }

        // 4. Read loop
        std::string buffer;
        char temp[4056];

        for (;;) {
            int n = recv(sock, temp, sizeof(temp), 0);

            if (n == SOCKET_ERROR) {
                closesocket(sock);
                throw std::runtime_error("recv() failed");
            }

            if (n == 0) {
                closesocket(sock);
                throw std::runtime_error("connection closed before receiving full message");
            }

            buffer.append(temp, n);

            // Try to find full message
            auto startPos = buffer.find(START_TAG);
            if (startPos == std::string::npos)
                continue;

            auto endPos = buffer.find(END_TAG, startPos + START_TAG.size());
            if (endPos == std::string::npos)
                continue;

            // Extract full message
            std::size_t msgEnd = endPos + END_TAG.size();
            std::string msg = buffer.substr(startPos, msgEnd - startPos);

            closesocket(sock);
            return msg;
        }
    }

    std::string sendMessage(const std::string& ip, int port, const std::string& msg)
    {

        // 1. Create socket
        SOCKET sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET)
            throw std::runtime_error("sendTaggedMessage: socket() failed");

        // 2. Fill sockaddr_in
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
            closesocket(sock);
            throw std::runtime_error("sendTaggedMessage: inet_pton() failed");
        }

        // 3. Connect
        if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
            closesocket(sock);
            throw std::runtime_error("sendTaggedMessage: connect() failed");
        }

        // 4. Send loop (ensure all bytes are sent)
        const char* data = msg.data();
        int total = 0;
        int toSend = static_cast<int>(msg.size());

        while (total < toSend) {
            int n = ::send(sock, data + total, toSend - total, 0);
            if (n == SOCKET_ERROR) {
                closesocket(sock);
                throw std::runtime_error("sendTaggedMessage: send() failed");
            }
            total += n;
        }

        // 5. Close socket and return ack string
        closesocket(sock);
        return msg;
    }


    std::string buildTaggedControlMessage(double linear, double angular)
    {
        const std::string START_TAG = "---START---";
        const std::string END_TAG   = "___END___";

        std::ostringstream oss;
        oss << START_TAG
            << "{\"linear\":" << linear << ",\"angular\":" << angular << "}"
            << END_TAG;

        return oss.str();
    }


    std::string buildTaggedControlMessageFromControlOutput(const ControlOutput& u)
    {
        return buildTaggedControlMessage(u.v, u.w);
    }

} // namespace connection