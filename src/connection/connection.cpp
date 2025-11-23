#include "connection.hpp"

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdexcept>

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
        char temp[2048];

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
}
