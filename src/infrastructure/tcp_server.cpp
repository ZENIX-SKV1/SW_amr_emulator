#include "tcp_server.h"
#include <iostream>
#include <vector>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <unistd.h>
#endif

TcpServer::TcpServer(int port) : port_(port), running_(false) 
{

}

TcpServer::~TcpServer() 
{
    stop();
}

void TcpServer::setCommandHandler(std::function<void(const std::string&)> handler) 
{
    onCommand_ = handler;
}

void TcpServer::start()
{
    running_ = true;
    server_thread_ = std::thread(&TcpServer::run, this);
}

void TcpServer::stop() 
{
    running_ = false;
    if (server_thread_.joinable()) server_thread_.join();
}

void TcpServer::run() 
{
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) 
    {
        std::cerr << "[TcpServer] WSAStartup failed!" << std::endl;
        return;
    }
#endif

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) 
    {
        std::cerr << "[TcpServer] socket() failed" << std::endl;
        return;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);

    int opt = 1;
    #ifdef _WIN32
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
    #else
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    #endif

    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) 
    {
        std::cerr << "[TcpServer] bind() failed" << std::endl;
#ifdef _WIN32
        closesocket(server_fd);
        WSACleanup();
#else
        close(server_fd);
#endif
        return;
    }

    if (listen(server_fd, 5) < 0) 
    {
        std::cerr << "[TcpServer] listen() failed" << std::endl;
#ifdef _WIN32
        closesocket(server_fd);
        WSACleanup();
#else
        close(server_fd);
#endif
        return;
    }

    std::cout << "[TcpServer] Listening on port " << port_ << std::endl;

    while (running_) 
    {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
#ifdef _WIN32
        SOCKET client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
#else
        int client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
#endif
        if (client_fd < 0) 
        {
            if (running_) std::cerr << "[TcpServer] accept() failed" << std::endl;
            continue;
        }
        else
        {
           std::cout << "[TcpServer]  Listening() success " << port_ << std::endl; 
        }

        char buffer[1024] = {0};
#ifdef _WIN32
        int bytes = recv(client_fd, buffer, sizeof(buffer)-1, 0);
#else
        int bytes = read(client_fd, buffer, sizeof(buffer)-1);
#endif
        if (bytes > 0 && onCommand_) {
            std::string cmd(buffer, bytes);
            onCommand_(cmd);
        }

#ifdef _WIN32
        closesocket(client_fd);
#else
        close(client_fd);
#endif
    }

#ifdef _WIN32
    closesocket(server_fd);
    WSACleanup();
#else
    close(server_fd);
#endif
}