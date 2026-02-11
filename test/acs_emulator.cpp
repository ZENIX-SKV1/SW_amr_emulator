#define WIN32_LEAN_AND_MEAN
// #include <winsock2.h>
// #include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

// #pragma comment(lib, "Ws2_32.lib")

int main(int argc, char* argv[]) 
{
    std::string config_path = "C:/Users/ZENIX/Documents/src/amr_emulator/config/amr_params.yaml";
    if (argc > 1) config_path = argv[1];

    YAML::Node config = YAML::LoadFile(config_path);
    int base_port = config["base_port"].as<int>();
    int amr_count = config["amr_count"].as<int>();

    std::cout << "base_port: " << base_port << ", amr_count: " << amr_count << std::endl;

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) 
    {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }

    const char* server_ip = "127.0.0.1";

    for (int i = 0; i < amr_count; ++i) 
    {
        int port = base_port + i;
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) 
        {
            std::cerr << "[AMR " << i << "] socket() failed\n";
            continue;
        }

        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, server_ip, &server_addr.sin_addr);

        if (connect(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR)
        {
            std::cerr << "[AMR " << i << "] connect() failed (port " << port << ")\n";
            closesocket(sock);
            continue;
        }

        std::string command = "MOVE 1.0 2.0";
        int sent = send(sock, command.c_str(), (int)command.size(), 0);
        if (sent == SOCKET_ERROR) 
        {
            std::cerr << "[AMR " << i << "] send() failed\n";
            closesocket(sock);
            continue;
        }
        std::cout << "[AMR " << i << "] Sent command to port " << port << "\n";

        char buf[512];
        int received = recv(sock, buf, sizeof(buf)-1, 0);
        if (received > 0) 
        {
            buf[received] = '\0';
            std::cout << "[AMR " << i << "] Received: " << buf << "\n";
        } 
        else 
        {
            std::cout << "[AMR " << i << "] No response or connection closed\n";
        }

        closesocket(sock);
    }

    WSACleanup();
    return 0;
}