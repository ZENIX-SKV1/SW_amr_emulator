#pragma once
#include "itcp_server.h"
#include <thread>
#include <atomic>
#include <functional>
class TcpServer : public ITcpServer 
{
public:
    TcpServer(int port);
    ~TcpServer();
    void start() override;
    void stop() override;
    void setCommandHandler(std::function<void(const std::string&)>) override;
private:
    int port_;
    std::thread server_thread_;
    std::atomic<bool> running_;
    std::function<void(const std::string&)> onCommand_;
    void run();
};