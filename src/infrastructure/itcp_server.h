#pragma once
#include <string>
#include <functional>
class ITcpServer 
{
public:
    virtual ~ITcpServer() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void setCommandHandler(std::function<void(const std::string&)>) = 0;
};