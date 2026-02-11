#pragma once
#include "iprotocol.h"
#include "iamr.h"
#include <string>
#include <iostream>

class CustomTcpProtocol : public IProtocol 
{
public:
    CustomTcpProtocol();
    void setAmr(IAmr* amr);
    std::string getProtocolType() const override { return "custom_tcp"; }
    void handleMessage(const std::string& msg, IAmr* amr) override;
    std::string makeStateMessage(IAmr* amr) override;
    void start() override;
};