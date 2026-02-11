#pragma once

#include "amr.h"
#include "tcp_server.h"
#include "yaml_config.h"
#include "iprotocol.h"
#include "vda5050_protocol.h"
#include "custom_tcp_protocol.h"

#include <vector>
#include <memory>
#include <string>

class AmrManager 
{
public:
    AmrManager(const AmrConfig& config);

    void startAll();
    void stopAll();

    std::vector<std::unique_ptr<Amr>>& getAmrs() 
    { 
        return amrs_; 
    }
    
    // 등록된 프로토콜의 개수를 반환합니다.
    size_t getProtocolCount() const;

    // 특정 인덱스의 프로토콜을 가져오는 메서드
    IProtocol* getProtocol(size_t index) const;    

private:
    std::vector<std::unique_ptr<Amr>> amrs_;
    std::vector<std::unique_ptr<TcpServer>> servers_;
    std::vector<std::unique_ptr<IProtocol>> protocols_;
    bool isVda5050OrderMessage(const std::string& msg);
    bool isCustomTcpProtocolMessage(const std::string& msg); 

    // 개별 AMR 생성 함수
    std::unique_ptr<Amr> createSingleAmr(int id, const AmrConfig& config);

    // 프로토콜 생성 및 초기설정 함수
    std::unique_ptr<IProtocol> createProtocol(const std::string& protocol_type, const std::string& server_address, const std::string& agv_id, Amr* amr, const AmrConfig& config);

    // TCP 서버 생성 및 핸들러 연결 함수
    void setupTcpServer(int port, int amr_idx);        
    
    AmrConfig config_;
};