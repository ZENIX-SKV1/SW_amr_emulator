#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation error!\n";
        return 1;
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8080); // 서버 포트

    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address\n";
        return 1;
    }

    if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed\n";
        return 1;
    }

    std::string json_msg = R"({
        "headerId": "header_test",
        "timestamp": 1650000000,
        "orderId": "order_test",
        "nodes": [
            {"nodeId": "N1", "sequenceId": 0, "nodePosition": {"x":0, "y":0, "theta":0}},
            {"nodeId": "N2", "sequenceId": 1, "nodePosition": {"x":1, "y":0, "theta":0}},
            {"nodeId": "N3", "sequenceId": 2, "nodePosition": {"x":1, "y":1, "theta":1.57}},
            {"nodeId": "N4", "sequenceId": 3, "nodePosition": {"x":0, "y":1, "theta":3.14}},
            {"nodeId": "N5", "sequenceId": 4, "nodePosition": {"x":0, "y":0, "theta":0}}
        ],
        "edges": [
            {"edgeId": "E1", "sequenceId": 0, "startNodeId": "N1", "endNodeId": "N2"},
            {"edgeId": "E2", "sequenceId": 1, "startNodeId": "N2", "endNodeId": "N3"},
            {"edgeId": "E3", "sequenceId": 2, "startNodeId": "N3", "endNodeId": "N4"},
            {"edgeId": "E4", "sequenceId": 3, "startNodeId": "N4", "endNodeId": "N5"}
        ]
    })\n";  // 끝에 개행문자 넣기 중요

    send(sock, json_msg.c_str(), json_msg.length(), 0);

    close(sock);

    return 0;
}


