import socket

HOST = '127.0.0.1'
PORT = 8080

json_msg = '''{
  "headerId": "header_test",
  "timestamp": 1650000000,
  "orderId": "order_test",
  "nodes": [
    {"nodeId": "N1", "sequenceId": 0, "nodePosition": {"x":0, "y":0, "theta":0}},
    {"nodeId": "N2", "sequenceId": 1, "nodePosition": {"x":1, "y":0, "theta":0}},
    {"nodeId": "N3", "sequenceId": 2, "nodePosition": {"x":1, "y":1, "theta":0}},
    {"nodeId": "N4", "sequenceId": 3, "nodePosition": {"x":0, "y":1, "theta":0}},
    {"nodeId": "N5", "sequenceId": 4, "nodePosition": {"x":0, "y":0, "theta":0}}
  ],
  "edges": [
    {"edgeId": "E1", "sequenceId": 0, "startNodeId": "N1", "endNodeId": "N2"},
    {"edgeId": "E2", "sequenceId": 1, "startNodeId": "N2", "endNodeId": "N3"},
    {"edgeId": "E3", "sequenceId": 2, "startNodeId": "N3", "endNodeId": "N4"},
    {"edgeId": "E4", "sequenceId": 3, "startNodeId": "N4", "endNodeId": "N5"}
  ]
}\n'''  # 개행 포함

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(json_msg.encode())

