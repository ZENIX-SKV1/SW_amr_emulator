#!/bin/bash

# Ctrl + C가 눌렸을 때 실행할 함수 정의
cleanup() {
    echo "종료 중... 모든 프로세스를 정지합니다."
    # 현재 스크립트에서 파생된 모든 백그라운드 프로세스 종료
    kill $(jobs -p) 2>/dev/null
    exit
}

# SIGINT(Ctrl+C) 신호를 받으면 cleanup 함수 호출
trap cleanup SIGINT


rviz2 &

sleep 1

./build/third_party/driving_viewer/driving_viewer ./config/amr_params.yaml &

sleep 3

./build/amr_emulator ./config/amr_params.yaml &

sleep 3

echo "publish a order"

mosquitto_pub -h localhost -p 1883 -t "agv/v2/ZENIXROBOTICS/0000/order" -f ./test/fms_orders/curve.json

echo "모든 프로세스가 실행 중입니다. 중단하려면 Ctrl+C를 누르세요."

wait
