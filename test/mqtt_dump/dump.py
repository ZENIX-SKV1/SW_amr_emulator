import paho.mqtt.client as mqtt
from datetime import datetime

# 로그 파일 열기 (append 모드)
log_file = open("mqtt_dump.log", "a", encoding="utf-8")

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("#")  # 모든 토픽 구독

def on_message(client, userdata, msg):
    # 밀리초 단위까지 포함된 timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    log_entry = (
        f"[{timestamp}]\n"
        f"Topic: {msg.topic}\n"
        f"QoS: {msg.qos}, Retain: {msg.retain}\n"
        f"Payload: {msg.payload.decode(errors='replace')}\n"
        "----------------------------------------\n"
    )

    # 콘솔 출력
    print(log_entry, end="")

    # 파일에 기록
    log_file.write(log_entry)
    log_file.flush()

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("test.mosquitto.org", 1883, 60)  # 실제 브로커 주소로 변경

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("Stopping client...")
finally:
    log_file.close()
