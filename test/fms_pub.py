import paho.mqtt.client as mqtt

mqtt = mqtt.Client("FMS_Pub")
mqtt.username_pw_set("username", "password")  # 필요할 경우
mqtt.connect("localhost", 1883)

mqtt.publish("fms/test", "테스트 메시지입니다.")
mqtt.loop()  # 또는 mqtt.loop_start()
print("Published!")
