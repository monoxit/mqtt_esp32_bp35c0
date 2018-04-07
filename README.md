# mqtt_esp32_bp35c0
An Arduino sample sketch - Simple smartmeter MQTT gateway.
# 簡単なスマート電力計MQTTゲートウェイ
Arduinoのサンプルスケッチ - ESP32とBP35C0を使ったスマート電力計とMQTTの簡単なゲートウェイです。
# 使い方
スケッチと同じフォルダにprivate.hを作成し、private.hにSSID等を設定してからESP32へスケッチを書き込みます。
## private.hの例
```
#define YOUR_SSID "YOURSSID"
#define YOUR_WIFI_PASSWORD "YOURPSWD"
#define MQTT_SERVER "mqtt.example.com"
#define MQTT_PORT 8883
#define MQTT_USER "mqttuserid"
#define MQTT_PASSWORD "mqttpassword"
#define BROUTE_ID "XXX000XXXXXX0000XXX000XXXXXXXXX"
#define BROUTE_PASSWORD "5XXXXXXXXXA"
```
