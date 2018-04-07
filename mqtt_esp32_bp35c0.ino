/*
 * mqtt_esp32_bp35c0.ino
 * Simple ESP32 + BP35C0 example
 * Copyright (c) 2018, Masami Yamakawa <silkycove@gmail.com>, except where otherwise indicated.
 * The callback(), setup_wifi() and reconnect() functions are:
 *  Copyright (c) 2008-2015 Nicholas O'Leary
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 * 
 * BP35C0とESP32を使用した、瞬時電力、瞬時電流ならびに定時積算電力を取得し、MQTTでパブリッシュするサンプルスケッチ。
 * ノート：積算電力は定時積算電力を取得しているためパブリッシュ時点での積算電力でない。
 * 　　　　合成構成比ならびに有効桁数を考慮していない。
*/

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "private.h"

// Update these with values suitable for your network.

const char* ssid = YOUR_SSID;
const char* password = YOUR_WIFI_PASSWORD;
const char* mqttServer = MQTT_SERVER;
const int   mqttPort=MQTT_PORT;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASSWORD;

const char* bRouteId = BROUTE_ID;
const char* bRoutePassword = BROUTE_PASSWORD;

const char* deviceId = "BP35C0";
const char* mqttPowerTopic = "school/wisun/power"; //瞬時電力トピック
const char* mqttIntegralPowerTopic = "school/wisun/integral"; //積算電力トピック
const char* mqttStateTopic = "school/wisun/state"; //スマートメーター無線接続状態トピック
const char* mqttCtrlTopic = "school/wisun/ctrl"; //将来使用

const uint8_t ledPin = 13;
const uint8_t BP35C0ResetPin = 23;

const uint16_t  resetWaitMillis = 3 * 1000; //[ms]
const uint16_t  resTimeoutMillis = 15 * 1000; //応答タイムアウト[ms]
const uint32_t  scanTimeoutMillis = 3 * 60 * 1000; //ステーションスキャンタイムアウト[ms]
const uint32_t  immediatePowerInterval = 15 * 1000; //瞬時電力取得間隔[ms]
const uint32_t  integralPowerInterval = 5 * 60 * 1000; //積算電力取得間隔[ms]
const uint8_t   maxErrorNumber = 3;

uint8_t   bp35c0State = 0;

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
unsigned long now;
unsigned long reqMillis;
unsigned long lastImmediatePowerMillis;
unsigned long lastIntegralPowerMillis;

int32_t   lastImmediatePower=0x7FFFFFFD;
int32_t   currentImmediatePower=0x7FFFFFFD;
int16_t   lastImmediateCurrentR=0x7FFE;
int16_t   currentImmediateCurrentR=0x7FFE;
int16_t   lastImmediateCurrentT=0x7FFE;
int16_t   currentImmediateCurrentT=0x7FFE;
uint32_t  lastIntegralPower=0xFFFFFFFF;
uint32_t  currentIntegralPower=0xFFFFFFFF;
float     integralPowerFactor = 0.1;

char bp35c0Buffer[2048];
uint16_t bp35c0BufferCounter;
uint8_t errorCounter = 0;

String channel;
String panid;
String addr;
String ipv6addr;
uint16_t tid;
int     ledDefaultState = HIGH;

uint8_t echonetReqData[] = {
    0x10, //EHD1:ECHONET Lite
    0x81, //EHD2:規定形式
    0x00, //TID[2]:トランザクションID
    0x01,
    0x05, //SEOJ[3]:送信元オブジェクト:コントローラー
    0xFF,
    0x01,
    0x02, //DEOJ[3]:送信先オブジェクト：スマートメーター
    0x88,
    0x01,
    0x62, //ESV:プロパティ読み出し
    0x01, //OPC:プロパティ数
    0xE7, //EPC:瞬時電力測定値
    0x00, //PDC:プロパティデータサイズ
    0xE1, //EPC:積算電力単位（オプション）
    0x00  //PDC:プロパティデータサイズ（オプション）
  };

float integralPowerFactorTable[] = {1,0.1,0.01,0.001,0.0001,0,0,0,0,0,10,100,1000,10000};

HardwareSerial Serial1(1); // UART1 ソフトリセット後UART2不良問題回避のためUART1を使用

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledDefaultState);
  Serial.begin(115200);
  pinMode(BP35C0ResetPin,OUTPUT);

  Serial1.begin(115200, SERIAL_8N1, 16, 17); //RX:IO16, TX:IO17
  // Set Serial1 input stream timeout to 3s (default 1s).
  Serial1.setTimeout(3000);
  while(!bp35c0SetAsciiDumpMode()){
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(1000);
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  bp35c0State = 0;
  errorCounter = 0;
  tid = 0;
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledPin,!digitalRead(ledPin));
    delay(250);
    digitalWrite(ledPin,!digitalRead(ledPin));
    delay(250);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Future use
}

void reconnect() {
  // Loop until we're reconnected
  for(int i = 0; i < 5 && !client.connected(); i++){
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(deviceId, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      char* msg = "OFF";
      if(bp35c0State >=18) msg = "ON";
      client.publish(mqttStateTopic, msg);
      // ... and resubscribe
      client.subscribe(mqttCtrlTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      for(int i = 0; i < 2; i++){
      digitalWrite(ledPin,!digitalRead(ledPin));
      delay(250);
      digitalWrite(ledPin,!digitalRead(ledPin));
      delay(250);
      }
      delay(4500);
    }
  }
  //MQTTブローカーに接続できないときはマイコンをリセット
  if(!client.connected()) ESP.restart();
}

bool bp35c0SetAsciiDumpMode(){
  Serial.println("Reseting BP35C0...");
  //Reset BP35C0
  digitalWrite(BP35C0ResetPin,LOW);
  delay(10);
  digitalWrite(BP35C0ResetPin,HIGH);
  delay(3000);
  
  while(Serial1.available()) Serial1.read();
  Serial.print("ROPT:");
  Serial1.println("ROPT");
  if(Serial1.find("OK ")){
    char optChars[3];
    optChars[0] = Serial1.read(); // Read OPT high byte
    optChars[1] = Serial1.read(); // Read OPT low byte
    optChars[2] = '\0';
    Serial.println(optChars);
    unsigned int opt;
    sscanf(optChars, "%x", &opt);// Convert hex string into integer.

    if((opt & 0x01) == 0x00){
      Serial.println("Setting ascii dump mode");
      opt = opt | 0x01;
      char optCmd[8];
      snprintf(optCmd, sizeof(optCmd), "WOPT %02X", opt);
      Serial.println(optCmd);
      Serial1.println(optCmd);
      if(!Serial1.find("OK")) return false;
    }else return true;
  }else return false;
}

bool isResReceived(String expectedStr){
  String  str = String(bp35c0Buffer);
  int     expectedStrIndex = str.indexOf(expectedStr);
  if(expectedStrIndex != -1){
    if(str.indexOf("\r\n",expectedStrIndex) != -1) return true;
    else return false;
  }else return false;
}

void resetBp35c0Buffer(){
  bp35c0BufferCounter = 0;
  bp35c0Buffer[bp35c0BufferCounter] = '\0';
  while(Serial1.available()) Serial1.read();
}

void bp35c0SendCommand(String cmd, String data = ""){
  resetBp35c0Buffer();
  if(data.length() > 0){
    Serial1.print(cmd);
    Serial1.println(data);
  }else{
    Serial1.println(cmd);    
  }
  reqMillis = now;
}

void stateTransition(int newState){
  if(bp35c0State < 18 && newState >= 18){
    client.publish(mqttStateTopic, "ON");
    ledDefaultState = LOW;
  }else if(bp35c0State >= 18 && newState < 18){
    client.publish(mqttStateTopic, "OFF");
    ledDefaultState = HIGH;
  }
  bp35c0State = newState;
  Serial.print("STATE:");
  Serial.println(bp35c0State);
}

void waitForResponse(String expectedStr){
  if(isResReceived(expectedStr)){
    stateTransition(bp35c0State+1);
  }else if((unsigned long)(now - reqMillis) > resTimeoutMillis){
    Serial.println("ERROR: No response received... Reset BP35C0");
    stateTransition(0);
 }
}

void handleEpandesc(){
  String str = String(bp35c0Buffer);
  bool error = false;
  if(str.indexOf("EVENT 22") != -1){
    // Scan completed
    if(str.indexOf("EPANDESC") != -1){
      // Found smartmeter!
      int offset = str.indexOf("Channel:");

      channel = str.substring(offset+8, offset+10);
      panid   = str.substring(offset+40, offset+44);
      addr    = str.substring(offset+53, offset+69);

      Serial.println(str);
      Serial.print("Channel:");
      Serial.println(channel);
      Serial.print("PanID:");
      Serial.println(panid);
      Serial.print("Addr:");
      Serial.println(addr);

      errorCounter=0;
      stateTransition(bp35c0State+1);
    }else error = true;
  }else if((unsigned long)(now - reqMillis) > scanTimeoutMillis){
    error = true;
  }

  if(error){
    Serial.println("ERROR:  Smartmeter not found...");
    errorCounter++;
    stateTransition(bp35c0State-1);
 }
}

void handleIpv6addr(){
  String str = String(bp35c0Buffer);
  int separatorIndex = str.indexOf(':');
  if(separatorIndex != -1){
    if(str.indexOf("\r\n",separatorIndex) != -1){
      int offset = separatorIndex - 4;
      ipv6addr = str.substring(offset, offset + 39);
    }
  }
  if(ipv6addr.length() >= 39){
    //Got ipv6 addr.
    Serial.print("IPv6addr:");
    Serial.println(ipv6addr);
    errorCounter=0;
    stateTransition(bp35c0State+1);
  }else if((unsigned long)(now - reqMillis) > resTimeoutMillis){
    Serial.println("ERROR: Cannot get IPv6 Addr");
    errorCounter++;
    stateTransition(bp35c0State-1);
  }
}

void sendEchonetReq(uint8_t epc, uint8_t epc2=0xFF){
  int dataSize = 14;
  char prefixStr[16];
  
  tid++;
  echonetReqData[2] = tid >> 8;
  echonetReqData[3] = tid & 0xFF;
  echonetReqData[12] = epc;
  if(epc2 != 0xFF){
    echonetReqData[11] = 2; //OPC = 2
    echonetReqData[14] = epc2;
    dataSize = 16;
  }else echonetReqData[11] = 1; //OPC = 1
  
  resetBp35c0Buffer();
  
  Serial1.print("SKSENDTO 1 ");
  Serial1.print(ipv6addr);
  snprintf(prefixStr, sizeof(prefixStr), " 0E1A 1 0 %04X ", dataSize);
  Serial1.print(prefixStr);//PORT:3610 SEC:1 SIDE:0 SIE:14
  Serial1.write(echonetReqData, dataSize);
  reqMillis = now;
}

void handleEchonetRes(){
  String  str = String(bp35c0Buffer);

  int indexErxudp = str.indexOf("ERXUDP");
  
  if(indexErxudp != -1){
    if(str.indexOf("\r\n",indexErxudp) != -1){
      int indexDport = indexErxudp+7+40+40+5;
      String dport = str.substring(indexDport, indexDport+4);
      Serial.print("DPORT:");
      Serial.println(dport);
      if(dport.equals("0E1A")){
        int indexSeoj = indexDport+5+17+2+2+5+8;
        String seoj = str.substring(indexSeoj, indexSeoj+6);
        Serial.print("SEOJ:");
        Serial.println(seoj);
        if(seoj.equals("028801")){
          int indexEsv = indexSeoj+6+6;
          String esv = str.substring(indexEsv, indexEsv+2);
          Serial.print("ESV:");
          Serial.println(esv);
          if(esv.equals("72")){
            String opcStr = str.substring(indexEsv+2, indexEsv+2+2);
            long opc = strtol(opcStr.c_str(),NULL,16);
            int indexEpc = indexEsv+4;
            for(int i = 0; i < opc; i++){
              String epc = str.substring(indexEpc, indexEpc+2);
              Serial.print("EPC:");
              Serial.println(epc);
              String pdcStr = str.substring(indexEpc+2, indexEpc+2+2);
              long pdc = strtol(pdcStr.c_str(),NULL,16);
              Serial.print("PDC:");
              Serial.println(pdc);
              
              if(epc.equals("E7")){
                String valStr = str.substring(indexEpc+4, indexEpc+4+8);
                long val = strtol(valStr.c_str(),NULL,16);
                currentImmediatePower = val;
                Serial.println(val);
              }else if(epc.equals("EA")){
                String valStr = str.substring(indexEpc+4+14, indexEpc+4+14+8);
                unsigned long val = strtoul(valStr.c_str(),NULL,16);
                currentIntegralPower = val;
                Serial.println(val);
              }else if(epc.equals("E1")){
                String valStr = str.substring(indexEpc+4, indexEpc+4+2);
                unsigned long val = strtoul(valStr.c_str(),NULL,16);
                integralPowerFactor = integralPowerFactorTable[val];
                Serial.println(integralPowerFactor);
              }else if(epc.equals("E8")){
                String valStr = str.substring(indexEpc+4, indexEpc+4+4);
                long val = strtol(valStr.c_str(),NULL,16);
                currentImmediateCurrentR = val;
                Serial.println(val);
                valStr = str.substring(indexEpc+4+4, indexEpc+4+4+4);
                val = strtol(valStr.c_str(),NULL,16);
                currentImmediateCurrentT = val;
                Serial.println(val);
              }else if(epc.equals("E0")){
                String valStr = str.substring(indexEpc+4, indexEpc+4+8);
                unsigned long val = strtoul(valStr.c_str(),NULL,16);
                currentIntegralPower = val;
                Serial.println(val);
              }
              indexEpc = indexEpc + 4 + (pdc*2);
            }// for opc
          }// esv:A response
        }// seoj:Smartmeter
      }// Port:Echonet
      errorCounter = 0;
      stateTransition(bp35c0State-1);
    }//UDP+CRLF
  }//UDP

  if((unsigned long)(now - reqMillis) > resTimeoutMillis){
    Serial.println("ERROR: No response from smartmeter");
    errorCounter++;
    stateTransition(bp35c0State-1);
  }
}

void bp35c0Loop(){
  while(Serial1.available()){
    char c = Serial1.read();
    //Serial.print(c);
    digitalWrite(ledPin, !digitalRead(ledPin));
    if(bp35c0BufferCounter < sizeof(bp35c0Buffer) - 1){
      Serial.print(c);
      bp35c0Buffer[bp35c0BufferCounter] = c;
      bp35c0BufferCounter++;
      bp35c0Buffer[bp35c0BufferCounter] = '\0';
    }
    yield();
  }
  digitalWrite(ledPin,ledDefaultState);
  switch(bp35c0State){
    case 0:
      ledDefaultState = HIGH;
      Serial.println("Reseting BP35C0...");
      //Reset BP35C0
      digitalWrite(BP35C0ResetPin,LOW);
      delay(10);
      digitalWrite(BP35C0ResetPin,HIGH);
      reqMillis = now;
      stateTransition(1);
      break;
    case 1:
      if((unsigned long)(now - reqMillis) > resetWaitMillis){
        Serial.println("Reset complete.");
        stateTransition(2);
      }
      break;
    case 2:
      bp35c0SendCommand("SKVER");
      stateTransition(3);
      break;
    case 3: //SKVER sent and wait OK
      waitForResponse("OK");
      break;
    case 4:
      bp35c0SendCommand("SKSETPWD C ", BROUTE_PASSWORD);
      stateTransition(5);
      break;
    case 5:
      waitForResponse("OK");
      break;
    case 6:
      bp35c0SendCommand("SKSETRBID ",BROUTE_ID);
      stateTransition(7);
      break;
    case 7:
      waitForResponse("OK");
      break;
    case 8:
      bp35c0SendCommand("SKSCAN 2 FFFFFFFF 9 0");
      stateTransition(9);
      break;
    case 9: //Scanning
      handleEpandesc();
      break;
    case 10:
      bp35c0SendCommand("SKLL64 ",addr);
      stateTransition(11);
      break;
   case 11:
      handleIpv6addr();
      break;
   case 12:
      bp35c0SendCommand("SKSREG S2 ",channel);
      stateTransition(13);
      break;
   case 13:
      waitForResponse("OK");
      break;
   case 14:
      bp35c0SendCommand("SKSREG S3 ",panid);
      stateTransition(15);
      break;
   case 15:
      waitForResponse("OK");
      break;
   case 16:
      bp35c0SendCommand("SKJOIN ",ipv6addr);
      stateTransition(17);
      break;
   case 17:
      waitForResponse("EVENT 25");
      if(bp35c0State == 18){
        lastImmediatePowerMillis = now;
        lastIntegralPowerMillis = now;
      }
      break;
   case 18:// JOINED
      if((unsigned long)(now - lastImmediatePowerMillis) > immediatePowerInterval){
        lastImmediatePowerMillis = now;
        sendEchonetReq(0xE7,0xE8);
        reqMillis = now;
        stateTransition(bp35c0State+1);
      }else if((unsigned long)(now - lastIntegralPowerMillis) > integralPowerInterval){
        lastIntegralPowerMillis = now;
        sendEchonetReq(0xE1,0xE0);
        stateTransition(bp35c0State+1);
      }
      break;
   case 19:
      handleEchonetRes();
      break;
  }
  if(errorCounter > maxErrorNumber){
    errorCounter = 0;
    stateTransition(0);
  }
}

void loop() {
  now = millis();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  bp35c0Loop();

  if(lastImmediatePower != currentImmediatePower ||
     lastImmediateCurrentR != currentImmediateCurrentR ||
     lastImmediateCurrentT != currentImmediateCurrentT) {
    String msg = "{";
    msg += "\"W\":";
    msg += currentImmediatePower;
    msg += ",";
    msg += "\"R\":";
    msg += String((float)currentImmediateCurrentR * 0.1, 1);
    msg += ",";
    msg += "\"T\":";
    msg += String((float)currentImmediateCurrentT * 0.1, 1);
    msg += "}";
    Serial.print("Publish:");
    Serial.println(msg);
    client.publish(mqttPowerTopic, msg.c_str());
    lastImmediatePower = currentImmediatePower;
    lastImmediateCurrentR = currentImmediateCurrentR;
    lastImmediateCurrentT = currentImmediateCurrentT;
  }

  if(lastIntegralPower != currentIntegralPower) {
    String msg = "{";
    msg += "\"I\":";
    msg += String((float)currentIntegralPower * integralPowerFactor, 1);
    msg += "}";
    Serial.print("Publish:");
    Serial.println(msg);
    client.publish(mqttIntegralPowerTopic, msg.c_str());
    lastIntegralPower = currentIntegralPower;
  }

  if ((unsigned long)(now - lastMsg) > 25 * 60 * 1000) {
    lastMsg = now;
    char* msg = "OFF";
    if(bp35c0State >=18) msg = "ON";
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(mqttStateTopic, msg);
  }
}
