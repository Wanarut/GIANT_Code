/*--------DATABASEs--------*/
#include <FirebaseESP32.h>
#include <MicroGear.h>
/*--------TIMESTAMP--------*/
#include <NTPClient.h>
#include <WiFiUdp.h>
/*--------COMMUNICATION--------*/
#include <LoRa_DSR.h>
/*--------SENSOR--------*/
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <board.h>

/*------WiFi------*/
//#define WIFI_SSID     "DAMASHII"
//#define WIFI_PASSWORD "12345678"
#define WIFI_SSID     "OASYS_9"
#define WIFI_PASSWORD "12345678"
/*------DATABASEs------*/
#define FIREBASE_HOST "gianttable.firebaseio.com"
#define FIREBASE_AUTH "gXoxriyHBb6KcnLsH83WVlb77YKT4oQVlV5q8wcj"
FirebaseJsonData jsonObj;
FirebaseData firebaseData;
FirebaseJson json;
#define APPID   "GIANT"
#define KEY     "jmcelnYHZMPXPvo"
#define SECRET  "8tuJjmr1l2zm4j1QLcvlyJXt3"
#define ALIAS   "GIANTGateway"
WiFiClient client;
MicroGear microgear(client);
/*------TIMESTAMP------*/
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
/*------COMMUNICATION------*/
#define LORA_BAND   920E6     // LoRa Band (Thailand)
#define PABOOST     true
byte currentID      = 93;     // This node address
int type = 2;
byte destinationID  = 255;    // Original destination (0xFF broadcast)
LoRa_DSR DSR(currentID, destinationID, type, true);
/*------SENSOR------*/
extern "C" {
  uint8_t temprature_sens_read();
}
Adafruit_BME280 bme; // I2C
TwoWire twi = TwoWire(1);
#define RXPin (17)
#define TXPin (16)
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
/*------STATUS------*/
#define pattern_size 8
#define interval 500
struct LEDBlink {
  int pin;
  bool pattern[pattern_size];
  int index;
};
struct Button {
  const uint8_t PIN;
  bool pressed;
  LEDBlink LEDOut;
  unsigned long lastdeb;
};
void IRAM_ATTR isr_btn(void* arg);
Button redBtn = {2, false, {12, {0, 0, 0, 0, 0, 0, 0, 0}, 0}, 0};
#define debounceDelay  1000

int current_status = 0;

long timeoutCheck = 300000;
bool hasgps = false;
bool hasbme = false;
bool haswifi = false;

void setup()
{
  Serial.begin(115200);
  twi.begin(21, 22);
  delay(5000);
  Serial.println(F("Gateway Starting..."));

  pinMode(redBtn.PIN, INPUT_PULLUP);
  attachInterruptArg(redBtn.PIN, isr_btn, &redBtn, FALLING);
  pinMode(redBtn.LEDOut.pin, OUTPUT);

  digitalWrite(redBtn.LEDOut.pin, HIGH);
  //  GPSSetup();
  if (bme.begin(&twi)) {
    hasbme = true;
    Serial.println(F("Found BME280 sensor"));
  } else {
    Serial.println(F("No BME280 sensor"));
  }
  digitalWrite(redBtn.LEDOut.pin, LOW);

  DSR.configForLoRaWAN(20, 6, 125000, 5, 8, 0x34); // (_TXPOWER = 14, _SPREADING_FACTOR = 12, _BANDWIDTH = 125000, _CODING_RATE = 5, _PREAMBLE_LENGTH = 8, _SYNC_WORD = 0x34)
  delay(1000);
  DSR.begin(LORA_BAND, PABOOST); // (_LORA_BAND = 868E6, _PABOOST = true)

  WiFiSetup();
  if (haswifi) {
    FirebaseSetup();
    netpieSetup();
    NTPSetup();
  }

  Serial.println(F("Start GIANT Gateway"));
}

#define interval 60000    // interval between sends
#define intervalLED 250
long lastsent = -interval + random(1000, 5000);
long lastLED = 0;
long lastlatency = 0;
int counter = 0;
int success = 0;
void loop()
{
  long cur_time = millis();
  DSR.check_timer(cur_time);

  String received = DSR.checkPacket();
  if (received != "") {
    Serial.print(F("Received -> "));
    Serial.print(received);
    if (received == "SENT_SUCCESS") {
      Serial.print("\tSend\t" + String(counter));
      Serial.print("\tSuccess\t" + String(++success));
      Serial.print("\tLatency\t" + String(cur_time - lastlatency) + "\tms");
      int rssi = DSR.packetRssi();
      Serial.print("\tRSSI\t" + String(rssi));
      Serial.println("\tSnr\t" + String(DSR.packetSnr()));

      if (rssi < -100) {
        bool pattern[] = {1, 0, 1, 0, 0, 0, 0, 0};
        memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
      } else {
        bool pattern[] = {1, 0, 1, 0, 1, 0, 1, 0};
        memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
      }
    } else {
      Serial.println();
      if (haswifi) timeClient.update();

      json.clear();
      String received_id = updateDataString(received);
      String received_datatable = received_id + "Weather";
      if (haswifi) sendData2Firebase(received_datatable, json);

      json.clear();
      if (updateWeatherString(received)) {
        received_datatable += "Feed";
        if (haswifi) sendWeather2Feed(received_datatable);
      }
    }
  }

  if (GPSSerial.available() || !hasgps) {
    if (gps.encode(GPSSerial.read()) || !hasgps) {
      if (cur_time - lastsent > interval) {
        lastsent = cur_time;
        if (haswifi) timeClient.update();

        json.clear();
        updateData();
        String datatable = String(currentID) + "Weather";
        if (haswifi) sendData2Firebase(datatable, json);

        //    String jsonStr;
        //    json.toString(jsonStr);
        //    Serial.println(jsonStr);

        json.clear();
        if (hasbme) updateWeather();
        datatable += "Feed";
        if (haswifi) sendWeather2Feed(datatable);

        //    json.toString(jsonStr, true);
        //    Serial.println(jsonStr);
      }

      if (redBtn.pressed) {
        Serial.println(F("RED Button is pressed."));
        redBtn.pressed = false;
        if (current_status == 2) {
          current_status = 0;
          bool pattern[] = {0, 0, 0, 0, 0, 0, 0, 0};
          memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
        } else {
          current_status = 2;
          bool pattern[] = {1, 1, 1, 1, 1, 1, 1, 1};
          memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
        }
        if (haswifi) timeClient.update();

        json.clear();
        String datatable = String(currentID) + "Weather";
        updateData();
        if (haswifi) sendData2Firebase(datatable, json);
      }
    }
  }

  if (cur_time - lastLED > intervalLED) {
    lastLED = cur_time;
    blinkUpdate(redBtn.LEDOut);
  }

  if (haswifi) {
    if (microgear.connected())
    {
      microgear.loop();
    } else {
      Serial.println("connection lost, reconnect...");
      microgear.connect(APPID);
      delay(1000);
    }
  }
}

void blinkUpdate(LEDBlink &LED) {
  digitalWrite(LED.pin, LED.pattern[LED.index]);
  LED.index++;
  LED.index %= pattern_size;
}

void GPSSetup()
{
  GPSSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  Serial.println(F("Waiting GPS..."));

  long prev_time = millis();
  while (millis() - prev_time < timeoutCheck) {
    if (GPSSerial.available() && gps.encode(GPSSerial.read()) && gps.location.isValid()) {
      hasgps = true;
      return;
    }
  }
  Serial.println(F("No GPS signal"));
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS module"));
  }
}
void WiFiSetup()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  long prev_time = millis();
  while (millis() - prev_time < timeoutCheck / 20)
  {
    if (WiFi.status() == WL_CONNECTED) {
      haswifi = true;
      break;
    }
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}
void FirebaseSetup()
{
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}
void NTPSetup()
{
  timeClient.begin();
  //  timeClient.setTimeOffset(7 * 3600);
  timeClient.setTimeOffset(0);
}
void netpieSetup()
{
  microgear.on(MESSAGE, onMsghandler);
  microgear.on(PRESENT, onFoundgear);
  microgear.on(ABSENT, onLostgear);
  microgear.on(CONNECTED, onConnected);
  microgear.init(KEY, SECRET, ALIAS);
  microgear.connect(APPID);
}

void updateData()
{
  int stat = current_status;
  json.set("id", int(currentID));
  json.set("type", type);
  json.set("status", stat);

  if (hasgps) {
    if (gps.location.isValid())
    {
      double lati = gps.location.lat();
      double longi = gps.location.lng();
      int sp = gps.speed.mph();
      int sat_count = gps.satellites.value();
      json.set("latitude", lati);
      json.set("longitude", longi);
      json.set("speed", sp);
      json.set("satellite_count", sat_count);
    }
  } else {
    json.set("latitude", 18.795402);
    json.set("longitude", 98.953029);
    json.set("speed", 0);
    json.set("satellite_count", 9);
  }

  double dev_temp = (temprature_sens_read() - 32) / 1.8;
  json.set("device_temperature", dev_temp);

  if (haswifi) {
    int timestamp = timeClient.getEpochTime();
    json.set("timestamp", timestamp);
  }
}
String updateDataString(String received)
{
  String received_id = "";
  FirebaseJson got_json;
  got_json.setJsonData(received);

  got_json.get(jsonObj, "node/[0]");
  json.set("id", jsonObj.intValue);
  received_id = jsonObj.stringValue;

  got_json.get(jsonObj, "node/[1]");
  json.set("type", jsonObj.intValue);
  got_json.get(jsonObj, "node/[2]");
  json.set("status", jsonObj.intValue);
  got_json.get(jsonObj, "gps/[0]");
  json.set("latitude", jsonObj.doubleValue);
  got_json.get(jsonObj, "gps/[1]");
  json.set("longitude", jsonObj.doubleValue);
  got_json.get(jsonObj, "gps/[2]");
  json.set("speed", jsonObj.intValue);
  got_json.get(jsonObj, "gps/[3]");
  json.set("satellite_count", jsonObj.intValue);
  got_json.get(jsonObj, "dtp");
  json.set("device_temperature", jsonObj.intValue);
  int timestamp = timeClient.getEpochTime();
  json.set("timestamp", timestamp);

  return received_id;
}
void updateWeather()
{
  double temp = bme.readTemperature();
  double humi = bme.readHumidity();
  double pres = bme.readPressure() / 100.0F;

  json.set("temperature", temp);
  json.set("humidity", humi);
  json.set("pressure", pres);
}
bool updateWeatherString(String received)
{
  FirebaseJson got_json;
  got_json.setJsonData(received);

  got_json.get(jsonObj, "wt/[0]");
  int temp = jsonObj.intValue;
  json.set("temperature", temp);
  got_json.get(jsonObj, "wt/[1]");
  int humi = jsonObj.intValue;
  json.set("humidity", humi);
  got_json.get(jsonObj, "wt/[2]");
  int pres = jsonObj.intValue;
  json.set("pressure", pres);

  if (temp == humi && temp == pres) return false;
  return true;
}

void sendData2Firebase(String datatable, FirebaseJson &json_param)
{
  if (Firebase.setJSON(firebaseData, "/now/" + datatable, json_param))
  {
    String jsonStr;
    json_param.toString(jsonStr);
    Serial.println(jsonStr);
    Serial.println("SET PASSED");
    //    Serial.println("PATH: " + firebaseData.dataPath());
    //    Serial.println("ETag: " + firebaseData.ETag());
    //    Serial.println("------------------------------------");
    //    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }
  if (Firebase.pushJSON(firebaseData, "/logs/" + datatable, json_param))
  {
    Serial.println("PUSH PASSED");
    //    Serial.println("PATH: " + firebaseData.dataPath());
    //    Serial.print("PUSH NAME: ");
    //    Serial.println(firebaseData.pushName());
    //    Serial.println("ETag: " + firebaseData.ETag());
    //    Serial.println("------------------------------------");
    //    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }
}
void sendWeather2Feed(String strfeed)
{
  char feed[16];
  strfeed.toCharArray(feed, strfeed.length() + 1);

  json.get(jsonObj, "temperature");
  float temp = jsonObj.doubleValue;
  json.get(jsonObj, "humidity");
  float humi = jsonObj.doubleValue;
  json.get(jsonObj, "pressure");
  float pres = jsonObj.doubleValue;

  char feed_data[64];
  sprintf(feed_data, "Temperature:%.2f,Humidity:%.2f,Pressure:%.2f", temp, humi, pres);

  Serial.print(F("Feeding to "));
  Serial.print(feed);
  Serial.print(F(" --> "));
  Serial.println(feed_data);
  microgear.writeFeed(feed, feed_data);
}

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen)
{
  Serial.print("Incoming message --> ");
  msg[msglen] = '\0';
  String received = (char *)msg;
  //  Serial.println((char *)msg);
  //  Serial.println(received);

  FirebaseJson got_json;
  got_json.setJsonData(received);
  if (haswifi) sendData2Firebase("CenterData", got_json);

  got_json.get(jsonObj, "dst");
  lastlatency = millis();
  DSR.sendDATA(received, jsonObj.intValue);
  Serial.print(F("Sending -> "));
  Serial.print(jsonObj.intValue);
  Serial.print(F(" to ID:"));
  Serial.println(received);
  counter++;
}
void onFoundgear(char *attribute, uint8_t* msg, unsigned int msglen)
{
  Serial.print("Found new member --> ");
  for (int i = 0; i < msglen; i++)
    Serial.print((char)msg[i]);
  Serial.println();
}
void onLostgear(char *attribute, uint8_t* msg, unsigned int msglen)
{
  Serial.print("Lost member --> ");
  for (int i = 0; i < msglen; i++)
    Serial.print((char)msg[i]);
  Serial.println();
}
void onConnected(char *attribute, uint8_t* msg, unsigned int msglen)
{
  Serial.println("Connected to NETPIE...");
  microgear.setAlias(ALIAS);
}

void IRAM_ATTR isr_btn(void* arg) {
  Button* btn = static_cast<Button*>(arg);

  unsigned long curtime = millis();
  if (!btn->pressed && curtime - btn->lastdeb > debounceDelay) {
    btn->lastdeb = curtime;
    btn->pressed = true;
  } else {
    btn->pressed = false;
  }
}
