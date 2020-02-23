/*--------DATABASEs--------*/
#include <FirebaseESP32.h>
/*--------COMMUNICATION--------*/
#include <LoRa_DSR.h>
/*--------SENSOR--------*/
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <board.h>

/*------DATABASEs------*/
FirebaseJsonData jsonObj;
FirebaseJson json;
/*------COMMUNICATION------*/
#define LORA_BAND   920E6 // LoRa Band (Thailand)
#define PABOOST     true
byte currentID      = 92;        // This node address
int type = 1;
byte destinationID  = 93;        // Original destination (0xFF broadcast)
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

void setup()
{
  Serial.begin(115200);
  twi.begin(21, 22);
  delay(5000);
  Serial.println(F("Cluster Starting..."));

  pinMode(redBtn.PIN, INPUT_PULLUP);
  attachInterruptArg(redBtn.PIN, isr_btn, &redBtn, FALLING);
  pinMode(redBtn.LEDOut.pin, OUTPUT);

  digitalWrite(redBtn.LEDOut.pin, HIGH);
  GPSSetup();
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

  Serial.println(F("Start GIANT Cluster"));
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

  if (GPSSerial.available() || !hasgps) {
    if (gps.encode(GPSSerial.read()) || !hasgps) {
      if (cur_time - lastsent > interval) {
        lastsent = cur_time;

        json.clear();
        updateData();
        if (hasbme) updateWeather();
        String message;
        json.toString(message);
        lastlatency = cur_time;
        DSR.sendDATA(message, destinationID);
        Serial.print(F("Sending -> "));
        Serial.println(message);
        counter++;
      }
    }
  }

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
      FirebaseJson got_json;
      got_json.setJsonData(received);
      String jsonStr;
      got_json.toString(jsonStr);
      Serial.println(jsonStr);

      got_json.get(jsonObj, "dst");
      Serial.println(jsonObj.intValue);
      got_json.get(jsonObj, "cod");
      Serial.println(jsonObj.intValue);

      switch (jsonObj.intValue) {
        case 1: {
            bool pattern[] = {1, 1, 0, 0, 0, 0, 0, 0};
            memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
            break;
          }
        case 5: {
            bool pattern[] = {1, 1, 0, 0, 1, 1, 0, 0};
            memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
            break;
          }
        case 7: {
            bool pattern[] = {1, 1, 1, 1, 1, 1, 0, 0};
            memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
            break;
          }
        default: {
            bool pattern[] = {1, 0, 1, 0, 1, 0, 1, 0};
            memcpy(redBtn.LEDOut.pattern, pattern, pattern_size);
            break;
          }
      }
    }
  }
  
  if (GPSSerial.available() || !hasgps) {
    if (gps.encode(GPSSerial.read()) || !hasgps) {
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

        json.clear();
        updateData();
        if (hasbme) updateWeather();
        String message;
        json.toString(message);
        lastlatency = cur_time;
        DSR.sendDATA(message, destinationID);
        Serial.print(F("Sending -> "));
        Serial.println(message);
        counter++;
      }
    }
  }

  if (cur_time - lastLED > intervalLED) {
    lastLED = cur_time;
    blinkUpdate(redBtn.LEDOut);
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
      Serial.println(F("Found GPS module"));
      return;
    }
  }
  Serial.println(F("No GPS signal"));
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS module"));
  }
}
void updateData()
{
  int stat = current_status;
  json.set("node/[0]", int(currentID));
  json.set("node/[1]", type);
  json.set("node/[2]", stat);

  if (hasgps) {
    if (gps.location.isValid())
    {
      double lati = gps.location.lat();
      double longi = gps.location.lng();
      int sp = gps.speed.mph();
      int sat_count = gps.satellites.value();
      json.set("gps/[0]", lati);
      json.set("gps/[1]", longi);
      json.set("gps/[2]", sp);
      json.set("gps/[3]", sat_count);
    }
  } else {
    json.set("gps/[0]", 18.787681);
    json.set("gps/[1]", 98.949387);
    json.set("gps/[2]", 0);
    json.set("gps/[3]", 10);
  }

  int dev_temp = (temprature_sens_read() - 32) / 1.8;
  json.set("dtp", dev_temp);
}
void updateWeather()
{
  int temp = bme.readTemperature();
  int humi = bme.readHumidity();
  int pres = bme.readPressure() / 100.0F;

  json.set("wt/[0]", temp);
  json.set("wt/[1]", humi);
  json.set("wt/[2]", pres);
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
