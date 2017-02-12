

#include <adafruit_feather.h>
#include <adafruit_mqtt.h>

#include <adafruit_aio.h>

#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <Adafruit_FeatherOLED_WiFi.h>
#include <Adafruit_NeoPixel.h>

#include "wiced.h"

/* wifi passwords and ssid */
#include "wifi.h" // you won't find this in the git repo, make one yourself and set it to be ignored by git, so you don't publish your wifi passwords to teh interwebs...

#ifndef _wifi_H_
#define WLAN_SSID     "YOUR_SSID_HERE"
#define WLAN_PASS     "YOUR_WIFI_PASSWORD_HERE"
#endif

/* basic io */

#define LED_PIN           PA15
#define VBAT_ENABLED      1
#define VBAT_PIN          PA1

/* OLED */

#define OLED_BTN_A         PC3
#define OLED_BTN_B         PC2
#define OLED_BTN_C         PA3

Adafruit_FeatherOLED_WiFi oled = Adafruit_FeatherOLED_WiFi();

/* NEOPIXEL */

#define NEO_PIN            PC7
#define NEO_ROW_CNT        4
#define NEO_COL_CNT        8
#define NEO_PIX_CNT        (NEO_ROW_CNT * NEO_COL_CNT)
int red   = 0xFF;
int green = 0x00;
int blue  = 0x00;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEO_PIX_CNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

/* MQTT*/

#define MQTT_CLIENT_ID          "Duizelhuis Feather"
#define MQTT_DATA_TOPIC         "duizelhuis/feather/data"
#define MQTT_WILL_TOPIC         "duizelhuis/feather/will"
#define MQTT_WILL_MSG           MQTT_CLIENT_ID
#define MQTT_BROKER_HOST        "192.168.0.33"
#define MQTT_BROKER_PORT        1883

AdafruitMQTT      mqtt      ;
AdafruitMQTTTopic mqttTopic (&mqtt, MQTT_DATA_TOPIC);

/* tasks */
#define TASK_DISABLED 0

#define TASK_ID_LED   0
#define TASK_ID_NEO   1
#define TASK_ID_MENU  2
#define TASK_ID_STAT  3
#define TASK_MQTT     4
#define TASK_COUNT    5

Task tasks[] {
  Task { 1, 0         , &updateLed     , 0},
  Task { 1, 73        , &updateNeo     , 0},
  Task { 1, 20        , &updateButtons , 0},
  Task { 1, 5 * SECOND, &updateOled    , 0},
  Task { 1, 5 * SECOND, &updateMqtt    , 0},
};

void initTasks() {
  unsigned long currentTime = millis();
  for (int i = 0; i < TASK_COUNT; i++) {
    tasks[i].time = currentTime;
  }
}

void setup()
{
  //oled
  pinMode(OLED_BTN_A, INPUT_PULLUP);
  pinMode(OLED_BTN_B, INPUT_PULLUP);
  pinMode(OLED_BTN_C, INPUT_PULLUP);
  oled.init();

  updateOledMessage("Booting...");

  // neopixel
  strip.begin();
  strip.setBrightness(10);
  neoSetPixelColor(0, 0, strip.Color(0xFF, 0, 0));
  neoSetPixelColor(3, 7, strip.Color(0xFF, 0, 0));
  strip.show();

  connectWifi();

  initTasks();
  neoOff();
}

void loop()
{
  unsigned long currTime = millis();

  for (int i = 0; i < TASK_COUNT; i++ ) {
    if (tasks[i].interval == TASK_DISABLED)
      continue;

    if (tasks[i].time <= currTime) {
      tasks[i].time += tasks[i].interval;
      tasks[i].execute(&tasks[i]);
    }
  }
}
/* mqtt */
void updateMqtt(Task* task) {
  if (Feather.connected()) {
    // only try this if we've got network
    bool connected = mqtt.connected();
    bool subscribed = false;
    if (!connected) {
      // setup client
      mqtt.err_actions(false, false);
      mqtt.clientID(MQTT_CLIENT_ID);
      mqtt.will(MQTT_WILL_TOPIC, MQTT_WILL_MSG, MQTT_QOS_AT_LEAST_ONCE);
      mqtt.setDisconnectCallback(mqttDisconnectCallback);
      // connect to mqtt
      connected = mqtt.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
      if (connected) {
        updateOledMessage("MQTT connected.");
        subscribed = mqttTopic.subscribe(mqttMessageCallback);
        if (subscribed) {
          updateOledMessage("MQTT subscribed.");
          iso8601_time_t time;
          Feather.getISO8601Time(&time);
          mqttTopic.printf("{\"version\":{\"bootloader\":\"v%s\", \"sdk\":\"v%s\", \"firmware\":\"v%s\", \"arduino\":\"v%s\"}, \"timestamp\":\"%s\"}", Feather.bootloaderVersion(), Feather.sdkVersion(), Feather.firmwareVersion(), Feather.arduinoVersion(), (char*)&time);
        }
      }
    }

    if (connected) {
      float vbat = readVBat();
      iso8601_time_t time;
      Feather.getISO8601Time(&time);
      mqttTopic.printf("{\"status\":{\"vbat\":%f,\"rssi\":%d}, \"timestamp\":\"%s\"}", readVBat(), Feather.RSSI(), (char*)&time);
    }
  }
}

void mqttSendButton(Task* task, char button, bool state) {
  iso8601_time_t time;
  Feather.getISO8601Time(&time);
  mqttTopic.printf("{\"buttons\":{\"%c\":%d}, timestamp:\"%s\"}", button, state ? 1 : 0, (char*) &time);
}


void mqttDisconnectCallback() {
  reset();
}

void mqttMessageCallback(UTF8String topic, UTF8String message)
{
  if ( message == "reboot" )
  {
    reset();
  }
  else if (message.len >= 2 && message.data[0] == '!') {
    switch (message.data[1]) {
      case '~':
        break;
      case '*':
        break;
    }
  }
  else if (message.len == 7 && message.data[0] == '#') {
    int number = (int) strtol( &message.data[1], NULL, 16);
    red = number >> 16;
    green = number >> 8 & 0xFF;
    blue = number & 0xFF;
    char buffer[50];
    sprintf(buffer, "Set color #%02X%02X%02X\0", red, green, blue);
    updateOledMessage(buffer);
  }
}


void reset() {
  updateOledMessage("Reboot...");
  delay(500);
  Feather.sdep(SDEP_CMD_RESET, 0, NULL, NULL, NULL);
}

/* Buttons */

bool btnA() {
  return digitalRead(OLED_BTN_A) == LOW;
}

bool btnB() {
  return digitalRead(OLED_BTN_B) == LOW;
}

bool btnC() {
  return digitalRead(OLED_BTN_C) == LOW;
}

void updateButtons(Task* task) {
  int oldStatus = task->status;
  bool a = updateButton(task, btnA(), 0);
  bool b = updateButton(task, btnB(), 1);
  bool c = updateButton(task, btnC(), 2);
}

bool updateButton(Task* task, bool pressed, int bit) {
  int mask = 1 << bit;
  if (pressed) {
    if ((task->status & mask) != mask ) {
      char buffer[6] = { 'b', 't', 'n' , ' ', 'a' + bit, 0 };
      updateOledMessage(buffer);
      task->status |= mask;
      mqttSendButton(task, 'a' + bit, pressed);
      return true;
    }
  } else {
    task->status &= ~mask;
  }
  return false;
}

/* OLED */

void updateOledMessage(const char* msg) {
  oled.clearMsgArea();
  oled.setCursor(0, 12);
  oled.println(msg);
  oled.display();
}

void updateOledMessage(const UTF8String msg) {
  oled.clearMsgArea();
  oled.setCursor(0, 12);
  oled.println(msg);
  oled.display();
}

void updateOled(Task* task) {
  updateOledBattery();
  updateOledWifi();
  oled.refreshIcons();
}

void updateOledBattery() {
  float vbat = readVBat();
  oled.setBattery(vbat);
}

float readVBat() {
  int ar = analogRead(VBAT_PIN);
  float vbat = (ar * 0.080566F) ;
  return vbat;
}

void updateOledWifi() {
  oled.setIPAddress(Feather.localIP());
  oled.setRSSI(Feather.RSSI());
  oled.setConnected(Feather.connected());
}

/* WIFI */

void onWifiDisconnected() {
  reset();
}

void onWifiDisconnectedNoop() {
  /* do nothing */
}

void connectWifi() {
  bool connected = false;
  Feather.setDisconnectCallback(&onWifiDisconnectedNoop);

  while (!connected) {

    ledOn();

    // try to connect via saved profile first.
    updateOledMessage(WLAN_SSID);

    // check if we have a profile for the SSID...
    if (Feather.checkProfile(WLAN_SSID)) {
      ledOff();
      // and try to connect via the profile, it's supposed to be faster...
      connected = Feather.connect();
    }

    if (!connected) {
      // if that doesn't work, try the hardcoded ssid/pass
      updateOledMessage("Login to wifi...");
      connected = Feather.connect(WLAN_SSID, WLAN_PASS);

      if (connected) {
        // if that works, save the profile.
        updateOledMessage("Save profile");
        Feather.clearProfiles();
        Feather.saveConnectedProfile();
      }
    }


    // update display
    if (connected) {
      Feather.setDisconnectCallback(&onWifiDisconnected);
      oled.setRSSIVisible(true);
      oled.setIPAddressVisible(true);
      updateOledMessage(WLAN_SSID);
      delay(200);
      updateOledMessage("connected");
    } else {
      ledOff();
      delay(200);
      
      updateOledMessage("connect failed...");
      ledOn();
      delay(200);
    }

    ledOff();
    delay(200);
  }
}

/* NEOPIXEL */

void updateNeo(Task* task) {
  neoScanner(task);
}

void neoScanner(Task* task) {
  int c0 = (task->status >> 0) & 0x0F;
  int c1 = (task->status >> 4) & 0x0F;
  int c2 = (task->status >> 8) & 0x0F;
  int lr = (task->status >> 12) & 0x0F;

  uint32_t color = strip.Color(0, 0, 0);
  for (int r = 0; r < NEO_ROW_CNT; r++) {
    neoSetPixelColor(r, c2, color);
  }

  color = strip.Color(red >> 3, green >> 3, blue >> 3);
  for (int r = 0; r < NEO_ROW_CNT; r++) {
    neoSetPixelColor(r, c1, color);
  }

  color = strip.Color(red, green, blue);
  for (int r = 0; r < NEO_ROW_CNT; r++) {
    neoSetPixelColor(r, c0, color);
  }

  strip.show();

  int i = (lr == 00) ? 1 : -1;
  c2 = c1;
  c1 = c0;
  c0 = c0 + i;
  if (c0 <= 0 || c0 + 1 >= NEO_COL_CNT) {
    lr = ((~lr) & 0x0F);
  }

  task->status = c0 | ( c1 << 4 ) | ( c2 << 8 ) | (lr << 12);
}

void neoClear() {
  for (int r = 0; r < NEO_ROW_CNT; r++) {
    for (int c = 0; c < NEO_COL_CNT; c++) {
      neoSetPixelColor(r, c, 0 & 255);
    }
  }
}

void neoOff() {
  neoClear();
  strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t neoWheel(byte wheelPos) {
  wheelPos = 255 - wheelPos;
  if (wheelPos < 85) {
    return strip.Color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  if (wheelPos < 170) {
    wheelPos -= 85;
    return strip.Color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  wheelPos -= 170;
  return strip.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

void neoSetPixelColor(int row, int col, uint32_t color ) {
  uint16_t index = (uint16_t)((row * NEO_COL_CNT) +  col);
  strip.setPixelColor(index, color);
}

/* STATUS_LED */

void updateLed(Task* task) {
  if (task->status <= 0) {
    task->interval = 0;
    ledOff();
    return;
  } else {
    if ((task->status & 1) == 1) {
      ledOn();
    } else {
      ledOff();
    }
    task->status -= 1;
  }
}

void ledOn() {
  digitalWrite(LED_PIN, HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LOW);
}

void ledBlinkFast(int count) {
  tasks[TASK_ID_LED].time = millis();
  tasks[TASK_ID_LED].interval = 20;
  tasks[TASK_ID_LED].status = count;
}

void ledBlinkSlow(int count) {
  tasks[TASK_ID_LED].time = millis();
  tasks[TASK_ID_LED].interval = 150;
  tasks[TASK_ID_LED].status = count;
}



