
/**
 * Application that uses an M5Stack Core to connect to a mqtt broker using wifi.
 * When connected it wil listen to serveral topics.
 * Data send to the topic will be display on the screens. Right now there are three screen.
 * You can use the buttons A and C to navigate between the screens.
 * 
 * You can connect a IR Module to Port A of the M5Stack Unit to send IR commands.
 * The programm IRrecvDumpV2 vom the examples of the IRremoteESP8266 lib is used to get the IR command.
 * 
 * The application also displays the time an the screen. The time source is a configured ntp server.
 * The ezTime lib is used to retrieve and format the time and date. 
 */

#include <M5Stack.h>
#include <WiFi.h>
#include "EspMQTTClient.h"
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ArduinoJson.h>
#include <ezTime.h>
#include "config.h"

#define TFT_GREY 0x5AEB
#define COL01 0x0066

// define the colors 
uint16_t backgroundCol = TFT_BLACK;
uint16_t menuBarBgCol = TFT_DARKGREY;
uint16_t menuTextCol = TFT_BLACK;
uint16_t mainAreaBgCol = COL01;
uint16_t infoArea1BgCol = TFT_OLIVE;
uint16_t infoArea2BgCol = TFT_OLIVE;
uint16_t mainTextCol = TFT_YELLOW;

// define the ports of the IR module for sending and receiving IR commands
const uint16_t kIrSendLed = 21;
const uint16_t kIrReveiveLed = 22;

// IR settings and commands
const uint16_t ir_freq = 38;
const uint16_t devon_cdr[95] = {274, 778,  270, 786,  248, 784,  246, 1846,  268, 786,  244, 786,  246, 1846,  270, 1844,  244, 1846,  250, 786,  270, 784,  268, 1822,  246, 1844,  268, 790,  268, 762,  248, 44742,  274, 754,  270, 786,  246, 784,  272, 1820,  270, 784,  248, 1844,  246, 788,  268, 788,  248, 784,  268, 1820,  270, 1846,  268, 762,  270, 764,  268, 1844,  270, 1794,  272, 44742,  248, 780,  272, 784,  246, 786,  244, 1846,  268, 790,  236, 794,  240, 1854,  268, 1844,  244, 1846,  244, 788,  268, 788,  244, 1846,  242, 1848,  268, 790,  266, 764,  196};  // DENON 9CC
const uint16_t devon_power[97] = {314, 758,  250, 782,  248, 1842,  272, 1842,  250, 790,  242, 784,  272, 784,  248, 784,  248, 784,  272, 1842,  250, 782,  248, 1842,  272, 786,  248, 782,  248, 784,  272, 46810,  314, 760,  248, 784,  250, 1842,  272, 1842,  250, 782,  248, 1840,  274, 1842,  250, 1840,  248, 1842,  272, 784,  250, 1840,  250, 784,  272, 1842,  250, 1816,  272, 1840,  276, 29576,  274, 780,  272, 784,  248, 1840,  250, 1842,  272, 784,  250, 30580,  274, 778,  272, 784,  250, 1840,  250, 1840,  274, 29214,  272, 754,  272, 786,  248, 1840,  250, 1840,  274, 784,  248};
const uint16_t devon_function[95] = {274, 780,  270, 786,  248, 1840,  248, 1842,  272, 784,  250, 1842,  250, 1840,  272, 1842,  248, 1842,  248, 1842,  274, 784,  248, 1840,  250, 782,  272, 784,  248, 758,  272, 42598,  274, 778,  272, 786,  246, 1842,  248, 1822,  320, 758,  248, 784,  248, 784,  272, 784,  248, 780,  250, 784,  274, 1840,  250, 780,  248, 1844,  272, 1844,  250, 1814,  274, 44742,  272, 732,  294, 786,  248, 1840,  250, 1842,  272, 784,  248, 1840,  274, 1816,  274, 1840,  250, 1840,  274, 1816,  272, 784,  248, 1840,  250, 784,  274, 782,  248, 760,  270};  // DENON 1BE8
const uint16_t devon_volUp[95] = {274, 780,  276, 782,  248, 1814,  300, 1818,  272, 784,  274, 1814,  274, 760,  272, 1842,  250, 1840,  276, 758,  272, 784,  248, 1840,  274, 762,  272, 784,  250, 756,  272, 44740,  274, 752,  276, 782,  274, 1814,  250, 1842,  276, 782,  250, 780,  274, 1816,  274, 784,  248, 782,  248, 1844,  272, 1840,  276, 756,  250, 1818,  318, 1820,  274, 1790,  274, 42598,  274, 780,  270, 786,  246, 1842,  250, 1842,  270, 786,  248, 1842,  270, 790,  266, 1822,  248, 1840,  244, 790,  266, 790,  238, 1848,  196, 862,  242, 788,  238, 794,  266};  // DENON 1AC8
const uint16_t devon_volDown[95] = {274, 756,  268, 812,  274, 1816,  274, 1818,  274, 782,  274, 758,  272, 758,  274, 1840,  274, 1816,  274, 758,  300, 758,  274, 1790,  274, 782,  286, 770,  274, 732,  272, 45798,  272, 754,  274, 784,  250, 1838,  250, 1842,  272, 786,  250, 1838,  250, 1842,  274, 756,  274, 782,  250, 1840,  272, 1842,  250, 782,  250, 1840,  276, 1840,  250, 1814,  272, 41566,  272, 754,  270, 786,  248, 1842,  250, 1842,  272, 784,  272, 758,  248, 786,  268, 1844,  248, 1842,  248, 786,  270, 786,  246, 1844,  246, 786,  270, 786,  248, 760,  270};  // DENON 18C8
const uint16_t grundig_power[23] = {848, 842,  856, 842,  1706, 840,  854, 842,  856, 840,  856, 844,  854, 846,  856, 842,  854, 1690,  856, 844,  1706, 840,  854};  // RC5 80C
const uint16_t grundig_volUp[23] = {848, 844,  1704, 840,  856, 838,  856, 842,  852, 842,  854, 840,  854, 846,  854, 1690,  1706, 840,  854, 842,  856, 842,  854};  // RC5 10
const uint16_t grundig_volDown[23] = {870, 840,  856, 842,  1706, 838,  856, 840,  856, 840,  856, 840,  856, 844,  856, 1666,  1728, 840,  854, 840,  856, 1688,  856};  // RC5 811

// set the main topic the application listens to
const String mainTopic = "irblaster";

// set the lowest and higest screen number
const short MIN_SCREEN = 0;
const short MAX_SCREEN = 2;
// the current screen
short screenNum = 0;

// init hour and minutes
short hours = 0;
short minutes = 0;

// init the strings displayed on the screen. Used for removing the display text
String oldTimeStr = "";
String oldOutdoorTempStr = "";
String oldIndoorTempStr = "";
String oldDect210State = "";
String oldMessageStr = "";

// init the debug flag
boolean showDebug = false;

// init the screen flag
boolean screenOn = true;

// define the Timezone object.
Timezone stuttgartTZ;

// init Wifi and MQTT
EspMQTTClient client(
  WIFI_SSID,
  WIFI_PASSWORD,
  MQTT_BROKER_URL,  // MQTT Broker server ip
  "",   // Can be omitted if not needed
  "",   // Can be omitted if not needed
  "M5Stack01"      // Client name that uniquely identify your device
);

// init IR sender
IRsend irsend(kIrSendLed);

void setup() {
  Serial.begin(115200);
  client.enableLastWillMessage("irblaster/lastwill", "I am going offline");
  // initialize the M5Stack object
  M5.begin();
  M5.Power.begin();
  irsend.begin();

  M5.Lcd.fillScreen(BLACK);
  displayText("Stelle die Zeit ein", " ", 20, 20, 1, 0);
}

// Call back used by MQTT lib when the connecton to the MQTT broker has been established
void onConnectionEstablished() {
  if (showDebug) {
    setDebug(INFO);
  }
  // set the ntp server
  setServer("fritz.box");
  // wait untill the time has been loaded
  waitForSync();
  // set the location to the timezone object
  stuttgartTZ.setLocation(F("de"));
  // blank the screen
  M5.Lcd.fillScreen(BLACK);
  // switch to the first screen
  screenOne();

  // subscribe to the topics
  client.subscribe(mainTopic + "/command/send",[] (const String& payload)  {
    if (payload == "devon_cdr") {
      irsend.sendRaw(devon_cdr, 95, ir_freq);
    } else if (payload == "devon_power") {
      irsend.sendRaw(devon_power, 97, ir_freq);
    } else if (payload == "devon_function") {
      irsend.sendRaw(devon_function, 95, ir_freq);
    } else if (payload == "devon_volUp") {
      irsend.sendRaw(devon_volUp, 95, ir_freq);
    } else if (payload == "devon_volDown") {
      irsend.sendRaw(devon_volDown, 95, ir_freq);
    } else if (payload == "grundig_power") {
      irsend.sendRaw(grundig_power, 23, ir_freq);
    } else if (payload == "grundig_volUp") {
      irsend.sendRaw(grundig_volUp, 23, ir_freq);
    } else if (payload == "grundig_volDown") {
      irsend.sendRaw(grundig_volDown, 23, ir_freq);
    } else {
      client.publish(mainTopic + "/help", "irblaster commands are: devon_cdr, devon_power, devon_function, devon_volUp, devon_volDown, grundig_power, grundig_volUp, grundig_volDown");
    }
  });

  client.subscribe(mainTopic + "/display", [] (const String& payload) {
    if (oldMessageStr.length() == 0) {
      oldMessageStr = payload;
    }
    displayText(payload, oldMessageStr, 30, 20, 2, 0);

    oldMessageStr = payload;
  });

  client.subscribe("gc/ble1", [] (const String& payload) {
    processPuckData(payload);
  });

  client.subscribe("fritzbox/power/dect210", [] (const String& payload) {
    processPower(payload);
  });

  client.subscribe("fritzbox/temp/wohnzimmer/fenster", [] (const String& payload) {
    processTemperature(payload);
  });

  client.publish(mainTopic + "/start", "irblaster has connected");
}

void loop() {
  // process MQTT messaged
  client.loop();
  // do M5Stack stuff like reading the button states
  M5.update();

  if (M5.BtnA.wasReleased() || M5.BtnA.pressedFor(1000, 200)) {
    screenNum = screenNum -1;
    if (screenNum < MIN_SCREEN) {
      screenNum = 0;
    } else {
      switchToScreen(screenNum);
    }
  }

  if (M5.BtnB.pressedFor(1000)) {
    showDebug = !showDebug;
    switchScreenOn();
  }

  if (M5.BtnC.wasReleased() || M5.BtnC.pressedFor(1000, 200)) {
    screenNum = screenNum + 1;
    if (screenNum > MAX_SCREEN) {
      screenNum = MAX_SCREEN;
    } else {
      switchToScreen(screenNum);
    }
  }

  // process ezTime events
  events();

  // when the minute has been chaanged update the time and switch the display on or off
  if (minuteChanged()) {
    processTime();
    uint8_t hour = stuttgartTZ.hour();
    if (hour > 23 || hour < 6) {
      switchScreenOff();
    } else {
      switchScreenOn();
    }
  }
}

/**
 * Write the time to the display 
 */
void processTime() {
  String payload = stuttgartTZ.dateTime("G:i");
  if (oldTimeStr.length() == 0) {
    oldTimeStr = payload; 
  }

  displayText(payload, oldTimeStr, 20, 30, 4, 2);

  oldTimeStr = payload;
}

/**
 * Display the provided text on the display
 * 
 * param newText The new text to be displayed
 * param oldText The current displayed text to overwrite it using the background color
 * param x The x position of the text
 * param y The y position of the text
 * param fontSize The size of the font used
 * param screen The number of the screen to display the text on
 */
void displayText(String newText, String oldText, int x, int y, short fontSize, short screen) {
  if (screen == screenNum) {
    M5.Lcd.setTextColor(backgroundCol);
    M5.Lcd.setTextSize(fontSize);
    M5.Lcd.drawString(oldText, x, y, 4);
    
    M5.Lcd.setTextColor(mainTextCol);
    M5.Lcd.setTextSize(fontSize);
    M5.Lcd.drawString(newText, x, y, 4);
  }
}

/**
 * Process json data
 * param data the data as string in json format
 */
void processPuckData(String data) {
  const char* c_data = data.c_str();
  DynamicJsonDocument doc(384);
  deserializeJson(doc, c_data);

  JsonObject root_0 = doc[0];
  const char* root_0_measurement = root_0["measurement"]; // "ble_plant_sensor"

  JsonObject root_0_fields = root_0["fields"];
  float root_0_fields_temp = root_0_fields["temp"]; // -2.25
  int root_0_fields_light = root_0_fields["light"]; // 77
  int root_0_fields_humidity = root_0_fields["humidity"]; // 78
  int root_0_fields_battery = root_0_fields["battery"]; // 78
  int root_0_fields_mag = root_0_fields["mag"]; // 180
  
  const char* root_0_tags_location = root_0["tags"]["location"]; // "garden"
  
  const char* root_0_timestamp = root_0["timestamp"]; // "2020-12-11T12:31:16.178Z"

  String payload = String(root_0_fields_temp);
  if (oldOutdoorTempStr.length() == 0) {
      oldOutdoorTempStr = payload;
    }

  if (screenNum == 1) {
    displayText(payload, oldOutdoorTempStr, 20, 80, 4, 1);
  } else if (screenNum == 0) {
    displayText(payload, oldOutdoorTempStr, 200, 100, 1, 0);
  }

  oldOutdoorTempStr = payload;
}

/**
 * Process power state of a remote controled plug.
 * 
 * param data the data as string
 */
void processPower(String data) {
  int p = data.toInt();
  String text = "";

  if (oldDect210State.length() == 0) {
    oldDect210State = "AUS";
  }
  
  if (p <= 0) {
    text = "AUS";
  } else {
    text = "AN";
  }

  displayText(text, oldDect210State, 200, 60, 1, 0);

  oldDect210State = text;
}

/**
 * Process temperature data
 * 
 * param data the data as string
 */
void processTemperature(String data) {
  if (oldIndoorTempStr.length() == 0) {
    oldIndoorTempStr = data;
  }
  displayText(data, oldIndoorTempStr, 30, 80, 3, 0);

  oldIndoorTempStr = data;
}


/**
 * Handle the switching between the screens
 * 
 * param num the screen num to switch to
 */
void switchToScreen(short num) {
  String msg = "Swiching to screen " + num;
  log(msg);
  M5.Lcd.fillScreen(backgroundCol);
  switch (num) {
    case 1:
      screenTwo();
      break;
    case 2:
      screenThree();
      processTime();
      break;
    default:
      screenOne();
  }
}


/**
 * Draw the first screen
 */
void screenOne() {
  drawTopBar("Wohnzimmer");
  drawButtonBar("", "", ">");
}

/**
 * Draw the second screen
 */
void screenTwo() {
  drawTopBar("Draussen");
  drawButtonBar("<", "", ">");
}

/**
 * Draw the third screen
 */
void screenThree() {
  drawTopBar("Uhr");
  drawButtonBar("<", "", "");
}

/**
 * Draw top bar of the screens
 * 
 * param title The title displayed
 */
void drawTopBar(char* title) {
  // top bar
  M5.Lcd.fillRect(0, 0, 320, 30, DARKGREY);
  M5.Lcd.setCursor(80, 10);
  M5.Lcd.setTextColor(backgroundCol);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf(title);
}

/**
 * Draw bottom bar of the screens
 * 
 * param btnAText The text above the button A
 * param btnBText The text above the button B
 * param btnCText The text above the button C
 */
void drawButtonBar(char* btnAText, char* btnBText, char* btnCText) {
  // bottom bar
  M5.Lcd.fillRect(0, 210, 320, 30, DARKGREY);
  M5.Lcd.setTextColor(menuTextCol);
  M5.Lcd.setTextSize(2);

  M5.Lcd.setCursor(30, 220);
  M5.Lcd.printf(btnAText);

  M5.Lcd.setCursor(135, 220);
  M5.Lcd.printf(btnBText);

  M5.Lcd.setCursor(240, 220);
  M5.Lcd.printf(btnCText);  
}

/**
 * Switch the display off
 */
void switchScreenOff() {
  if(screenOn) {
    M5.Lcd.sleep();
    M5.Lcd.setBrightness(0);
    screenOn = false;
  }
}

/**
 * Switch the display on
 */
void switchScreenOn() {
  if(!screenOn) {
    M5.Lcd.wakeup();
    M5.Lcd.setBrightness(200);
    screenOn = true;
  }
}

/**
 * Write debug messages to Serial
 * 
 * param msg The message to diplay
 */
void log(String msg) {
  if (showDebug) {
    Serial.println(msg);
  }
}

/**
 * Set the color theme (Not used now)
 * 
 * param themeNum The number of the theme to switch to
 */
void setColorTheme(short themeNum) {
  switch(themeNum) {
    case 1:
      backgroundCol = TFT_DARKGREY;
      menuBarBgCol = TFT_DARKCYAN;
      menuTextCol = TFT_BLACK;
      mainAreaBgCol = TFT_DARKGREY;
      infoArea1BgCol = TFT_DARKGREY;
      infoArea2BgCol = TFT_DARKGREY;
      mainTextCol = TFT_MAROON;
      break;
    case 2:
      backgroundCol = TFT_DARKGREY;
      menuBarBgCol = TFT_DARKCYAN;
      menuTextCol = TFT_BLACK;
      mainAreaBgCol = TFT_DARKGREY;
      infoArea1BgCol = TFT_DARKGREY;
      infoArea2BgCol = TFT_DARKGREY;
      mainTextCol = TFT_MAROON;
      break;
    default:
      backgroundCol = TFT_BLACK;
      menuBarBgCol = TFT_DARKGREY;
      menuTextCol = TFT_BLACK;
      mainAreaBgCol = COL01;
      infoArea1BgCol = TFT_OLIVE;
      infoArea2BgCol = TFT_OLIVE;
      mainTextCol = TFT_YELLOW;
  }
}
