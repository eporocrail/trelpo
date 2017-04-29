
/*
  EltracoDecoder

  In a single software package functionality is available for two decoders:
  Turnout decoder with 4 extra sensors
  Sensor decoder for 8 sensors.

  Number of turnouts is controlling the program flow.

  March 2017 - Adding servo adjustment.


  /*************************************************************************************

  Aim is to develop a turnout and signaling system for model railroad track control.
  Communication is wireless.

  To disconnect the decoder from the controlling software in space and time the MOSQUITTO
  MQTT broker is used.

  Hardware setup is as simple as possible.

  Sofware is modularised.

*********************************************************************************************

  The decoder collects sensor information, activates relais and controls hobby RC servo's.

  Target platform is Wemos D1 mini.

  11 digital input/output pins, all pins have interrupt/pwm/I2C/one-wire supported(except D0)
  1 analog input(3.2V max input)
  a Micro USB connection
  compatible with Arduino IDE.
  pins are active LOW.

  All of the IO pins run at 3.3V:
  A levelshifter or voltage divider is needed.

  For a relay a pin is followed by a transistor/IC. A relay is driven with 5V.

  In stead of having one hardware module for diffferent applications the design aims for two PCB's specific to task
  were the Wemos has to be slotted into.

  8 pins are used.

  trelpo decoder                 Function
  trelpo sensor decoder          Scanning 8 sensors
  trelpo turnout decoder         Controlling 1 turnout and scanning 4 extra sensors
                                 Extra: ext button, ext LED, ext analogue sensor

  GPIO usage:

  Shield        pins                function
  Sensor         D0 .. D7           sensor
  Turnout        D0                 Sensor 1
                 D1                 ext LED
                 D2                 current detection block
                 D3, D4             sensor 2, 3
                 D5                 PWM signal for servo
                 D6                 relais
                 D7                 current detection turnout
                 D8                 ext button
                 A0                 ext analogue sensor

  ROCNET PROTOCOL

  packet payload content:
  byte 1  : groupId
  byte 2  : receiveIdH
  Byte 3  : receiveIdL
  byte 4  : sendIdH
  byte 5  : sendIdL
  byte 6  : group
  byte 7  : code
  byte 8  : length
  byte 9  : data1
  byte 10 : data2
  byte 11 : data3
  byte 12 : data4

  --byte 1 only used for large network. Normally 0.

  --byte 2 only used when more than 255 decoders. Normally 0.

  --byte 3 Rocrail Server default Id is 1

  Broadcast Id = 0
  Decoder   Id = 2 ... 255   Not used for switching decoder

  --byte 4 only used when more than 255 decoders. Normally 0.

  --byte 5 Rocrail Server default Id is 1

  Decoder Id     = 2 ... 255

  --byte 6

  groups
  code   description           remark                              MQTT topic
  0      Host                  Rocrail                             rocnet/ht
  3      Stationary decoders   Multiport for inputs and outputs    rocnet/dc
  7      Clock Fast            Clock                               rocnet/ck
  8      Sensor                Position determination              rocnet/sr
  9      Output                                                    rocnet/ot


  --byte 7

  Code:  bit 7 = 0
       bit 6 and bit 5 represent Type of code
       bit 4 .. 0:  5 bit command 0 .. 31

  Type: 0 - test
        1 - request
        2 - event
        3 - reply

  Sensor

  Actions
  code description data 1  data 2  data 3  data 4  data n
  1     report    addrH¹  addrL¹  status  port    identifier (RFID)

  ¹) Address of the reporting loco.
  The sensor ID is set in the header; Sender.
  A sensor secure ack is done with the stationary acknowledge msg:
  Code = 1
  Port = port

  Output

  Type     Value
  switch     0
  light      1
  servo      2

  Actions
  code description data 1  data 2  data 3
  0       off      type    value   address
  1       on       type    value   address

  --byte 8 Netto number of following data bytes.

  At a speed of 200 KmH a loc runs 64 mm per second in scale H0 and 35 mm per second in scale N.
  To come to a reliable detection reaction a point sensor must be scanned at least 20 times per second in H0
  and at least 10 times per second in scale N.

  For scale H0 it should be not larger than 50 and for scale N not larger than 100.
  Default will be 20.

  The Wemos Module needs no 3.3V power supply. To the 5V pin external powersupply can be connected.

  Addressing by Rocrail:

  Turnout decoder
    The current detector of the turnout has to be the first sensor. The current detector of the block sensor 2.
    S1, S2 and S3 are sensor 3, 4 and 5. (port numbers)

  Table: Sensor tab Interface - the lower part of the IP-address is inserted into field "Bus"
                                port number is inserted into field "Address"
                                e.g. (192.168.0.32 current detector block is inserted as Bus 32 Address 2)

    All outputs of Rocrail are numbered consecutively. "Bus 9" is used to address outputs.
    This also means that IP-address xxx.xxx.xxx.9 can not be used for a decoder.

  Table: Switches tab Interface - "9" is inserted into field "Bus".
                                  a unique number between 0 and 256 is inserted into field "Address"

  Sensor decoder

  Table: Sensor tab Interface - the lower part of the IP-address is inserted into field "Bus"
                                port number is inserted into field "Address"
                                e.g. (192.168.0.76 sensor 5 is inserted as Bus 76 Address 5)


  Author: E. Postma

  April 2017

*****/
#include <FS.h>                                                               //this needs to be first

#include <PubSubClient.h>
#include <Servo.h>
#include <stdlib.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <EEPROM.h>

extern "C" {
#include "user_interface.h"
}

Servo servo[4];
WiFiClient espClient;
PubSubClient client(espClient);

static char decoderId = 1;                                                   // also used in IP address decoder (check if IP address is available)
static char wiFiHostname[] = "ELTRACO-TO-001 ";                              // Hostname displayed in OTA port
static const char *ssid = "SSID";                                             // ssid WiFi network
static const char *password = "PASSWRD";                                  // password WiFi network
static const char *topicPub = "rocnet/rs";                                   // rocnet/rs for sensor
static const char *topicSub1 = "rocnet/ot";                                  // rocnet/ot for turnout control
static const char *topicSub2 = "rocnet/cf";                                  // rocnet/cf for servo configuration
static const char *topicSub3 = "rocnet/rs";                                  // rocnet/rs for sensor
static const char *MQTTclientId = (wiFiHostname);                            // MQTT client Id, differs per decoder, to be set by user

IPAddress mosquitto(192, 168, 2, 254);                                       // IP address Mosquitto
IPAddress _ip = IPAddress(192, 168, 2, decoderId);                           // IP address decoder
IPAddress _gw = IPAddress(192, 168, 2, 1);                                   // IP address gateway
IPAddress _sn = IPAddress(255, 255, 255, 0);                                 // subnet mask

//////////////////////////////////////// decoder function selection ///////////////////////////////////////////////////
// Sensor decoder
/*

  static const byte turnoutNr = 0;                                           // amount of turnouts differs per decoder type
  const byte addressTu[turnoutNr] = {};                                      // turnout addresses for three turnouys with current detection
  static const byte servoPin[turnoutNr] = {};                                // servo pin number
  static byte servoPos[1][2] = {                                             // pos straight, pos thrown
  {90, 90}
  };
  static const byte relais[turnoutNr] =  {};                                 // relais pin number
  static byte servoDelay[1] = {40};                                          // controls speed servo movement, higher is slower
  // sensor
  static const byte sensorNr = 8;                                            // amount of sensors differs per decoder type
  static const byte addressSr[sensorNr] = {1, 2, 3, 4, 5, 6, 7, 8};          // sensor addresses for sensor decoder,
  static const byte sensor[sensorNr] = {D0, D1, D2, D3, D4, D5, D6, D7};     // sensor pins with each a pull-up resistor
*/
// Turnout decoder

static const byte turnoutNr = 1;                                            // amount of turnouts differs per decoder type
static const byte addressTu[turnoutNr] = {1};                               // turnout addresses for three turnouys with current detection
static const byte servoPin[turnoutNr] = {D5};                               // servo pin number
static byte servoPos[turnoutNr][2] = {                                      // pos straight, pos thrown
  {45, 135}
};
static const byte relais[turnoutNr] =  {D6};                                // relais pin number
static byte servoDelay[turnoutNr] = {40};                                   // controls speed servo movement, higher is slower
// sensor
static const byte sensorNr = 5;                                             // amount of sensors differs per decoder type
static const byte addressSr[sensorNr] = {1, 2, 3, 4, 5};                    // sensor addresses for three turnouts with current detection
static const byte sensor[sensorNr] = {D7, D2, D0, D3, D4};                  // sensor pins with each a pull-up resistor

static const byte ledPin = D1;                                              // not used but made available for the user
static const byte buttonPin = D8;                                           // not used but made available for the user
static const byte analoguePin = A0;                                         // not used but made available for the user

//////////////////////////////////////// end of decoder function selection ///////////////////////////////////////////////////

static const int msgLength = 12;                                           // message number of bytes
static byte msgOut[msgLength];                                             // outgoing messages
static byte msgIn[msgLength];                                              // incoming messages
static boolean sendMsg = false;                                            // set true to publish message
static boolean forMe = false;                                              // flag for handling incoming message
static boolean firstMsg = true;                                            // flag to throw away first message (dump from rocrail)

static boolean sensorStatus[sensorNr];                                     // status sensor pins
static boolean sensorStatusOld[sensorNr];                                  // old status sensor pins
static unsigned long sensorProcessTime[sensorNr];                          // sensor timer
static byte sensorCountOff[sensorNr];                                      // counter negative sensor values
static boolean scan = false;                                               // sensorvalue
static const byte scanDelay = 5;                                           // delay in sensor scan processing
static const byte scanNegativeNr = 5;                                      // number of negative scan values for negative sensorstatus


static boolean order[turnoutNr];                                           // orders sent by rocrail
static boolean orderOld[turnoutNr];                                        // old orders to detect changes
static String(turnoutOrder) = "";                                          // used with debugging
static unsigned int relaisSwitchPoint[turnoutNr];                          // relais switch start time
static unsigned int relaisSwitchDelay[turnoutNr];                          // calculated relais switch time
static boolean turnoutInit[turnoutNr];                                     // servo initiation flag
static boolean movingStraight[turnoutNr];                                  // moving Straight
static boolean movingStraightOld[turnoutNr];
static boolean movingThrown[turnoutNr];                                    // moving Thrown
static boolean movingThrownOld[turnoutNr];
static byte currentPosition[turnoutNr];                                    // servo position
static byte targetPosition[turnoutNr];                                     // servo position to be reached
static unsigned long servoMoveTime[turnoutNr];                             // servo timer
static boolean executedThrown[turnoutNr];                                  // order thrown execution flag
static boolean executedStraight[turnoutNr];                                // order straight execution
static byte turnoutId = 0;                                                 // id turnout

static byte servoId = 0;                                                   // id servo
static byte servoAngle = 0;                                                // angle servo
static byte ackStraight = 0;                                               // ack straight
static byte ackThrown = 0;                                                 // ack thrown
static boolean straightFlag = false;                                       // control
static boolean thrownFlag = false;                                         // control
static byte servoPos1 = 0;                                                 // configuration servo straight position
static byte servoPos2 = 0;                                                 // configuration servo thrown position
static boolean servoInverted[1];                                           // if needed invert value of servo angle

static boolean debugFlag = true;                                           // display debug messages
static boolean configFlag = true;                                          // control servoconfiguration
///////////////////////////////////////////////////////////////set-up//////////////////////////////
void setup() {
  Serial.begin(9600);
  Serial.println();
  //  WiFi.hostname(wiFiHostname);                                        // both same effect, nodisplay on IP scanner
  wifi_station_set_hostname(wiFiHostname);                                // but sets host name visibale on serial monitor (OTA)
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  //tries to connect to last known settings
  //if it does not connect it starts an access point with the specified name
  //here  "trelpo" with password "loco"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("trelpo", "loco")) {
    Serial.println("failed to connect, reset and see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  Serial.println("Wifi connected");

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  EEPROM.begin(512);
  if ((EEPROM.read(255) == 0xFF) && (EEPROM.read(256) == 0xFF)) {                    //eeprom empty, first run
    Serial.println(" ******* EPROM EMPTY....Setting Default EEPROM values *****");
    for (int index = 0; index < 512; index++)
      EEPROM.write(index, 0);
    EEPROM.commit();
    delay(10000);
    WriteEEPROM();
  }

  ReadEEPROM();     //read the servo data

  memset(sensorStatus, 0, sizeof(sensorStatus));                                  // initialising arrays sensor decoder
  memset(sensorStatusOld, 0, sizeof(sensorStatusOld));
  memset(sensorProcessTime, 0, sizeof(sensorProcessTime));
  memset(msgOut, 0, sizeof(msgOut));
  memset(msgIn, 0, sizeof(msgIn));
  memset(sensorCountOff, 0, sizeof(sensorCountOff));

  msgOut[2] = 1;                                                                 // intializing fixed content outgoing message
  msgOut[4] = decoderId;
  msgOut[5] = 8;
  msgOut[6] = 1;
  msgOut[7] = 4;

  for (byte index = 0; index < sensorNr; index++) {                              // initialising sensor pins
    pinMode(sensor[index], INPUT_PULLUP);
  }

  if (turnoutNr > 0) {
    memset(movingStraight, 0, sizeof(movingStraight));                           // initialising extra arrays for turnout decoder
    memset(movingStraightOld, 0, sizeof(movingStraightOld));
    memset(movingThrown, 0, sizeof(movingThrown));
    memset(movingThrownOld, 0, sizeof(movingThrownOld));
    memset(relaisSwitchPoint, 0, sizeof(relaisSwitchPoint));
    memset(relaisSwitchDelay, 0, sizeof(relaisSwitchDelay));
    memset(executedThrown, 0, sizeof(executedThrown));
    memset(executedStraight, 0, sizeof(executedStraight));
    memset(servoMoveTime, 0, sizeof(servoMoveTime));
    memset(currentPosition, 0, sizeof(currentPosition));
    memset(targetPosition, 0, sizeof(targetPosition));
    memset(order, 0, sizeof(order));
    memset(orderOld, 0, sizeof(orderOld));
    memset(turnoutInit, 0, sizeof(turnoutInit));

    for (byte index = 0; index < turnoutNr ; index++) {                           // initialising relais pins
      pinMode(relais[index], OUTPUT);
      digitalWrite(relais[index], LOW);
    }
    for (byte index = 0; index < turnoutNr ; index++) {                           // initialising relais switch points
      relaisSwitchPoint[index] = ((servoPos[index][0] + servoPos[index][1]) / 2);
    }
    for (byte index = 0; index < turnoutNr ; index++) {                           // attaching servos
      servo[index].attach (servoPin[index]);
    }
    StartPosition();
  }

  WiFi.hostname(wiFiHostname);                                                    // both same effect, nodisplay on IP scanner
  //wifi_station_set_hostname(wiFiHostname);                                      // but sets host name visibale on serial monitor (OTA)

  client.setServer(mosquitto, 1883);
  client.setCallback(callback);

  //// begin of OTA ////////
  ArduinoOTA.setPort(8266);                                                      // Port defaults to 8266
  ArduinoOTA.setHostname(wiFiHostname);                                          // Hostname defaults to esp8266-[ChipID]
  //ArduinoOTA.setPassword((const char *)"123");                                 // No authentication by default
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  ////// end of OTA ////////
}
///////////////////////////////////////////////////////////////end of set-up////////////////////////
/////////////////////////////////////////////////////////////// program loop ////////////////////////////////
void loop() {
  ArduinoOTA.handle();                                                          // OTA handle must stay here in loop

  if (configFlag == false) ServoAdjust();
  ScanSensor();
  if (turnoutNr > 0) ProcessOrder();

  if (!client.connected()) {                                                   // maintain connection with Mosquitto
    reconnect();
  }
  client.loop();                                                               // content of client.loop can not be moved to function
  if (sendMsg == true) {                                                       // set sendMsg = 0 to transmit message
    if (debugFlag == true) {
      Serial.print(F("Publish msg ["));
      Serial.print(topicPub);
      Serial.print(F(" - DEC] "));
      for (int index = 0 ; index < msgLength ; index++) {
        Serial.print((msgOut[index]), DEC);
      }
      Serial.println();
    }
    if (client.publish(topicPub, msgOut, 12) != true) {                        // message is published
      Serial.println(F("fault publishing"));
    } else sendMsg = false;
  }
}
///////////////////////////////////////////////////////////// end of program loop ///////////////////////
/*

   ScanSensor

   function : read status of input pins. adjust status of sensor, generate outgoing message,
              sensorCountOff used to decide after how many scans "LOW" the sensor status is adjusted
              required for current detector
              set sendMsg = true to send message, set sendMsg = false for scan loop

   called by: loop

*/
void ScanSensor() {
  for (byte index = 0; index < sensorNr; index++) {
    if ( millis() > sensorProcessTime[index]) {
      sensorProcessTime[index] = millis() + scanDelay;
      sensorStatusOld[index] = sensorStatus[index];
      scan = !digitalRead(sensor[index]);
      if (scan == false) {
        sensorCountOff[index]++;
      }
      if (scan == true) {
        sensorCountOff[index] = 0;
        sensorStatus[index] = true;
      }
      if ((sensorCountOff[index]) > scanNegativeNr) {
        sensorStatus[index] = false;
      }
      if ((sensorStatusOld[index]) != (sensorStatus[index])) {
        sendMsg = true;
        msgOut[10] = sensorStatus[index];
        msgOut[11] = addressSr[index];
      }
    }
  }
} // end of Scansensor

/*
   ServoAdjust

   function : adjust straight and thrown position of servo. positions sent by servo tool.

   called by: loop

*/
void ServoAdjust() {
  servo[servoId].write(servoAngle);                                              // servo takes position sent by servo tool
  if (ackStraight == 1) {
    servoPos1 = servoAngle;                                                      // angle accepted when ack received
    straightFlag = true;
  }
  if (ackThrown == 1) {
    servoPos2 = servoAngle;                                                      // angle accepted when ack received
    thrownFlag = true;
  }
  if ((straightFlag == true) && (thrownFlag == true)) {
    if (servoPos1 > servoPos2) {
      servoInverted[servoId] = true;                                             // smallest angle is stored first
      servoPos[servoId][0] = servoPos2;
      servoPos[servoId][1] = servoPos1;
      servoPos[servoId][2] = 1;                                                  // if required, inverted is set
    } else {
      servoInverted[servoId] = false;
      servoPos[servoId][0] = servoPos1;
      servoPos[servoId][1] = servoPos2;
      servoPos[servoId][2] = 0;
    }
    if (debugFlag == true) {
      Serial.println();
      Serial.print("Servo nr: ");
      Serial.print(addressTu[servoId]);
      Serial.print(" - ");
      Serial.print(servoPos[servoId][0]);
      Serial.print(" - ");
      Serial.print(servoPos[servoId][1]);
      Serial.println();
    }
    EEPROM.write ((servoId * 3), servoPos[servoId][0]);                         // write values in EEPROM
    EEPROM.write ((servoId * 3 + 1), servoPos[servoId][1]);
    EEPROM.write ((servoId * 3 + 2), servoPos[servoId][2]);
    EEPROM.commit();
    delay(200);
    memset(msgOut, 0, sizeof(msgOut));
    msgOut[0] = addressTu[servoId];
    msgOut[4] = 1;
    sendMsg == true;
    straightFlag = false;
    thrownFlag = false;
    configFlag = true;
    ReadEEPROM();
    if (debugFlag == true) {
      Serial.println();
      Serial.println("servo adjusted");
      Serial.println();
    }
  }
} //end of servoAdjust

/*
   WriteEEPROM

   function : write initial servo values to EEPROM

   called by: setup

*/
void WriteEEPROM() {
  byte val = 0;
  for (int index = 0; index < 3; index++) {
    for (int ind = 0; ind < 3; ind++) {
      val = servoPos[index][ind];
    }
  }
  for (int tlr = 0; tlr < 9; tlr++) {
    EEPROM.write (tlr, val);
  }
  EEPROM.commit();
  delay(500);
} // end of WriteEEPROM

/*
   ReadEEPROM

   function : read servo values from EEPROM

   called by: setup, ServoAdjust

*/
void ReadEEPROM() {
  if (debugFlag == true) Serial.println();
  for (int index = 0; index < turnoutNr; index++) {
    if (debugFlag == true) {
      Serial.print("Servo nr: ");
      Serial.print(addressTu[index]);
      Serial.println();
    }
    for (int ind = 0; ind < 3; ind++) {
      servoPos[index][ind] = EEPROM.read((3 * index) + ind);
      if (debugFlag == true) {
        Serial.print(" - ");
        Serial.print(servoPos[index][ind]);
        Serial.print(" ");
        Serial.println();
      }
    }
  }
} // end of ReadEEPROM

/*
   ProcessOrder

   function : check orders if action required

   called by: loop
   calls    : Thrown, Straight

*/
void ProcessOrder() {
  for (byte index = 0; index < turnoutNr; index++) {
    if ((orderOld[index]) != (order[index])) {
      if ((order[index]) == 1) {
        Thrown(index);
      } else {
        Straight(index);
      }
    }
  }
} // end of ProcessOrder()

/*
   Thrown

   function : move servo into thrown position
              check if order already executed, compare current servo position with ordered position, signal start servo movement
              switch relais in middle of servo movement, signal end of movement, set flag order executed

   called by: ProcessOrder

   calls    : txMsgMove, txMsgMoveStop

*/
void Thrown(byte turnoutNr) {
  if ((executedThrown[turnoutNr]) == false) {
    currentPosition[turnoutNr] = servo[turnoutNr].read();
    if ((currentPosition[turnoutNr]) != (servoPos[turnoutNr][1]) && millis() >= servoMoveTime[turnoutNr]) {
      movingThrownOld[turnoutNr] = movingThrown[turnoutNr];
      movingThrown[turnoutNr] = true;
      servo[turnoutNr].write((currentPosition[turnoutNr]) + 1);
      servoMoveTime[turnoutNr] = millis() + servoDelay[turnoutNr];
      if ((movingThrownOld[turnoutNr]) != (movingThrown[turnoutNr])) {
        txMsgMove(turnoutNr);
      }
      if ((currentPosition[turnoutNr]) == (relaisSwitchPoint[turnoutNr])) digitalWrite(relais[turnoutNr], HIGH);
    }
    if ((currentPosition[turnoutNr]) == (servoPos[turnoutNr][1])) {
      executedThrown[turnoutNr] = true;
      movingThrownOld[turnoutNr] = movingThrown[turnoutNr];
      movingThrown[turnoutNr] = false;
      txMsgMoveStop(turnoutNr);
    }
  }
} // end of Thrown

/*

   function : move servo into straight position
              check if order already executed, compare current servo position with ordered position, signal start servo movement
              switch relais in middle of servo movement, signal end of movement, set flag order executed

   called by: ProcessOrder

   calls    : txMsgMove, txMsgMoveStop

*/
void Straight(byte turnoutNr) {
  if ((executedStraight[turnoutNr]) == false) {
    currentPosition[turnoutNr] = servo[turnoutNr].read();
    if ((currentPosition[turnoutNr]) != (servoPos[turnoutNr][0]) && millis() >= servoMoveTime[turnoutNr]) {
      movingStraightOld[turnoutNr] = movingStraight[turnoutNr];
      movingStraight[turnoutNr] = true;
      servo[turnoutNr].write((currentPosition[turnoutNr]) - 1);
      servoMoveTime[turnoutNr] = millis() + servoDelay[turnoutNr];
      if ((movingStraightOld[turnoutNr]) != (movingStraight[turnoutNr])) {
        txMsgMove(turnoutNr);
      }
      if ((currentPosition[turnoutNr]) == (relaisSwitchPoint[turnoutNr])) digitalWrite(relais[turnoutNr], LOW);
    }
    if ((currentPosition[turnoutNr]) == (servoPos[turnoutNr][0])) {
      executedStraight[turnoutNr] = true;
      movingStraightOld[turnoutNr] = movingStraight[turnoutNr];
      movingStraight[turnoutNr] = false;
      txMsgMoveStop(turnoutNr);
    }
  }
} // end of Straight

/*
   txMsgMove

   function : signal start movement servo to Rocrail

   called by: Thrown, Straight

*/
void txMsgMove(byte nr) {
  msgOut[11] = addressSr[nr];
  msgOut[10] = 1;
  sendMsg = true;
} // end of txMsgMove

/*
   txMsgMoveStop

   function : signal stop movement servo to Rocrail

   called by: Thrown, Straight

*/
void txMsgMoveStop(byte nr) {
  msgOut[11] = addressSr[nr];
  msgOut[10] = 0;
  sendMsg = true;
} // end of txMsgMoveStop

/*

   callback

   function : receive incoming message, test topic, test if message for me, suppres first message (some rocrail dump?)
              calculate turnout number, store order, set execute flag to false

*/
void callback(char *topic, byte *payload, unsigned int length) {
  if ((strncmp("rocnet/cf", topic, 9) == 0)) {
    forMe = false;                                        // check if address is contained in address array
    servoId = 0;
    for (byte index = 0; index < turnoutNr; index++) {
      if (((byte)payload[0]) == (addressTu[index])) {
        forMe = true;
        servoId = index;
      }
    }
    if (forMe == true) {
      if (debugFlag == true) {
        Serial.print("Message in [");
        Serial.print(topic);
        Serial.print("] ");
        for (byte index = 0; index < length; index++) {
          Serial.print(((char)payload[index]), DEC);
          if (index < length - 1) Serial.print(F("."));
        }
        Serial.println();
      }
      configFlag = false;
      servoAngle = ((byte)payload[1]);
      ackStraight = ((byte)payload[2]);
      ackThrown = ((byte)payload[3]);
      if ((ackStraight == 1) && (ackThrown == 1)) {
        straightFlag = false;
        thrownFlag = false;
        ackStraight = 0;
        ackThrown = 0;
      }
    }
  }
  if ((strncmp("rocnet/ot", topic, 9) == 0)) {
    forMe = false;                                        // check if address is contained in address array
    for (byte index = 0; index < turnoutNr; index++) {
      if (((byte)payload[11]) == (addressTu[index])) forMe = true;
    }
    if ((forMe == true) && (firstMsg == false)) {
      if (debugFlag == true) {
        Serial.print(F("Message in ["));
        Serial.print(topic);
        Serial.print(F("] "));
        for (byte index = 0; index < length; index++) {
          Serial.print(((char)payload[index]), DEC);
          if (index < length - 1) Serial.print(F("."));
        }
        Serial.println();
      }
    }
    turnoutId = ((((byte)payload[11]) % 4) - 1);        // turnout number is calculated
    if (turnoutId == 255) {
      turnoutId = 3;
    }
    orderOld[turnoutId] = order[turnoutId];
    order[turnoutId] = ((byte) payload[8]);             // switching order is stored
    if ((order[turnoutId]) == 1) {
      executedStraight[turnoutId] = false;
    } else {
      executedThrown[turnoutId] = false;
    }
    ////////// debugging ///////
    if (debugFlag == true) {
      if ((order[turnoutId]) == 1) {
        turnoutOrder = "thrown";
      } else {
        turnoutOrder = "straight";
      }
      if (turnoutNr > 0) {
        Serial.print(F("Message in ["));
        Serial.print(topic);
        Serial.print(F("] "));
        Serial.print(F("switch turnout - "));
        Serial.print(turnoutId + addressTu[0]);
        Serial.print(F(" to "));
        Serial.print(turnoutOrder);
        Serial.println();
      }
    }
    /////////// debugging ///////
  }
  firstMsg = false;
} // end of callback

/*
   re-establish connection with MWTTclientID.
   publish topic topicPub. subscribe to topic topicSub.
   when Mosquitto not available try again after 3 seconds

*/
void reconnect() {
  while (!client.connected()) {
    Serial.print("Establishing connection with Mosquitto ...");
    // Attempt to connect
    if (client.connect(MQTTclientId)) {
      Serial.println("connected");
      client.publish(topicPub, "connection established");       // Once connected, publish an announcement
      client.subscribe(topicSub1);                              // and resubscribe topic 1
      client.subscribe(topicSub2);                              // and resubscribe topic 2
      client.subscribe(topicSub3);                              // and resubscribe topic 3
    } else {
      Serial.print("no Broker");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      delay(3000);                                             // Wait 3 seconds before retrying
    }
  }
}
// end of reconnect

/*
   StartPosition()

   function : bring servo into straight position. switch relais off
   called by: Setup

*/
void StartPosition() {
  for (turnoutId = 0; turnoutId < turnoutNr ; turnoutId++) {
    servo[turnoutId].write(servoPos[turnoutId][0]);
  }
  for (turnoutId = 0; turnoutId < turnoutNr ; turnoutId++) {
    digitalWrite(relais[turnoutNr], LOW);
  }
} // end of StartPosition

/*
  display the content of a variable with associated text to the serial monitor

  debug tool
*/
void Dbg (int val, String text) {
  if (debugFlag == true) {
    Serial.println ("");
    Serial.println (F(" - - - - - - - - - - "));
    Serial.print (text);
    Serial.print (val);
    Serial.println ("");
  }
}  // end of Dbg

