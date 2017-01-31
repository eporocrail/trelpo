/*
  Trelpo v1.1

  Next to a configuration fore 8 sensors and 4 turnouts a third one is created. This one is a configuration of 3 turnouts with current detection.

  Also the initialization of the turnouts is placed into the setup. Does not work.

  Configuration has to be done outside setup. array indexes are constants.


  /*************************************************************************************

  Aim is to develop a turnout and signalling system for model railroad track control.
  Communication is wireless.

  To disconnect the decoder from the controlling software in space and time the MOSQUITTO
  MQTT broker is used.

  Hardware setup is as simple as possible.

  Sofware is modularised.

*********************************************************************************************

  The decoder collects sensor information or activates relais and controls hobby RC servo's.

  Target platform is Wemos D1 mini.

  11 digital input/output pins, all pins have interrupt/pwm/I2C/one-wire supported(except D0)
  1 analog input(3.2V max input)
  a Micro USB connection
  compatible with Arduino IDE.
  pins are active LOW.

  All of the IO pins run at 3.3V:
  For sensors all pins are high in No Detection (ND). Each sensor connects a pin to GND at detection(D).
  No consequence for IO voltage.

  For a relay a pin is followed by a transistor/IC. A relay is driven with 5V.

  In stead of having one module for all applications the design aims for different PCB's specific to task
  were the Wemos has to be slotted into.

  8 pins are used.

  trelpo shield                 Function                                          Hardware ID
  trelpo signalling shield      Detection 8 sensors                                   0 0
  trelpo turnout shield         Controlling 4 turnouts                                0 1

  GPIO usage:

  Hardware ID      pins        function
  0 0            D0 .. D7    read contact
  0 1            D0 .. D3    switch relay
  0 1            D4 .. D7    PWM signal for servo

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
  0       off     type    value   address
  1       on      type    value   address

  --byte 8 Netto number of following data bytes.

  At a speed of 200 KmH a loc runs 64 mm per second in scale H0 and 35 mm per second in scale N.
  To come to a reliable detection reaction a point sensor must be scanned at least 20 times per second in H0
  and at least 10 times per second in scale N.

  For debouncing purpose a delay is built in into the processing of a sensor scan. This delay is adjustable.
  For scale H0 it should be not larger than 50 and for scale N not larger than 100.
  Default will be 20.

  For each individual decoder a config file needs to be stored in the same directory as the main program.
  For creation of another decoder only a new config file has to be created and than the program
  can be compiled and uploaded

  For the decoder with three turnouts and current detection pinA0 has to be used.
  It can be found that this pin only reeads values up to 1V. Measurement revealed
  that this pin operates at 3.3V also.

  The Wemos Module needs no 3.3V power supply. To the 5V pin extenal powersupply can be connected.

  Scanning A0 in continuous loop disrupts WiFi. A pause between scans has to be applied. testing revealed that
  times shorter than debounce time are sufficient. Apply debounce time as delay for scanning A0.

  Author: E. Postma

  Januari 2017

*****/

#include <PubSubClient.h>
#include <Servo.h>
#include <stdlib.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include "trelpo_TO_009cfg.h"         // decoder config file

extern "C" {
#include "user_interface.h"
}

Servo servo[4];
WiFiClient espClient;
PubSubClient client(espClient);
/*
/////////////////////////////////////////// user adjustble variables /////////////////////////////////////////
static const char decoderId = 9;                                    // also used in IP address decoder (check if IP address is available)
static byte hardwareId = 2;                                         // 0: sensor 1: four turnouts 2: three turnouts with current detetction
char wiFiHostname[] = "Trelpo-TO-009";                              // Hostname displayed in OTA port
static const char *ssid = "XXX";                                    // ssid WiFi network
static const char *password = "pwd";                                // password WiFi network
static const char *topicPub = "rocnet/rs";                          // rocnet/rs for sensor
static const char *topicSub = "rocnet/ot";                          // rocnet/ot for turnout control
static const char *MQTTclientId = (wiFiHostname);                   // MQTT client Id, differs per decoder, to be set by user

IPAddress mosquitto(192, 168, 2, 193);                              // IP address Mosquitto
IPAddress decoder(192, 168, 2, decoderId);                          // IP address decoder
IPAddress gateway(192, 168, 2, 1);                                  // IP address gateway
IPAddress subnet(255, 255, 255, 0);                                 // subnet masker

// hardwareId = 0
//static const byte addressSr[8] = {71, 72, 73, 74, 75, 76, 77, 78}; // sensor addresses for sensor decoder,


// hardwareId = 1
/* 
static const byte addressTu[4] = {57, 58, 59, 60};                // turnout addresses for 4 turnouts
static byte servoDelay[4] = {40, 40, 40, 40};                     // controls speed servo movement, higher is slower
static byte servoPos[4][2] = {                                    // pos straight, pos thrown
  {10, 90},
  {90, 160},
  {75, 100},
  {45, 141}
};


// hardwareId = 2
static const byte addressTu[3] = {1, 2, 3};                       // turnout addresses for three turnouys with current detection
static const byte addressSr[3] = {71, 72, 73};                    // sensor addresses for three turnouts with current detection
static byte servoDelay[3] = {40, 40, 40};                         // controls speed servo movement, higher is slower
static byte servoPos[3][2] = {                                    // pos straight, pos thrown
  {10, 90},
  {90, 160},
  {75, 100},
};
static boolean debugFlag = false;                                   // set on and off in setup
static boolean caseFlag = false;                                    // controls display of control loop

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
static const int msgLength = 12;                      // message number of bytes
static byte msgOut[msgLength];
static byte msgIn[msgLength];
static boolean forMe = false;                         // flag for handling incoming message
static boolean firstMsg = true;                       // flag to throw away first message (dump from rocrail)

static const byte turnoutNr = 3;                      // amount of turnouts differs per hardware id
static const byte sensorNr = 2;                       // amount of sensor attached minus 1 for separate scanned pin A0
static boolean order[turnoutNr];                      // orders sent by rocrail
static boolean orderOld[turnoutNr];                   // old orders to detect changes

// hardwareId = 0
//static const byte sensor[sensorNr] = {D0, D1, D2, D3, D4, D5, D6, D7};     // sensor pins with each a pull-up resistor if hardwareId=0
//static boolean sensorStatus[sensorNr];              // status sensor pins if hardwareId=0
//static boolean sensorStatusOld[sensorNr];           // old status sensor pins if hardwareId=0

// hardwareId 1 
//static const byte servoPin[turnoutNr] = {D4, D5, D6, D7}; // servo pin number if hardwareId=1
//static const byte relais[turnoutNr] =  {D0, D1, D2, D3};  // relais pin number if hardwareId=1

// hardwareId = 2
static const byte servoPin[turnoutNr] = {D5, D6, D7}; // servo pin number if hardwareId=2
static const byte relais[turnoutNr] =  {D2, D3, D4};  // relais pin number if hardwareId=2
static const byte sensor[sensorNr] = {D0, D1};        // sensor pins with each a pull-up resistor if hardwareId=2
static boolean sensorStatus[sensorNr + 1];            // status sensor pins if hardwareId=2
static boolean sensorStatusOld[sensorNr + 1];         // old status sensor pins if hardwareId=2


static unsigned long sensorProcessTime = 0;           // sensor timer
static int analogProcessTime = 0;                     // analog pin timer 
static int analogValue = 0;                           // value analog pin

static String(turnoutOrder) = "";                     // used with debugging
static unsigned int relaisSwitchPoint[turnoutNr];     // relais switch start time
static unsigned int relaisSwitchDelay[turnoutNr];     // calculated relais switch time
static boolean turnoutInit[turnoutNr];                // servo initiation flag
static boolean movingStraight[turnoutNr];             // moving Straight
static boolean movingStraightOld[turnoutNr];
static boolean movingThrown[turnoutNr];               // moving Thrown
static boolean movingThrownOld[turnoutNr];
static boolean msgMoveStart[turnoutNr];               // flag to control signalling message
static boolean msgMoveStop[turnoutNr];                // flag to control signalling message
static byte currentPosition[turnoutNr];               // servo position
static byte targetPosition[turnoutNr];                // servo position to be reached
static unsigned long servoMoveTime[turnoutNr];        // servo timer
static boolean executedThrown[turnoutNr];             // order thrown execution flag
static boolean executedStraight[turnoutNr];           // order straight execution
static byte turnoutId = 0;
static const byte debounce = 20;                      // delay in sensor scan processing

static byte control = 0;                              // controls statemachine
///////////////////////////////////////////////////////////////set-up//////////////////////////////
void setup() {
  memset(sensorStatus, 0, sizeof(sensorStatus));
  memset(sensorStatusOld, 0, sizeof(sensorStatusOld));
  memset(movingStraight, 0, sizeof(movingStraight));
  memset(movingStraightOld, 0, sizeof(movingStraightOld));
  memset(movingThrown, 0, sizeof(movingThrown));
  memset(movingThrownOld, 0, sizeof(movingThrownOld));
  memset(relaisSwitchPoint, 0, sizeof(relaisSwitchPoint));
  memset(servoMoveTime, 0, sizeof(servoMoveTime));
  memset(currentPosition, 0, sizeof(currentPosition));
  memset(targetPosition, 0, sizeof(targetPosition));
  memset(order, 0, sizeof(order));
  memset(orderOld, 0, sizeof(orderOld));
  memset(msgOut, 0, sizeof(msgOut));
  memset(turnoutInit, 0, sizeof(turnoutInit));
  memset(msgMoveStart, 0, sizeof(msgMoveStart));
  memset(msgMoveStop, 0, sizeof(msgMoveStop));
  memset(relaisSwitchPoint, 0, sizeof(relaisSwitchPoint));
  memset(relaisSwitchDelay, 0, sizeof(relaisSwitchDelay));
  memset(executedThrown, 0, sizeof(executedThrown));
  memset(executedStraight, 0, sizeof(executedStraight));

  msgOut[2] = 1;
  msgOut[4] = decoderId;
  msgOut[5] = 8;
  msgOut[6] = 1;
  msgOut[7] = 4;

  switch (hardwareId) {
    case 0:
      control = 1;                                           // control=1 is sensor loop
      for (byte index = 0; index < sensorNr; index++) {
        pinMode(sensor[index], INPUT_PULLUP);
        digitalWrite(sensor[index], LOW);
      }
      break;

    case 1:
      control = 3;                                           // control=3 is loop for 4 turnouts
      for (byte index = 0; index < turnoutNr ; index++) {
        pinMode(relais[index], OUTPUT);
        digitalWrite(relais[index], LOW);
      }
      for (byte index = 0; index < turnoutNr ; index++) {
        relaisSwitchPoint[index] = ((servoPos[index][0] + servoPos[index][1]) / 2);
      }
      servo[0].attach (servoPin[0]);
      servo[1].attach (servoPin[1]);
      servo[2].attach (servoPin[2]);
      servo[3].attach (servoPin[3]);
      StartPosition();
      break;

    case 2:

      control = 6;
      for (byte index = 0; index < sensorNr; index++) {
        pinMode(sensor[index], INPUT_PULLUP);
        digitalWrite(sensor[index], LOW);
      }
      for (byte index = 0; index < turnoutNr ; index++) {
        pinMode(relais[index], OUTPUT);
        digitalWrite(relais[index], LOW);
      }
      for (byte index = 0; index < turnoutNr ; index++) {
        relaisSwitchPoint[index] = ((servoPos[index][0] + servoPos[index][1]) / 2);
      }
      servo[0].attach (servoPin[0]);
      servo[1].attach (servoPin[1]);
      servo[2].attach (servoPin[2]);
      StartPosition();
      break;
  }
  Serial.begin(115200);
  //WiFi.hostname(wiFiHostname);                  // both same effect, nodisplay on IP scanner
  wifi_station_set_hostname(wiFiHostname);        // but sets host name visibale on serial monitor (OTA)
  setup_wifi();

  //// begin of OTA ////////
  ArduinoOTA.setPort(8266);                       // Port defaults to 8266
  ArduinoOTA.setHostname(wiFiHostname);           // Hostname defaults to esp8266-[ChipID]
  //ArduinoOTA.setPassword((const char *)"123");  // No authentication by default
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

  client.setServer(mosquitto, 1883);
  client.setCallback(callback);

}
///////////////////////////////////////////////////////////////end of set-up////////////////////////
/////////////////////////////////////////////////////////////// program loop ////////////////////////////////
void loop() {
  ArduinoOTA.handle();                                      // OTA handle must stay in loop
  switch (control) {
    case 1:
      if (debugFlag == true) CaseMelding();
      ScanSensor();
      break;
    case 3:
      ServoInit();
      if (debugFlag == true) CaseMelding();
      break;
    case 4:
      ProcessOrder();
      if (debugFlag == true) CaseMelding();
      break;

    case 6:
      ServoInit();
      if (debugFlag == true) CaseMelding();
      break;
    case 7:
      ScanSensor();
      ScanA0();
      ProcessOrder();
      if (debugFlag == true) CaseMelding();
      break;
  }
  if (!client.connected()) {                               // maintain connection with Mosquitto
    reconnect();
  }
  client.loop();                                           // content of client.loop can not be moved to function
  if (control == 0) {                                      // set control =0 to transmit message
    if (debugFlag == true) {
      Serial.print(F("Publish msg ["));
      Serial.print(topicPub);
      Serial.print(F(" - DEC] "));
      for (int index = 0 ; index < msgLength ; index++) {
        Serial.print((msgOut[index]), DEC);
      }
      Serial.println();
    }
    if (client.publish(topicPub, msgOut, 12) != true) {
      Serial.println(F("fault publishing"));
    }
    switch (hardwareId) {
      case 0:
        control = 1;
        break;
      case 1:
        control = 4;
        for (byte index = 0; index < turnoutNr; index++) {
          msgMoveStop[index] = false;
          msgMoveStart[index] = false;
        }
        break;
      case 2:
        control = 7;
        for (byte index = 0; index < turnoutNr; index++) {
          msgMoveStop[index] = false;
          msgMoveStart[index] = false;
        }
        break;
    }
  }
}

///////////////////////////////////////////////////////////// end of program loop ///////////////////////
/*

   ScanA0

   function : scan sensor attached to pin A0

   called by: loop

*/
void ScanA0() {
  sensorStatusOld[2] = sensorStatus[2];
  if ( millis() > analogProcessTime) {
    analogValue = analogRead(A0);
    analogProcessTime = millis() + debounce;
  }
  if (analogValue < 300) (sensorStatus[2] = HIGH);
  else (sensorStatus[2] = LOW);
  if (sensorStatus[2] != sensorStatusOld[2]) {
    msgOut[10] = sensorStatus[2];
    msgOut[11] = addressSr[2];
    if ( millis() > sensorProcessTime) {
      sensorProcessTime = millis() + debounce;
      control = 0;
    }
  }
  if (control == 0) {
  } else control = 7;
} // end of ScanA0

/*

   ScanSensor

   function : read status of input pins. adjust status of sensor, generate outgoing message,
              transmit messageafter debounce period, set control=0 to send message, set control=1 for scan loop

   called by: loop

*/
void ScanSensor() {
  for (byte index = 0; index < sensorNr; index++) {
    sensorStatusOld[index] = sensorStatus[index];
    sensorStatus[index] = !digitalRead(sensor[index]);
    if (sensorStatusOld[index] != sensorStatus[index]) {
      msgOut[10] = sensorStatus[index];
      msgOut[11] = addressSr[index];
      if ( millis() > sensorProcessTime) {
        sensorProcessTime = millis() + debounce;
        control = 0;
      }
    }
    if (control == 0) {
      break;
    } else {
      if (hardwareId == 0) {
        control = 1;
      } else control = 7;
    }
  }
}
// end of Scansensor

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
              switch relais in middle of servo movement, signalend of movement, set flag order executed

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
        if (msgMoveStart[turnoutNr] == false) {
          txMsgMove(turnoutNr);
          msgMoveStart[turnoutNr] = true;
        }
      }
      if ((currentPosition[turnoutNr]) == (relaisSwitchPoint[turnoutNr])) digitalWrite(relais[turnoutNr], HIGH);
    }
    if ((currentPosition[turnoutNr]) == (servoPos[turnoutNr][1])) {
      executedThrown[turnoutNr] = true;
      movingThrownOld[turnoutNr] = movingThrown[turnoutNr];
      movingThrown[turnoutNr] = false;
      if (msgMoveStop[turnoutNr] == false) {
        txMsgMoveStop(turnoutNr);
        msgMoveStop[turnoutNr] = true;
      }
    }
  }
} // end of Thrown

/*

   function : move servo into straight position
              check if order already executed, compare current servo position with ordered position, signal start servo movement
              switch relais in middle of servo movement, signalend of movement, set flag order executed

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
        if (msgMoveStart[turnoutNr] == false) {
          txMsgMove(turnoutNr);
          msgMoveStart[turnoutNr] = true;
        }
      }
      if ((currentPosition[turnoutNr]) == (relaisSwitchPoint[turnoutNr])) digitalWrite(relais[turnoutNr], LOW);
    }
    if ((currentPosition[turnoutNr]) == (servoPos[turnoutNr][0])) {
      executedStraight[turnoutNr] = true;
      movingStraightOld[turnoutNr] = movingStraight[turnoutNr];
      movingStraight[turnoutNr] = false;
      if (msgMoveStop[turnoutNr] == false) {
        txMsgMoveStop(turnoutNr);
        msgMoveStop[turnoutNr] = true;
      }
    }
  }
} // end of Straight

/*
   txMsgMove

   function : signal start movement servo to Rocrail

   called by: Thrown, Straight

*/
void txMsgMove(byte nr) {
  msgMoveStart[nr] = true;
  msgOut[11] = addressSr[nr];
  msgOut[10] = 1;
  control = 0;
} // end of txMsgMove

/*
   txMsgMoveStop

   function : signal stop movement servo to Rocrail

   called by: Thrown, Straight

*/
void txMsgMoveStop(byte nr) {
  msgMoveStop[nr] = true;
  msgOut[11] = addressSr[nr];
  msgOut[10] = 0;
  control = 0;
}

/*
   ServoInit()
   function : move all servos one cycle. set servos in straight position. switch relais on and off.

   called by: setup

*/
void ServoInit() {
  for (turnoutId = 0; turnoutId < turnoutNr ; turnoutId++) {
    if (turnoutInit[turnoutId] == false) {
      currentPosition[turnoutId] = servo[turnoutId].read();
      if ((currentPosition[turnoutId]) == (servoPos[turnoutId][0])) (targetPosition[turnoutId]) = (servoPos[turnoutId][1]);
      if ((currentPosition[turnoutId]) == (servoPos[turnoutId][1])) (targetPosition[turnoutId]) = (servoPos[turnoutId][0]);
      if ((currentPosition[turnoutId]) != (targetPosition[turnoutId]) && millis() >= servoMoveTime[turnoutId]) {
        if ((currentPosition[turnoutId]) < (targetPosition[turnoutId])) {
          servo[turnoutId].write((currentPosition[turnoutId]) + 1);
          if ((currentPosition[turnoutId]) == (relaisSwitchPoint[turnoutId])) digitalWrite(relais[turnoutId], HIGH);
        }
        if ((currentPosition[turnoutId]) > (targetPosition[turnoutId])) {
          servo[turnoutId].write((currentPosition[turnoutId]) - 1);
          if ((currentPosition[turnoutId]) == (relaisSwitchPoint[turnoutId])) digitalWrite(relais[turnoutId], LOW);
          if ((servo[turnoutId].read()) == (targetPosition[turnoutId])) {
            turnoutInit[turnoutId] = true;
            if ((turnoutInit[0]) == true && (turnoutInit[1]) == true && (turnoutInit[2]) == true) {
              if (debugFlag == true) Serial.println("turnout initialization ready");
              if (hardwareId == 1) {
                control = 4;
              } else control = 7;
            }
          }
        }
        servoMoveTime[turnoutId] = millis() + servoDelay[turnoutId];
      }
    }
  }
} // end of ServoInit()

/*

   callback

   function : receive incoming message, test topic, test if message for me, suppres first message (some rocrail dump?)
              calculate turnout number, store order, set execute flag tofalse, set control=4 into order processing loop

*/
void callback(char *topic, byte * payload, unsigned int length) {
  if (*topic != *topicSub) {
    Serial.print(F("Wrong toppic ["));
    Serial.print(topicSub);
    Serial.println(F("] "));
    Serial.print(F("Message in ["));
    Serial.print(topic);
    Serial.print(F("] "));
    for (byte index = 0; index < length; index++) {
      Serial.print(((char)payload[index]));
    }
    Serial.println();
  } else {
    forMe = false;                                        // check if address is contained in address array
    for (byte index = 0; index < turnoutNr; index++) {
      if (((byte)payload[11]) == (addressTu[index])) forMe = true;
    }
    if ((forMe == true) && (firstMsg == false)) {
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
      control = 4;                                         // order processing loop
      ////////// debugging ///////
      if (debugFlag == true) {
        if ((order[turnoutId]) == 1) {
          turnoutOrder = "thrown";
        } else {
          turnoutOrder = "straight";
        }
        if (hardwareId == 1) {
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
  }
} // end of callback



/*
   setup_wifi

   connect to network, install static IP address

*/
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  WiFi.config(decoder, gateway, subnet);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println(F("WiFi connected"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("hostname: "));
  Serial.println(WiFi.hostname());
}
// end of setup_wifi

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
      client.publish(topicPub, "connection established");      // Once connected, publish an announcement
      client.subscribe(topicSub);                              // and resubscribe
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

/*
   CaseMelding

   function: trace trough control levels

   debugging tool

*/
void CaseMelding() {
  if (caseFlag == true) {
    Serial.print ("we zijn in HWI: ");
    Serial.print (hardwareId);
    Serial.print (" in case: ");
    Serial.print (control);
    Serial.println();
  }
} // end of CaseMelding

