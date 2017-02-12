/*

  This software constitutes an RFID reader based on the MFC522 module.
  Target platform is the Wemos D1 mini module.

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

  The Wemos module needs no 3.3V power supply. To the 5V pin external powersupply can be connected.
  The MFC522 module reqires 3.3V powersupply. Apply separate DC-DC convertor.


  wiring the MFRC522 to Wemos D1 Mini
  GPIO5    D1   SCL
  GPIO4    D2   SDA
  GPIO14   D5   SCK
  GPIO12   D6   MISO
  GPIO13   D7   MOSI
  GND     = GND
  3.3V    = 3.3V

  The RFID code is translated into "HEXA" to ensure that Rocrail displays the info correctly.
  The code is made up out of four, seven or ten bytes. It turned out that only the first fout bytes were relevant.
  

*/


#include <PubSubClient.h>
#include <stdlib.h>
#include <stdio.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <SPI.h>
#include "MFRC522.h"
#include "trelpo_TO_009cfg.h"                                 // decoder config file

extern "C" {
#include "user_interface.h"
}

WiFiClient espClient;
PubSubClient client(espClient);

#define RST_PIN  5  
#define SS_PIN  4   

MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance

static byte byteIn = 0;
uint8_t oldUid[tagLen];        // stores the old UID
static String inTag[tagLen];
static String inTagH[tagLen];

static const int msgLength = 20;                      // message number of bytes
static byte msgOut[msgLength];
static byte msgIn[msgLength];

static byte readByte = 0;                             // variable to read a single byte
static byte recTag = 0;                               // recognised tag index nr
static byte msgFlag = 0;                              // control message loop
static byte control = 1;                              // controls program flow

static boolean msgONSent = false;                     // message ON sent OK
static boolean msgOFFSent = false;                    // message OFF sent OK
static boolean flashFlag = false;                     // start flasher
static boolean scanReady = true;                      // access scanning

static unsigned long recTime = 0;                     // tag recognition time
static unsigned long flashTime = 0;                   // flash timer

/*
  /////////////////////////////////////////// user adjustble variables /////////////////////////////////////////
  static const char decoderId = 9;                             // also used in IP address decoder (check if IP address is available)
char wiFiHostname[] = "Trelpo-TO-009";                       // Hostname displayed in OTA port
static const char *ssid = "xxxx";                            // ssid WiFi network
static const char *password = "password";                    // password WiFi network
static const char *topicPub = "rocnet/rs";                   // rocnet/rs for sensor
static const char *topicSub = "rocnet/rs";                   // rocnet/rs for sensor message published is received
static const char *MQTTclientId = (wiFiHostname);            // MQTT client Id, differs per decoder, to be set by user

IPAddress mosquitto(192, 168, 2, 193);                       // IP address Mosquitto
IPAddress decoder(192, 168, 2, decoderId);                   // IP address decoder
IPAddress gateway(192, 168, 2, 1);                           // IP address gateway
IPAddress subnet(255, 255, 255, 0);                          // subnet masker

static const int tagLen = 7;                                 // tag number of bytes
static const int offset = 3;                                 // number of irrelevant last bytes of tag
static const byte addressSr = 71;                            // sensor address in Rocrail
static const int tagAmount = 5;                              // amount of tags
static int releaseTime = 3000;                               // time before scanning next tag
static const int flashDuration = 100;                        // flash at tag recognition
static const byte LedPin = D3;
static boolean blankTag = false;                             // if true unknown tag is scanned and output to serial monitor

static String tag[tagAmount][tagLen] = {                     // tag numbers, only first fout bytes are relevant
  {"04", "1e", "3a", "d2"},
  {"04", "22", "39", "d2"},
  {"04", "14", "3a", "d2"},
  {"04", "1c", "3a", "d2"},
  {"04", "24", "39", "d2"}
};

static boolean debugFlag = true;                             // support debugging
static boolean caseFlag = false;                             // controls display of control loop
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
///////////////////////////////////////////////////////////////set-up//////////////////////////////
void setup() {

  memset(oldUid, 0, sizeof(oldUid));
  memset(inTag, 0, sizeof(inTag));
  memset(msgOut, 0, sizeof(msgOut));
  memset(msgIn, 0, sizeof(msgIn));

  msgOut[0] = 0;
  msgOut[1] = 0;
  msgOut[2] = 1;
  msgOut[3] = 0;
  msgOut[4] = decoderId;
  msgOut[5] = 8;
  msgOut[6] = 1;
  msgOut[7] = (msgLength - 8);
  msgOut[8] = 0;
  msgOut[9] = 0;
  msgOut[11] = addressSr;

  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, LOW);

  SPI.begin();                                    // Init SPI bus
  mfrc522.PCD_Init();                             // Init MFRC522

  Serial.begin(9600);                             // Init serial communications

  //WiFi.hostname(wiFiHostname);                  // both same effect, nodisplay on IP scanner
  wifi_station_set_hostname(wiFiHostname);        // but sets host name visibale on serial monitor (OTA)
  Setup_wifi();

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
  client.setCallback(Callback);



}
///////////////////////////////////////////////////////////////end of set-up////////////////////////
/////////////////////////////////////////////////////////////// program loop ////////////////////////////////
void loop() {
  ArduinoOTA.handle();                                      // OTA handle must stay in loop
  switch (control) {
    case 1:
      if (debugFlag == true) CaseMelding();
      if (recTag == 0) ReadTag();
      ReadTag();
      break;
    case 2:
      if (debugFlag == true) CaseMelding();
      if (flashFlag == true) Flash();
      else SendMsgOn();
      break;
    case 3:
      SendMsgOff();
      if (debugFlag == true) CaseMelding();
      break;
  }
  if (!client.connected()) {                               // maintain connection with Mosquitto
    Reconnect();
  }
  client.loop();                                           // content of client.loop can not be moved to function
  if (control == 0) {                                      // set control = 0 to transmit message
    if (debugFlag == true) {
      Serial.print(F("Publish msg ["));
      Serial.print(topicPub);
      Serial.print(F(" - DEC] "));
      for (int index = 0 ; index < msgLength ; index++) {
        Serial.print((msgOut[index]), DEC);
        if (index < msgLength - 1) Serial.print(F("."));
      }
      Serial.println();
    }
    switch (msgFlag) {
      case 0:
        control = 1;
        break;
      case 1:
        msgONSent = client.publish(topicPub, msgOut, msgLength);
        if (msgONSent == false) Serial.println(F("fault publishing"));
        else control = 3;
        break;
      case 2:
        msgOFFSent = client.publish(topicPub, msgOut, msgLength);
        if (msgOFFSent == false) Serial.println(F("fault publishing"));
        else {
          recTag = 0;
          control = 1;
        }
        break;
    }
  }
}
///////////////////////////////////////////////////////////// end of program loop ///////////////////////


/*

  readTag

  function : read tag

  called by: loop

  calls    : mfrc522.PICC_IsNewCardPresent, mfrc522.PICC_ReadCardSerial, CompareUid, DumpByteArray,
             CopyUid, ChechTag

*/
void ReadTag() {
  if ( mfrc522.PICC_IsNewCardPresent () && mfrc522.PICC_ReadCardSerial ()) {
    if ( ! CompareUid ( mfrc522.uid.uidByte, oldUid, mfrc522.uid.size)) {     // if the card-ID differs from the previous card-ID
      CopyUid ( mfrc522.uid.uidByte, oldUid, mfrc522.uid.size);
      if (debugFlag == true) {
        Serial.println();
        Serial.print("Tag number - ");
        PrintTag();
        Serial.println ();
      }
      CheckTag();
    }
  }
}  //  end of ReadTag

/*

   CompareUid

   function : compare two RFID UIDs

   called by: readTag()

*/
bool CompareUid (byte * buffer1, byte * buffer2, byte bufferSize) {
  for (byte index = 0; index < bufferSize; index++) {
    if (buffer1[index] != buffer2[index])
      return false;                          // if only one byte differs then leave the function and return "false"
  }
  return true;
}
//  end of CompareUid

/*

   CopyUid

   function : copy one RFID UID to another one

   called by: readTag()

*/
void CopyUid (byte * buffIn, byte * buffOut, byte bufferSize) {
  char T;
  byte len;
  for (byte index = 0; index < bufferSize; index++) {
    buffOut[index] = buffIn[index];
    T = (buffIn[index]);
    inTag[index] = String(T, HEX);
    len = (inTag[index]).length();        // calculates the length of string str
    if (len < 2) {                        // when str is only one character a "0" is added to the front
      inTag[index] = "0" + inTag[index];
    }
  }
  if (bufferSize < tagLen) {
    for (byte index = bufferSize; index < tagLen; index++) buffOut[index] = 0;
  }
}
// end of CopyUid

/*
   checkTag

   function : goes through tag table, action when recognised

   called by: readTag

   calls    : CompareTag

*/
void CheckTag() {
  control = 1;
  for (byte index = 0 ; index < tagAmount ; index++) {
    if (CompareTag(inTag, tag[index]) == true) {
      recTag = index + 1;
      recTime = millis();
      flashTime = millis();
      flashFlag = true;
      msgONSent = false;
      control = 2;
      if (debugFlag == true) {
        Serial.println();
        Serial.print("Tag number - ");
        PrintTag();
        Serial.print(" - acknowledged");
        Serial.println();
      }
    }
  }
}  //  end of CheckTag

/*

   CompareTag

   function : compares tag read with tag known

   called by: checktag

*/
boolean CompareTag(String scannedTag[tagLen], String knownTag[tagLen]) {
  boolean match = false;
  byte byteCount = 0;
  for (byte index = 0 ; index < (tagLen - offset) ; index++) {
    if (scannedTag[index] == knownTag[index]) byteCount++;
  }
  if (byteCount == (tagLen - offset)) match = true;
  return match;
}  // end of CompareTag

/*
   printTag

   function : print scanned tag to serial monitor

   called by: checkTag, readTag

*/
void PrintTag () {
  for (byte index = 0 ; index < (tagLen - offset) ; index++) {
    Serial.print ((inTag[index]));
    if (index < (tagLen - offset) - 1) (Serial.print("."));
  }
}  //  end of PrintTag

/*
   SendMsgOn

   function : send message after recognition

   called by: loop

*/
void SendMsgOn() {
  control = 2;
  if (recTag > 0) {
    msgOut[10] = 1;
    MsgBody();
    msgFlag = 1;
    control = 0;
  }
}  // end of SendMsgOn

/*
   SendMsgOff

   function : send message after recognition

   called by: loop

*/
void SendMsgOff() {
  control = 3;
  if (recTag > 0) {
    if ((millis() - recTime) > releaseTime) {
      msgOut[10] = 0;
      MsgBody();
      control = 0;
      msgFlag = 2;
    }
  }
}  // end of TagUnblock

/*
   MsgBody

   function : add tag number to message content

   called by: SendMsgOn, SendMsgOff

*/
void MsgBody() {
  msgOut[12] = String(tag[recTag - 1][0])[0];
  msgOut[13] = String(tag[recTag - 1][0])[1];
  msgOut[14] = String(tag[recTag - 1][1])[0];
  msgOut[15] = String(tag[recTag - 1][1])[1];
  msgOut[16] = String(tag[recTag - 1][2])[0];
  msgOut[17] = String(tag[recTag - 1][2])[1];
  msgOut[18] = String(tag[recTag - 1][3])[0];
  msgOut[19] = String(tag[recTag - 1][3])[1];
}

/*
   Flash

   function : flash on tag recognised

   called by: loop

*/
void Flash() {
  digitalWrite (LedPin, HIGH);
  if (millis() - flashTime >= flashDuration) {
    flashTime = millis();
    digitalWrite(LedPin, LOW);
    flashFlag = false;
  }
}  // end of Flash

/*

   Callback

   function : receive incoming message, test topic, display incoming message if debugFlag is set


*/
void Callback(char *topic, byte * payload, unsigned int length) {
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
    if (((byte)payload[11]) == (addressSr)) {
      if (debugFlag == true) {
        Serial.print(F("Message in  ["));
        Serial.print(topic);
        Serial.print(F(" - DEC] "));
        for (byte index = 0; index < msgLength; index++) {
          Serial.print(((char)payload[index]), DEC);
          if (index < msgLength - 1) Serial.print(F("."));
        }
        Serial.println();
      }
    }
  }
} // end of Callback

/*

   Setup_wifi

   connect to network, install static IP address

*/
void Setup_wifi() {
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
// end of Setup_wifi

/*

   Reconnect

   function : re-establish connection with MWTTclientID ,publish topic topicPub. subscribe to topic topicSub.
              when Mosquitto not available try again after 3 seconds

   called by: loop
*/
void Reconnect() {
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
// end of Reconnect

/*

   CaseMelding

   function: trace trough control levels

   debugging tool

*/
void CaseMelding() {
  if (caseFlag == true) {
    Serial.print ("we zijn in case: ");
    Serial.print (control);
    Serial.println();
  }
} // end of CaseMelding

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
