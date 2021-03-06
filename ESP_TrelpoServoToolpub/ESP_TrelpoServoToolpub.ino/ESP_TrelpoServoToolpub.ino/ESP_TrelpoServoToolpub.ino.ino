/*
  Eltraco Servo Remote

  This software constitutes a Servo adjustment tool.
  Address of decoder is read from 8 dipswitches.
  Target platform is the Wemos D1 mini module.

  Message structure:
  byte 0: decoderId of Decoder
  byte 1: StraightPos    1 .. 255
  byte 2: StraightAck       1
  byte 3: ThrownAck         1

  The address of the servo is selected with a 8-bit dipswitch.
  During selection process LED is continuously ON.

  The address is acknowledged with the "ack" push button connected to 5, switching to 3.3V.
  LED flashes 3 times.

  The messages with the angle for the servo are generated by turning the knob of a potmeter.
  The mid point connected to A0.
  The left point connected to 3.3V and the right point connected to GND.

  The "ack" button is used to confirm the angle for the "straight" position of the turnout.
  LED flashes 3 times.

  The next message for the servo is generated with the potmeter.

  The "ack" button is used to confirm the angle for the "thrown" position of the turnout.
  LED flashes 3 times.

  Use the "ack" button to conclude the sequence. The address selection process starts again.


    y = (x >> n) & 1;    // n=0..15.  stores nth bit of x in y.  y becomes 0 or 1.
    x &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
    x |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
    x ^= (1 << n);       // toggles nth bit of x.  all other bits left alone.
    x &= (1<<(n+1))-1;   // leaves alone the lowest n bits of x; all higher bits set to 0.
    x = ~x;              // toggles ALL the bits in x.


*/
#include <FS.h>                   //this needs to be first

#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <ESP_PCF8574.h>

#define SDA_PIN D1
#define CLK_PIN D2
#define PCF8574_ADDRESS (0x20)

ESP_PCF8574 dipSwitch;


WiFiClient espClient;
PubSubClient client(espClient);

char wiFiHostname[] = "ServoTool";                           // Hostname displayed in OTA port
static const char *ssid = "SSID";                            // ssid WiFi network
static const char *password = "PASSWRD";                     // password WiFi network
static const char *topicPub = "rocnet/cf";                   // rocnet/cf for servo tool
static const char *topicSub = "rocnet/cf";                   // rocnet/cf for servo tool
static const char *MQTTclientId = (wiFiHostname);            // MQTT client Id, differs per decoder, to be set by user

IPAddress mosquitto(192, 168, 2, 254);                       // IP address Mosquitto
IPAddress _ip = IPAddress(192, 168, 2, 252);                 // IP address decoder
IPAddress _gw = IPAddress(192, 168, 2, 1);                   // IP address gateway
IPAddress _sn = IPAddress(255, 255, 255, 0);                 // subnet mask

static boolean debugFlag = true;
static boolean caseFlag = false;

extern "C" {
#include "user_interface.h"
}

///////////////////////////////////////////////////////////////////////////////////////


static const int msgLength = 4;                              // message number of bytes
static byte msgOut[msgLength];                               // outgoing messages
static byte msgIn[msgLength];                                // incoming messages
static byte msgFlag = 0;                                     // control sending message
static byte sendNr = 1;
static byte sendCntr = 0;

static boolean dip[8];                                       // dipswitch 
static byte const Led = D0;                                  // acknowledge LED
static byte const ackButton = D5;                            // acknowledge button
static byte const potPin = A0;                               // potentiometer read pin

static int potVal = 0;                                       // pot meter value
static byte potValOld = 0;
static byte butVal = 0;                                      // button value

static byte decoderAddress = 0;
static byte decoderAddressOld = 0;
static boolean addressSelected = false;
static boolean msgPotSent = false;
static boolean msgStraightSent = false;
static boolean msgThrownSent = false;
static byte confirmCount = 0;

static unsigned long readPotTimer = millis();                 // timers
static unsigned long flashTimer = millis();
static unsigned long addressSelectTimer = millis();
static unsigned long confirmTimer = millis();
static unsigned long nextAddressTimer = millis();

static const byte readPotDelay = 100;                        // delays
static const byte addressSelectDelay = 100;
static const byte confirmDelay = 250;
static const byte nextAddressDelay = 250;

boolean ackConfirm = false;
boolean ackConfirmOld = false;


static const byte flashDuration = 100;                      // flash
static byte flashCounter = 0;
static const byte flashAmount = 3;

static byte control = 1;
static byte stepBack = 0;
static byte ledState = LOW;
///////////////////////////////////////////////////////////////set-up//////////////////////////////
void setup() {
  Serial.begin(9600);

  Serial.println();
  //  WiFi.hostname(wiFiHostname);                  // both same effect, nodisplay on IP scanner
  wifi_station_set_hostname(wiFiHostname);          // but sets host name visibale on serial monitor (OTA)
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  //tries to connect to last known settings
  //if it does not connect it starts an access point with the specified name
  //here  "trelpo" with password "loco"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("trelpo", "loco")) {
    Serial.println("failed to connect, we should reset and see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...Wifi");

  Serial.println("local ip");
  Serial.println(WiFi.localIP());


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
  wifi_station_set_hostname(wiFiHostname);        // sets host name visibale on serial monitor (OTA)

  client.setServer(mosquitto, 1883);
  client.setCallback(Callback);

  memset(msgOut, 0, sizeof(msgOut));
  memset(msgIn, 0, sizeof(msgIn));
  memset(dip, 0, sizeof(dip));

  dipSwitch.begin(PCF8574_ADDRESS, SDA_PIN, CLK_PIN);

  pinMode(ackButton, INPUT_PULLUP);
  pinMode(Led, OUTPUT);
  digitalWrite(Led, LOW);
}
///////////////////////////////////////////////////////////////end of set-up////////////////////////
/////////////////////////////////////////////////////////////// program loop ////////////////////////////////
void loop() {
  ArduinoOTA.handle();                                      // OTA handle must stay in loop
  switch (control) {
    case 1:
      SelectAddress();
      break;
    case 2:
      ReadPot();
      Confirm();
      break;
    case 9:
      if (flashCounter < flashAmount) Flash();
      else control = stepBack;
      break;
  }
  if (!client.connected()) {                               // maintain connection with Mosquitto
    Reconnect();
  }
  client.loop();                                           // content of client. loop can not be moved to function
  if (control == 0) {                                      // set control =0 to transmit message
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
        msgPotSent = client.publish(topicPub, msgOut, msgLength);
        if (msgPotSent == false) Serial.println(F("fault publishing"));
        else control = 2;
        break;
      case 2:
        msgStraightSent = client.publish(topicPub, msgOut, msgLength);
        if (msgStraightSent == false) Serial.println(F("fault publishing"));
        else {
          Serial.println(F("Straight position sent"));
          msgOut[2] = 0;
          control = 2;
        }
        break;
      case 3:
        msgThrownSent = client.publish(topicPub, msgOut, msgLength);
        if (msgThrownSent == false) Serial.println(F("fault publishing")); {
          Serial.println(F("Thrown position sent"));
          msgOut[3] = 0;
          control = 2;
        }
        break;
      case 5:
        if ((client.publish(topicPub, msgOut, msgLength)) == false) Serial.println(F("fault publishing")); {
          sendCntr++;
          if (sendCntr == sendNr) {
            Serial.println(F("Sequence concluded, next decoder"));
            msgOut[2] = 0;
            msgOut[3] = 0;
            sendCntr = 0;
            ackConfirm = false;
            control = 1;
            addressSelected = false;
            msgStraightSent = false;
            msgThrownSent = false;
          }
        }
    }
  }
}
///////////////////////////////////////////////////////////// end of program loop ///////////////////////

/*
   SelectAddress

   function : read decoderId from dipswitches, LED on during reading
              flash three times after selection by push button

   called by: loop

*/
void SelectAddress() {
  if (addressSelected == false) {
    digitalWrite(Led, HIGH);                                        // switch LED on
    if (millis() - addressSelectTimer >= addressSelectDelay) {
      confirmCount = 0;
      addressSelectTimer = millis();
      for (byte index = 0 ; index < 8 ; index++) {
        dip[index] = dipSwitch.getBit(index);                       // read dipswitch
      }
      decoderAddressOld = decoderAddress;
      for (byte index = 0 ; index < 8 ; index++) {
        if (dip[index] == true) decoderAddress |= (1 << index);     // forces nth bit of x to be 1.
        else  decoderAddress &= ~(1 << index);                      // forces nth bit of x to be 0.
      }
      if (debugFlag == true) {
        if (decoderAddressOld != decoderAddress) {
          Serial.println();
          Serial.print(F("Decoder address selected = "));
          Serial.println(decoderAddress);
        }
      }
      butVal = digitalRead(ackButton);                              // read button
      if (butVal == 0) addressSelected = true;
      if (addressSelected == true) {
        msgOut[0] = decoderAddress;
        digitalWrite(Led, LOW);                                     // switch LED off
        control = 9;                                                // divert to flash
        flashCounter = 0;
        stepBack = 2;                                               // restore control loop
        confirmCount = 0;
      }
    }
  }
} // end of selectAddress

/*
   ReadPot

   function : read val potmeter, convert to degrees, publish degrees

   called by: loop

*/
void ReadPot() {
  if (millis() - readPotTimer >= readPotDelay) {
    readPotTimer = millis();
    potValOld = potVal;
    potVal = analogRead(potPin);                                  // reads the value of the potentiometer (value between 0 and 1023)
    potVal = map(potVal, 0, 1000, 0, 180);                        // scale it to use it with the servo (value between 0 and 180)
    control = 2;
    if (potValOld != potVal) {                                    // publish message on change of potmeter reading
      msgOut[1] = potVal;
      msgFlag = 1;
      control = 0;
    }
  }
} // end of ReadPot

/*
   Confirm

   function : read button to confirm servo position and publish messsage

*/
void Confirm() {
  if (millis() - confirmTimer >= confirmDelay) {
    confirmTimer = millis();
    ackConfirm = false;
    butVal = digitalRead(ackButton);                               // read button
    if (butVal == 0) {
      ackConfirm = true;
      if (confirmCount == 0) confirmCount = 1;
    }
    if (ackConfirm == true) {
      switch (confirmCount) {
        case 0:
          control = 2;
        case 1:
          msgOut[2] = 1;
          flashCounter = 0;
          control = 9;                                              // divert to flash
          stepBack = 0;                                             // restore control loop
          msgFlag = 2;
          confirmCount = 2;
          break;
        case 2:
          msgOut[3] = 1;
          flashCounter = 0;
          control = 9;                                              // divert to flash
          stepBack = 0;                                             // restore control loop
          msgFlag = 3;
          confirmCount = 3;
          break;
        case 3:
          msgOut[2] = 1;
          msgOut[3] = 1;
          flashCounter = 0;
          control = 9;                                              // divert to flash
          stepBack = 0;                                             // restore control loop
          msgFlag = 5;
          break;
        default:
          break;
      }
    }
  }
} // end of Confirm

/*
   Flash

   function : flash on tag recognised

   called by: loop

*/

void Flash() {
  if (millis() - flashTimer >= flashDuration) {
    flashTimer = millis();
    if (ledState == LOW) ledState = HIGH;
    else {
      ledState = LOW;
      flashCounter++;
    }
    digitalWrite(Led, ledState);
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
    if (debugFlag == true) {
      Serial.print(F("Message in  ["));
      Serial.print(topic);
      Serial.print(F(" - DEC] "));
      for (byte index = 0; index < msgLength; index++) {
        msgIn[index] = (char)payload[index];
        Serial.print(((char)payload[index]), DEC);
        if (index < msgLength - 1) Serial.print(F("."));
      }
      Serial.println();
    }
  }
} // end of Callback

/*
   re-establish connection with MWTTclientID.
   publish topic topicPub. subscribe to topic topicSub.
   when Mosquitto not available try again after 3 seconds

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
