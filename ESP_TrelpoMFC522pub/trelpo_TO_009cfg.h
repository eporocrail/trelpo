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
