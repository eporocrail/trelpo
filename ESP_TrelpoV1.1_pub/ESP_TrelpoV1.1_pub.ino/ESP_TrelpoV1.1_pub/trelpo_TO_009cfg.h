/////////////////////////////////////////// user adjustble variables /////////////////////////////////////////
static const char decoderId = 9;                                    // also used in IP address decoder (check if IP address is available)
static byte hardwareId = 0;                                         // 0: sensor 1: four turnouts 2: three turnouts with current detetction
char wiFiHostname[] = "Trelpo-TO-009";                              // Hostname displayed in OTA port
static const char *ssid = "EPO";                                    // ssid WiFi network
static const char *password = "!1PkwdrT8";                         // password WiFi network
static const char *topicPub1 = "rocnet/rs";                          // rocnet/rs for sensor
static const char *topicPub2 = "rocnet/cf";                          // rocnet/rs for sensor
static const char *topicSub1 = "rocnet/ot";                          // rocnet/ot for turnout control
static const char *topicSub2 = "rocnet/cf";                          // rocnet/cf for turnout adjustment
static const char *MQTTclientId = (wiFiHostname);                   // MQTT client Id, differs per decoder, to be set by user

IPAddress mosquitto(192, 168, 2, 193);                              // IP address Mosquitto
IPAddress decoder(192, 168, 2, decoderId);                          // IP address decoder
IPAddress gateway(192, 168, 2, 1);                                  // IP address gateway
IPAddress subnet(255, 255, 255, 0);                                 // subnet masker

// hardwareId = 0
static const byte addressSr[8] = {71, 72, 73, 74, 75, 76, 77, 78}; // sensor addresses for sensor decoder,


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
*/

// hardwareId = 2
//static const byte addressSr[3] = {71, 72, 73};                    // sensor addresses for three turnouts with current detection

static const byte addressTu[3] = {3, 7, 15};                     // turnout addresses for three turnouys with current detection
static byte servoDelay[3] = {40, 40, 40};                         // controls speed servo movement, higher is slower
static byte servoPos[3][3] = {                                    // pos straight, pos thrown, inverted
  {10, 90, 0},
  {90, 160, 0},
  {75, 100, 0},
};

static boolean debugFlag = true;                                   // set on and off in setup
static boolean caseFlag = false;                                    // controls display of control loop

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
