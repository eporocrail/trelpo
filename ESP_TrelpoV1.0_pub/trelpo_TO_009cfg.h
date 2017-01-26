/////////////////////////////////////////// user adjustble variables /////////////////////////////////////////
static const char decoderId = 9;                                    // also used in IP address decoder (check if IP address is available)
static byte hardwareId = 0;                                         // 0: sensor 1: turnout
char wiFiHostname[] = "Trelpo_TO_009";                              // Hostname displayed in OTA port
static const char *ssid = "xxxx";                                   // ssid WiFi network
static const char *password = "pwrd";                               // password WiFi network
static const char *topicPub = "rocnet/rs";                          // rocnet/rs for sensor
static const char *topicSub = "rocnet/ot";                          // rocnet/ot for turnout control
static const char *MQTTclientId = (wiFiHostname);                   // MQTT client Id, differs per decoder, to be set by user

IPAddress mosquitto(192, 168, 2, 193);                              // IP address Mosquitto
IPAddress decoder(192, 168, 2, decoderId);                          // IP address decoder
IPAddress gateway(192, 168, 2, 1);                                  // IP address gateway
IPAddress subnet(255, 255, 255, 0);                                 // subnet masker

static const byte sensorNr = 8;                                     // amount of sensor attached
static const byte sensor[sensorNr] = {D0, D1, D2, D3, D4, D5, D6, D7};     // sensor pins with each a pull-up resistor
static const byte addressSr[sensorNr] = {71, 72, 73, 74, 75, 76, 77, 78};  // sensor addresses in Rocrail

static const byte turnoutNr = 4;                                    // amount of turnout attached
static const byte servoPin[turnoutNr] = {D4, D5, D6, D7};           // servo pins
static const byte relais[turnoutNr] =  {D0, D1, D2, D3};            // relais pins
static const byte addressTu[turnoutNr] = {57, 58, 59, 60};          // turnout addresses in Rocrail

static const byte debounce = 20;                                    // delay in sensor scan processing 

static byte servoDelay[turnoutNr] = {40, 40, 40, 40};               // controls speed servo movement, higher is slower
static byte servoPos[turnoutNr][2] = {                              // pos straight, pos thrown
  {10, 90},
  {90, 160},
  {75, 100},
  {45, 141}
};
static boolean debugFlag = false;                                   // set on and off in setup
static boolean caseFlag = false;                                    // controls display of control loop

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
