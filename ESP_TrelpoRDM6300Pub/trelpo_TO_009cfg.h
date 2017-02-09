/////////////////////////////////////////// user adjustble variables /////////////////////////////////////////
static const char decoderId = 9;                             // also used in IP address decoder (check if IP address is available)
char wiFiHostname[] = "Trelpo-TO-009";                       // Hostname displayed in OTA port
static const char *ssid = "xxxxxx";                          // ssid WiFi network
static const char *password = "password";                    // password WiFi network
static const char *topicPub = "rocnet/rs";                   // rocnet/rs for sensor
static const char *topicSub = "rocnet/rs";                   // rocnet/ot for turnout control
static const char *MQTTclientId = (wiFiHostname);            // MQTT client Id, differs per decoder, to be set by user

IPAddress mosquitto(192, 168, 2, 193);                       // IP address Mosquitto
IPAddress decoder(192, 168, 2, decoderId);                   // IP address decoder
IPAddress gateway(192, 168, 2, 1);                           // IP address gateway
IPAddress subnet(255, 255, 255, 0);                          // subnet masker

static const byte addressSr = 71;                            // sensor address in Rocrail
static const byte scanDelay = 50;                            // scan delay to receive tag number. default 50. larger than 100 useless
static const int tagAmount = 3;                              // amount of tags
static int releaseTime = 3000;                               // time before scanning next tag
static const int flashDuration = 100;                        // flash at tag recognition
static const byte LedPin = D1;       
static boolean blankTag = false;                             // if true unknown tag is scanned and output to serial monitor

  static byte tag[tagAmount][14] = {                         // tag numbers
  {2, 48, 54, 48, 48, 52, 57, 57, 49, 54, 56, 66, 54, 3},    // keyfob for testing
  {2, 48, 68, 48, 48, 52, 48, 54, 50, 70, 53, 68, 65, 3},    // keyfob for testing
  {2, 48, 48, 48, 48, 48, 48, 48, 52, 54, 56, 54, 67, 3}     // glass tagg for testing
  };

static boolean debugFlag = true;                             // support debugging
static boolean caseFlag = false;                             // controls display of control loop

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
