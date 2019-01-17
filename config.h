// You can add as many remote control emulators as you want by adding elements to the "remotes" vector
// The id and mqtt_topic can have any value but must be unique
// default_rolling_code can be any unsigned int, usually leave it at 1
// eeprom_address must be incremented by 4 for each remote

// Once the programe is uploaded on the ESP32:
// - Long-press the program button of YOUR ACTUAL REMOTE until your blind goes up and down slightly
// - send 'p' using MQTT on the corresponding topic
// - You can use the same remote control emulator for multiple blinds, just repeat these steps.
//
// Then:
// - u will make it go up
// - s make it stop
// - d will make it go down


//                                 id            mqtt_topic     default_rolling_code     eeprom_address
std::vector<REMOTE> const remotes = {{0x184623, "home/attic-back/blinds-left", 1,                0 }
                                    ,{0x971547, "home/attic-back/blinds-right",1,                4 }
                                    };

// Change reset_rolling_codes to true to clear the rolling codes stored in the non-volatile storage
// The default_rolling_code will be used

const bool reset_rolling_codes = false;

#define OTApassword "" // change this to whatever password you want to use when you upload OTA
int OTAport = 8266;

#define PIRPIN    D5
#define DHTPIN    D7
#define DHTTYPE   DHT22
#define LDRPIN    A0

#define IsFahrenheit false //to use celsius change to false

const char*        wifi_ssid = "<your ssid>";
const char*    wifi_password = "<your password>";

const char*      mqtt_server = "<mqtt server ip>";
const unsigned int mqtt_port = 1883;
const char*        mqtt_user = "";
const char*    mqtt_password = "";
const char*          mqtt_id = "<mqtt id>";

const char*      state_topic = "home/somfy-remote-attic-back/state";
const char*     status_topic = "home/somfy-remote-attic-back/status"; // Online / offline
const char*        ack_topic = "home/somfy-remote-attic-back/ack"; // Commands ack "id: 0x184623, cmd: u"

#define PORT_TX D2 // Output data on pin D2 (can range from 0 to 31). Check pin numbering on ESP8266.
