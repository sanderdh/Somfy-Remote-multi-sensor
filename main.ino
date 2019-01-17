#include <Arduino.h>
#include <vector>

#include <DHT.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

/* 
 *  Code taken from:
 *  https://github.com/cbrherms/ESP-MQTT-JSON-Multisensor/blob/8c87c4a5ac824b71de16bef0f4620344e4516924/bruh_mqtt_multisensor_github/bruh_mqtt_multisensor_github.ino
 *  and
 *  https://github.com/marmotton/Somfy_Remote.git
 *  
 *  Needs:
 *  https://github.com/adafruit/Adafruit_Sensor
 *  https://github.com/adafruit/DHT-sensor-library
 *  
 *  Note: I did not test this code on ESP32 yet.
 *  
 *  Adapted to run on ESP32 from original code at https://github.com/Nickduino/Somfy_Remote

This program allows you to emulate a Somfy RTS or Simu HZ remote.
If you want to learn more about the Somfy RTS protocol, check out https://pushstack.wordpress.com/somfy-rts-protocol/

The rolling code will be stored in non-volatile storage (Preferences), so that you can power the Arduino off.

Serial communication of the original code is replaced by MQTT over WiFi.

Modifications should only be needed in config.h.

For more information see https://projects.dehaan.net/

All the credits go to the original authors! (see links above)
*/

// Configuration of the remotes that will be emulated
struct REMOTE {
    unsigned int id;
    char const* mqtt_topic;
    unsigned int default_rolling_code;
    uint32_t eeprom_address;
};

#include "config.h"

float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 25;

float diffTEMP = 0.2;
float tempValue;

float diffHUM = 1;
float humValue;

int pirValue;
int pirStatus;
String motionStatus;

char message_buff[100];

int calibrationTime = 0;

const int BUFFER_SIZE = 300;

#define MQTT_MAX_PACKET_SIZE 512

DHT dht(DHTPIN, DHTTYPE);

// GPIO macros
#ifdef ESP32
    #define SIG_HIGH GPIO.out_w1ts = 1 << PORT_TX
    #define SIG_LOW  GPIO.out_w1tc = 1 << PORT_TX
#elif ESP8266
    #define SIG_HIGH GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << PORT_TX)
    #define SIG_LOW  GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << PORT_TX)
#endif

// Store the rolling codes in NVS
#ifdef ESP32
    #include <Preferences.h>
    Preferences preferences;
#elif ESP8266
    #include <EEPROM.h>
#endif


// Wifi and MQTT
#ifdef ESP32
    #include <WiFi.h>
#elif ESP8266
    #include <ESP8266WiFi.h>
#endif

#include <PubSubClient.h>

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Buttons
#define SYMBOL 640
#define HAUT 0x2
#define STOP 0x1
#define BAS 0x4
#define PROG 0x8

byte frame[7];

void BuildFrame(byte *frame, byte button, REMOTE remote);
void SendCommand(byte *frame, byte sync);
void receivedCallback(char* topic, byte* payload, unsigned int length);
void mqttconnect();

void setup() {
    // USB serial port
    Serial.begin(115200);

    pinMode(PIRPIN, INPUT);
    pinMode(DHTPIN, INPUT);
    pinMode(LDRPIN, INPUT);

    ArduinoOTA.setPort(OTAport);
    ArduinoOTA.setHostname(mqtt_id);
    ArduinoOTA.setPassword((const char *)OTApassword);

    Serial.print("calibrating sensor ");
    for (int i = 0; i < calibrationTime; i++) {
      Serial.print(".");
      delay(1000);
    }

    ArduinoOTA.onStart([]() {
      Serial.println("Starting");
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
    
    // Output to 433.42MHz transmitter
    pinMode(PORT_TX, OUTPUT);
    SIG_LOW;

    // Connect to WiFi
    Serial.print("Connecting to ");
    Serial.println(wifi_ssid);

    WiFi.begin(wifi_ssid, wifi_password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Configure MQTT
    mqtt.setServer(mqtt_server, mqtt_port);
    mqtt.setCallback(receivedCallback);

    // Open storage for storing the rolling codes
    #ifdef ESP32
        preferences.begin("somfy-remote", false);
    #elif ESP8266
        EEPROM.begin(1024);
    #endif

    // Clear all the stored rolling codes (not used during normal operation). Only ESP32 here (ESP8266 further below).
    #ifdef ESP32
        if ( reset_rolling_codes ) {
                preferences.clear();
        }
    #endif

    // Print out all the configured remotes.
    // Also reset the rolling codes for ESP8266 if needed.
    for ( REMOTE remote : remotes ) {
        Serial.print("Simulated remote number : ");
        Serial.println(remote.id, HEX);
        Serial.print("Current rolling code    : ");
        unsigned int current_code;

        #ifdef ESP32
            current_code = preferences.getUInt( (String(remote.id) + "rolling").c_str(), remote.default_rolling_code);
        #elif ESP8266
            if ( reset_rolling_codes ) {
                EEPROM.put( remote.eeprom_address, remote.default_rolling_code );
                EEPROM.commit();
            }
            EEPROM.get( remote.eeprom_address, current_code );
        #endif

        Serial.println( current_code );
    }
    Serial.println();
}

/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["humidity"] = (String)humValue;
  root["motion"] = (String)motionStatus;
  root["ldr"] = (String)LDR;
  root["temperature"] = (String)tempValue;
  root["heatIndex"] = (String)dht.computeHeatIndex(tempValue, humValue, IsFahrenheit);


  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  mqtt.publish(state_topic, buffer, true);
}

void loop() {

    ArduinoOTA.handle();
      
    // Reconnect MQTT if needed
    if ( !mqtt.connected() ) {
        mqttconnect();
    }

    mqtt.loop();

    float newTempValue = dht.readTemperature(IsFahrenheit);
    float newHumValue = dht.readHumidity();

    //PIR CODE
    pirValue = digitalRead(PIRPIN); //read state of the

    if (pirValue == LOW && pirStatus != 1) {
      motionStatus = "standby";
      sendState();
      pirStatus = 1;
    }

    else if (pirValue == HIGH && pirStatus != 2) {
      motionStatus = "motion detected";
      sendState();
      pirStatus = 2;
    }

    if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
      tempValue = newTempValue;
      sendState();
    }

    if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
      humValue = newHumValue;
      sendState();
    }


    int newLDR = analogRead(LDRPIN);

    if (checkBoundSensor(newLDR, LDR, diffLDR)) {
      LDR = newLDR;
      sendState();
    }
    delay(100);
}

void mqttconnect() {
    // Loop until reconnected
    while ( !mqtt.connected() ) {
        Serial.print("MQTT connecting ...");

        // Connect to MQTT, with retained last will message "offline"
        if (mqtt.connect(mqtt_id, mqtt_user, mqtt_password, status_topic, 1, 1, "offline")) {
            Serial.println("connected");

            // Subscribe to the topic of each remote with QoS 1
            for ( REMOTE remote : remotes ) {
                mqtt.subscribe(remote.mqtt_topic, 1);
                Serial.print("Subscribed to topic: ");
                Serial.println(remote.mqtt_topic);
            }

            // Update status, message is retained
            mqtt.publish(status_topic, "online", true);
        }
        else {
            Serial.print("failed, status code =");
            Serial.print(mqtt.state());
            Serial.println("try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
    char command = *payload; // 1st byte of payload
    bool commandIsValid = false;
    REMOTE currentRemote;

    Serial.print("MQTT message received: ");
    Serial.println(topic);

    Serial.print("Payload: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Command is valid if the payload contains one of the chars below AND the topic corresponds to one of the remotes
    if ( length == 1 && ( command == 'u' || command == 's' || command == 'd' || command == 'p' ) ) {
        for ( REMOTE remote : remotes ) {
            if ( strcmp(remote.mqtt_topic, topic) == 0 ){
                currentRemote = remote;
                commandIsValid = true;
            }
        }
    }

    if ( commandIsValid ) {
        if ( command == 'u' ) {
            Serial.println("Monte"); // Somfy is a French company, after all.
            BuildFrame(frame, HAUT, currentRemote);
        }
        else if ( command == 's' ) {
            Serial.println("Stop");
            BuildFrame(frame, STOP, currentRemote);
        }
        else if ( command == 'd' ) {
            Serial.println("Descend");
            BuildFrame(frame, BAS, currentRemote);
        }
        else if ( command == 'p' ) {
            Serial.println("Prog");
            BuildFrame(frame, PROG, currentRemote);
        }

        Serial.println("");

        SendCommand(frame, 2);
        for ( int i = 0; i<2; i++ ) {
            SendCommand(frame, 7);
        }

        // Send the MQTT ack message
        String ackString = "id: 0x";
        ackString.concat( String(currentRemote.id, HEX) );
        ackString.concat(", cmd: ");
        ackString.concat(command);
        mqtt.publish(ack_topic, ackString.c_str());
    }
}

void BuildFrame(byte *frame, byte button, REMOTE remote) {
    unsigned int code;

    #ifdef ESP32
        code = preferences.getUInt( (String(remote.id) + "rolling").c_str(), remote.default_rolling_code);
    #elif ESP8266
        EEPROM.get( remote.eeprom_address, code );
    #endif

    frame[0] = 0xA7;            // Encryption key. Doesn't matter much
    frame[1] = button << 4;     // Which button did  you press? The 4 LSB will be the checksum
    frame[2] = code >> 8;       // Rolling code (big endian)
    frame[3] = code;            // Rolling code
    frame[4] = remote.id >> 16; // Remote address
    frame[5] = remote.id >>  8; // Remote address
    frame[6] = remote.id;       // Remote address

    Serial.print("Frame         : ");
    for(byte i = 0; i < 7; i++) {
        if(frame[i] >> 4 == 0) { //  Displays leading zero in case the most significant nibble is a 0.
            Serial.print("0");
        }
        Serial.print(frame[i],HEX); Serial.print(" ");
    }

    // Checksum calculation: a XOR of all the nibbles
    byte checksum = 0;
    for(byte i = 0; i < 7; i++) {
        checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
    }
    checksum &= 0b1111; // We keep the last 4 bits only


    // Checksum integration
    frame[1] |= checksum; //  If a XOR of all the nibbles is equal to 0, the blinds will consider the checksum ok.

    Serial.println(""); Serial.print("With checksum : ");
    for(byte i = 0; i < 7; i++) {
        if(frame[i] >> 4 == 0) {
            Serial.print("0");
        }
        Serial.print(frame[i],HEX); Serial.print(" ");
    }


    // Obfuscation: a XOR of all the bytes
    for(byte i = 1; i < 7; i++) {
        frame[i] ^= frame[i-1];
    }

    Serial.println(""); Serial.print("Obfuscated    : ");
    for(byte i = 0; i < 7; i++) {
        if(frame[i] >> 4 == 0) {
            Serial.print("0");
        }
        Serial.print(frame[i],HEX); Serial.print(" ");
    }
    Serial.println("");
    Serial.print("Rolling Code  : ");
    Serial.println(code);

    #ifdef ESP32
        preferences.putUInt( (String(remote.id) + "rolling").c_str(), code + 1); // Increment and store the rolling code
    #elif ESP8266
        EEPROM.put( remote.eeprom_address, code + 1 );
        EEPROM.commit();
    #endif
}

void SendCommand(byte *frame, byte sync) {
    if(sync == 2) { // Only with the first frame.
        //Wake-up pulse & Silence
        SIG_HIGH;
        delayMicroseconds(9415);
        SIG_LOW;
        delayMicroseconds(89565);
    }

    // Hardware sync: two sync for the first frame, seven for the following ones.
    for (int i = 0; i < sync; i++) {
        SIG_HIGH;
        delayMicroseconds(4*SYMBOL);
        SIG_LOW;
        delayMicroseconds(4*SYMBOL);
    }

    // Software sync
    SIG_HIGH;
    delayMicroseconds(4550);
    SIG_LOW;
    delayMicroseconds(SYMBOL);

    //Data: bits are sent one by one, starting with the MSB.
    for(byte i = 0; i < 56; i++) {
        if(((frame[i/8] >> (7 - (i%8))) & 1) == 1) {
            SIG_LOW;
            delayMicroseconds(SYMBOL);
            SIG_HIGH;
            delayMicroseconds(SYMBOL);
        }
        else {
            SIG_HIGH;
            delayMicroseconds(SYMBOL);
            SIG_LOW;
            delayMicroseconds(SYMBOL);
        }
    }

    SIG_LOW;
    delayMicroseconds(30415); // Inter-frame silence
}

/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}
