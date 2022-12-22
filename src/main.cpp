/*
 *
 * vim: sta:et:sw=4:ts=4:sts=4
 * https://arduinojson.org/v6/how-to/use-arduinojson-with-arduinomqttclient/
 * https://github.com/arduino-libraries/ArduinoMqttClient/
 *
 * https://github.com/maxgerhardt/platform-raspberrypi
 * https://github.com/earlephilhower/arduino-pico
 * https://components101.com/development-boards/raspberry-pi-pico-w
 *******************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

//#include <PicoOTA.h>
#include <LittleFS.h>

//#define SECRET_SSID ""
//#define SECRET_PASS ""

#ifndef SECRET_SSID
#include "arduino_secrets.h"
#endif

const int waterAutoOn = 60UL; // in sec. Auto water/ day

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

const int HL_PIN = A0; // PICO pin 31
const int HR_PIN = A1; // PICO pin 32
const int relayPin = 22; // PICO pin 29

const int msgPeriod = 15 * 1000U;
const int wifiPeriod = 300 * 1000U;
int ledPeriod = 2 * 1000UL;

const char* mqttBroker = "test.mosquitto.org";
const int mqtt_port = 1883;  // Sets the server details.
// const char* mqtt_server = "91.121.93.94";
const char* ntpServer = "ntp1.tecnico.ulisboa.pt";
// int        port        = 1883;
const char willTopic[] = "ipfn/picoW/will";
const char inTopic[]   = "ipfn/picoW/in";
const char outTopic[]  = "ipfn/picoW/out";
String willPayload = "oh no!, lost Mqtt ";
bool willRetain = true;
int willQos = 1;
int subscribeQos = 1;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
WiFiClient wClient;
MqttClient mqttClient(wClient);

bool led_state = false;
bool led_blink = false;
int count = 0;

unsigned int sumWater = 0;
unsigned long stopPump = 0, nextWater;

const long gmtOffset_sec     = 0;
const int daylightOffset_sec = 0; // 3600;

#define WIFI_RETRY 20

#define MSG_BUFFER_SIZE 50
int wifiOK = 0;
int wifiRetries = 0 ;
bool ntpOK = false;
bool relayState = false;

unsigned long reboots;
void onMqttMessage(int messageSize);

int setupMqtt(){
    mqttClient.stop();
    delay(20);
    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqttBroker);

    if (!mqttClient.connect(mqttBroker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        return -1;
    }
    else
        Serial.println("You're connected to the MQTT broker!");

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);
    //Serial.println();

    // subscribe to a topic
    // the second parameter sets the QoS of the subscription,
    // the the library supports subscribing at QoS 0, 1, or 2

    mqttClient.subscribe(inTopic, subscribeQos);
    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(inTopic);
    return 0;
}
void reconnect_wifi(unsigned long nH2O) {
    delay(10);
    if (WiFi.status() == WL_CONNECTED && mqttClient.connected() == 1)
        return;
    Serial.print("No link. Wifi "); Serial.print(WiFi.status());
    Serial.print(" MQTT "); Serial.println(mqttClient.connected());
    mqttClient.unsubscribe(inTopic);
    if(wifiRetries++ > 5){
        // End setup(). Next H20 1671582148
        EEPROM.put(0x10, nH2O);
        reboots++;
        EEPROM.put(0x14, reboots);
        if (EEPROM.commit()) {
            Serial.println("EEPROM successfully committed");
        } else {
            Serial.println("ERROR! EEPROM commit failed");
        }
        Serial.println("Rebooting....");
        rp2040.reboot();
    }
    wifiOK = 0;
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Re-coonnecting to ");
    Serial.println(ssid);

    WiFi.disconnect();
    WiFi.begin(ssid, password);

    led_state = false;
    led_blink = true;
    ledPeriod = 500UL;
    for(int i = 0; i < WIFI_RETRY; i++){
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print(".");
            led_state = not led_state;
            digitalWrite(LED_BUILTIN, led_state);
            delay(500);
            //continue;
        }
        else {
            wifiOK = 1;
            led_state = true;
            //led_blink = true;
            ledPeriod = 2000UL;
            Serial.println("");
            Serial.print("WiFi connected, IP address: ");
            Serial.println(WiFi.localIP());
            digitalWrite(LED_BUILTIN, led_state);
            if(setupMqtt() == 0)
                wifiRetries = 0;
            break;
        }
    }
}

void setClockNtp(uint32_t timeout) {
    NTP.begin(ntpServer, "pool.ntp.org");
    NTP.waitSet(timeout);
    time_t now = time(nullptr);
    struct tm timeinfo;
    if(now < 8 * 3600 * 2){
        Serial.println("Could not get ntp time");
    }
    else{
        gmtime_r(&now, &timeinfo);
        Serial.print("Current ntp time: ");
        Serial.print(asctime(&timeinfo));
    }
}

void onMqttMessage(int messageSize) {
    StaticJsonDocument<256> doc;

    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', duplicate = ");
    Serial.print(mqttClient.messageDup() ? "true" : "false");
    Serial.print(", QoS = ");
    Serial.print(mqttClient.messageQoS());
    Serial.print(", retained = ");
    Serial.print(mqttClient.messageRetain() ? "true" : "false");
    Serial.print("', length ");
    Serial.print(messageSize);
    //Serial.print(", msg:");

    deserializeJson(doc, mqttClient);

    //Serial.print("Msg: ");
    //Serial.println((char)mqttClient.read());
    int pump = doc["pump"];
    unsigned long now = millis();
    time_t nowTime = time(nullptr);
    unsigned long ntpTime = nowTime;
    if (pump == 1){
        nextWater = now + 2000UL;
    }
    Serial.print("\"pump\":");
    Serial.println(pump);
    //Serial.println();

    // use the Stream interface to print the contents
    /*
       while (mqttClient.available()) {
       Serial.print((char)mqttClient.read());
       }
       */

}
// The normal, core0 setup
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(relayPin, OUTPUT);

    relayState = false;
    digitalWrite(relayPin, relayState);

    Serial.begin(115200);
    // Set in station mode
    WiFi.mode(WIFI_STA);
    reconnect_wifi(0);

    setClockNtp(10000);

    ArduinoOTA.setHostname("PicoW");  // Set the network port name.  设置网络端口名称
    ArduinoOTA.setPassword("666666");  // Set the network port connection
    ArduinoOTA.begin();            // Initialize the OTA.  初始化OTA
    Serial.println("OTA ready!");  // M5.lcd port output format str§
    nextWater =  millis() + 24UL * 3600UL * 1000UL; // start next day
    EEPROM.begin(256);
    unsigned long val;
    EEPROM.get(0x10, val);
    EEPROM.get(0x14, reboots);
        /*
         * EEPROM.put(0x14, 0);
        //EEPROM.put(0x10, ntpTime);
        if (EEPROM.commit()) {
            Serial.println("EEPROM successfully committed");
        } else {
            Serial.println("ERROR! EEPROM commit failed");
        }
*/
    Serial.print("Reboots:  ");
    Serial.println(reboots);
    //Serial.print("Size:  ");
    //Serial.println(sizeof(unsigned long));
    Serial.print("End setup(). Next H20 ");
    Serial.println(val);
    delay(20);
}

void loop() {
    static  unsigned long lastMsg = 0;
    static  unsigned long lastWifiCheck = 0;
    static  unsigned long lastLed = 0;
    char msg[MSG_BUFFER_SIZE];

    StaticJsonDocument<256> doc;
    struct tm timeinfo;

    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();

    unsigned long now = millis();
    time_t nowTime = time(nullptr);
    unsigned long ntpTime = nowTime;
    gmtime_r(&nowTime, &timeinfo);

    if (now > nextWater){
        nextWater =  now + 24UL * 3600UL * 1000UL; // repeat next day
        stopPump = now + waterAutoOn * 1000UL;
        sumWater += waterAutoOn;
        //led_state = true;
        //digitalWrite(M5_LED, led_state);
        relayState = true;
        digitalWrite(relayPin, relayState);
    }
    if (now > stopPump){
        relayState = false;
        digitalWrite(relayPin, relayState);
        //digitalWrite(M5_LED, led_state);
    }


    if (now - lastWifiCheck > wifiPeriod) {
        lastWifiCheck = now;
        reconnect_wifi(ntpTime);
    }

    if(led_blink)
        if (now - lastLed > ledPeriod) {
            lastLed = now;
            led_state = not led_state;
            digitalWrite(LED_BUILTIN, led_state);
        }

    int rawADC_HR, rawADC_HL;
    if (now - lastMsg > msgPeriod) {
        lastMsg = now;
        
        rawADC_HR = analogRead(HR_PIN);
        rawADC_HL = analogRead(HL_PIN);
        snprintf(msg, MSG_BUFFER_SIZE, "Water ADC HL: %u, ADC HR: %u, %lu, ",
                rawADC_HL, rawADC_HR, ntpTime);
        Serial.print(msg);
        strftime(msg, MSG_BUFFER_SIZE, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        Serial.print(msg);
        Serial.print(", Wifi:"); Serial.print(wifiOK);
        Serial.print(" mqttClient: "); Serial.print(mqttClient.connected());
        Serial.print(" Reboots: "); Serial.print(reboots);
        Serial.print(" relay: "); Serial.println(relayState);

        bool retained = false;
        int qos = 1;
        // bool dup = false;

        float coreTemp = analogReadTemp();

        doc["humidL"] = rawADC_HL;
        doc["humidR"] = rawADC_HR;

        doc["time"] = msg;
        doc["tempCore"] = coreTemp;
        doc["count"]   = nowTime;
        doc["sumWater"] = sumWater;
        doc["reboots"] = reboots;
        //doc["payload"]   = payload;

        mqttClient.beginMessage(outTopic,  (unsigned long) measureJson(doc), retained, qos, false);
        serializeJson(doc, mqttClient);
        mqttClient.endMessage();
    }

}

/*
// Running on core1
void setup1() {
delay(5000);
Serial.printf("C1: Red leader standing by...\n");
}
void loop1() {
static  unsigned long lastMsg = 0;
unsigned long now = millis();
//time_t t_now;
//char buff[80];


if (now - lastMsg > 5000) {
lastMsg = now;

//time(&t_now);
//strftime(buff, sizeof(buff), "%c", localtime(&t_now));

float coreTemp =analogReadTemp();
//Serial.printf("Core temperature %2.1fC ", coreTemp);
//Serial.println(buff);
snprintf(msg, MSG_BUFFER_SIZE, "Core temperature: %2.1fC", coreTemp);

PmqttClient.publish("ipfn/rega/temp", msg);
Serial.println(msg);

}
//delay(500);
}
*/
/*
   void callback(char* topic, byte* payload, unsigned int length) {
   StaticJsonDocument<256> doc;
   deserializeJson(doc, payload, length);
   Serial.print("Message arrived [");
   Serial.print(topic);
   Serial.print("] ");
   for (int i = 0; i < length; i++) {
   Serial.print((char)payload[i]);
   }
   led_state = not led_state;
   Serial.print(F(", LED:"));
   Serial.println(led_state);
   }

   void reConnect() {
   while (!PmqttClient.connected()) {
   Serial.print(F("Attempting MQTT connection..."));
// Create a random client ID.
String clientId = "ipfn/rega-";
clientId += String(random(0xffff), HEX);
// Attempt to connect.
if (PmqttClient.connect(clientId.c_str())) {
Serial.print("\nSuccess\n");
// Once connected, publish an announcement to the topic.
PmqttClient.publish("ipfn/rega/rp", "hello world");
// ... and resubscribe.
PmqttClient.subscribe("ipfn/rega/led");
} else {
Serial.print("failed, rc=");
Serial.print(PmqttClient.state());
Serial.println("try again in 5 seconds");
delay(5000);
}
}
}
*/
