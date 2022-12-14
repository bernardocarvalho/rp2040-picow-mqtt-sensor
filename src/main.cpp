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

#ifndef SECRET_SSID
#include "arduino_secrets.h"
#endif

const int waterAutoOn = 60UL; // in sec. Auto water/ day

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

const int HL_PIN = A0; // PICO pin 31
const int HR_PIN = A1; // PICO pin 32
const int relayPin = 22; // PICO pin 292

const int msgPeriod = 5 * 1000U;
const int wifiPeriod = 300 * 1000U;
const int ledPeriod = 2 * 1000U;

const char* mqttBroker = "test.mosquitto.org";
const int mqtt_port = 1883;  // Sets the server details.
// const char* mqtt_server = "91.121.93.94";
const char* ntpServer = "ntp1.tecnico.ulisboa.pt";
// int        port        = 1883;
const char willTopic[] = "ipfn/will";
const char inTopic[]   = "ipfn/picoW/in";
const char outTopic[]  = "ipfn/picoW/out";

WiFiClient wClient;

// By default 'pool.ntp.org' is used with 60 seconds update interval and

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
char msg[MSG_BUFFER_SIZE];
int wifiOK = 0;
bool ntpOK = false;
bool relayState = false;

void reconnect_wifi() {
    delay(10);
    if (WiFi.status() == WL_CONNECTED)
        return;

    wifiOK = 0;
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.disconnect();
    WiFi.begin(ssid, password);

    led_state = false;
    led_blink = false;
    for(int i = 0; i < WIFI_RETRY; i++){
        //while (WiFi.status() != WL_CONNECTED)
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
            led_blink = true;
            //timeClient.begin();
            Serial.println("");
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
            digitalWrite(LED_BUILTIN, led_state);
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
    Serial.println(" bytes:");

    deserializeJson(doc, mqttClient);

    Serial.print(", Pump: ");
    int pump = doc["pump"];
    unsigned long now = millis();
    if (pump == 1){
        nextWater = now + 2000UL;
    }
    Serial.print(pump);

    // use the Stream interface to print the contents
    /*
       while (mqttClient.available()) {
       Serial.print((char)mqttClient.read());
       }
       */
    Serial.println();

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
    reconnect_wifi();

    String willPayload = "oh no!";
    bool willRetain = true;
    int willQos = 1;

    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqttBroker);

    if (!mqttClient.connect(mqttBroker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());

        //        while (1);
    }

    Serial.println("You're connected to the MQTT broker!");

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);
    Serial.println();

    // subscribe to a topic
    // the second parameter sets the QoS of the subscription,
    // the the library supports subscribing at QoS 0, 1, or 2
    int subscribeQos = 1;

    mqttClient.subscribe(inTopic, subscribeQos);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(inTopic);
    setClockNtp(10000);


    Serial.println("End setup()");
    ArduinoOTA.setHostname("PicoW");  // Set the network port name.  设置网络端口名称
    ArduinoOTA.setPassword("666666");  // Set the network port connection
    ArduinoOTA.begin();            // Initialize the OTA.  初始化OTA
    Serial.println("OTA ready!");  // M5.lcd port output format str§
    delay(20);
}

void loop() {
    static  unsigned long lastMsg = 0;
    static  unsigned long lastWifiCheck = 0;
    static  unsigned long lastLed = 0;

    StaticJsonDocument<256> doc;
    struct tm timeinfo;

    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();

    unsigned long now = millis();
    time_t nowTime = time(nullptr);

    if (now > nextWater){
        nextWater =  now + 24UL * 3600UL * 1000UL; // repeat next day
        stopPump = now + waterAutoOn * 1000UL;
        sumWater += waterAutoOn;
        //led_state = true;
        //digitalWrite(M5_LED, led_state);
        digitalWrite(relayPin, HIGH);
    }
    if (now > stopPump){
        digitalWrite(relayPin, LOW);
        //led_state = false;
        //digitalWrite(M5_LED, led_state);
    }


    if (now - lastWifiCheck > wifiPeriod) {
        lastWifiCheck = now;
        reconnect_wifi();
    }

    if(led_blink)
        if (now - lastLed > ledPeriod) {
            lastLed = now;
            led_state = not led_state;
            digitalWrite(LED_BUILTIN, led_state);
        }

    if (now - lastMsg > msgPeriod) {
        lastMsg = now;
        snprintf(msg, MSG_BUFFER_SIZE, "Water ADC: %ld, %lu",
                now, nowTime);
        Serial.print(msg);
        Serial.print(" Wifi:");
        Serial.println(wifiOK);

        bool retained = false;
        int qos = 1;
        bool dup = false;

        float coreTemp =analogReadTemp();

        doc["humidL"] = analogRead(HL_PIN);
        doc["humidR"] = analogRead(HR_PIN);

        gmtime_r(&nowTime, &timeinfo);
        strftime(msg, MSG_BUFFER_SIZE, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        doc["time"] = msg;
        doc["temp"] = coreTemp;
        doc["count"]   = nowTime;
        //doc["payload"]   = payload;

        mqttClient.beginMessage(outTopic,  (unsigned long)measureJson(doc), retained, qos, dup);
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
