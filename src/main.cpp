/*
 *
 * vim: sta:et:sw=4:ts=4:sts=4
 *******************************************************************************/

#include <Arduino.h>
#include <WiFi.h>

#include <NTPClient.h>
#include <PubSubClient.h>

#ifndef STASSID
// #define STASSID "TP-Link_D99E"
// #define STAPSK "06189136"
#define STASSID "Cabovisao-E30F"
#define STAPSK "e0cec31ae30f"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

const char* mqtt_server = "test.mosquitto.org";
//const char* mqtt_server = "91.121.93.94";
const char* ntpServer = "ntp1.tecnico.ulisboa.pt";

#define MSG_BUFFER_SIZE 50
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    //timeClient.begin();
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    // put your setup code here, to run once:
    //setup_wifi();
    delay(20);
}

void loop() {
    static  unsigned long lastMsg = 0;
    static bool led_state = false;
    unsigned long now = millis();

    if (now - lastMsg > 1000) {
        lastMsg = now;
        snprintf(msg, MSG_BUFFER_SIZE, "Watering ADC value: %ld",
                now);
        Serial.println(msg);
        led_state = not led_state;
        digitalWrite(LED_BUILTIN, led_state);
    }

}
