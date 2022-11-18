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
const int mqtt_port = 1883;  // Sets the server details.
// const char* mqtt_server = "91.121.93.94";
const char* ntpServer = "ntp1.tecnico.ulisboa.pt";

WiFiClient wClient;
WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

PubSubClient mqttClient(wClient);

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
    timeClient.begin();
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void reConnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID.
        String clientId = "ipfn/rega-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect.
        if (mqttClient.connect(clientId.c_str())) {
            Serial.print("\nSuccess\n");
            // Once connected, publish an announcement to the topic.
            mqttClient.publish("ipfn/rega/rp", "hello world");
            // ... and resubscribe.
            mqttClient.subscribe("ipfn/rega/rp");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println("try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    // put your setup code here, to run once:
    setup_wifi();
    mqttClient.setServer(mqtt_server, mqtt_port);  // Sets the server details.
    mqttClient.setCallback(callback);  // Sets the message callback function.
    delay(20);
}

void loop() {
    static  unsigned long lastMsg = 0;
    static bool led_state = false;
    unsigned long now = millis();

    if (!mqttClient.connected()) {
        reConnect();
    }
    mqttClient.loop();  // This function is called periodically to allow clients to

    if (now - lastMsg > 1000) {
        lastMsg = now;
        snprintf(msg, MSG_BUFFER_SIZE, "Watering ADC value: %ld",
                now);
        Serial.println(msg);
        mqttClient.publish("ipfn/rega/rp", msg);  // Publishes a message to the specified
        timeClient.update();

        Serial.println(timeClient.getFormattedTime());

        led_state = not led_state;
        digitalWrite(LED_BUILTIN, led_state);
    }

}
