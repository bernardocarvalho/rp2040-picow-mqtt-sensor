/*
 *
 * vim: sta:et:sw=4:ts=4:sts=4
 * https://arduinojson.org/v6/how-to/use-arduinojson-with-arduinomqttclient/
 * https://github.com/arduino-libraries/ArduinoMqttClient/
 *******************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
//#include <time.h>

#include <NTPClient.h>
//#include <PubSubClient.h>

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

#ifndef STASSID
#include "arduino_secrets.h"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

const char* mqtt_broker = "test.mosquitto.org";
const int mqtt_port = 1883;  // Sets the server details.
                             // const char* mqtt_server = "91.121.93.94";
const char* ntpServer = "ntp1.tecnico.ulisboa.pt";
// int        port        = 1883;
const char willTopic[] = "ipfn/will";
const char inTopic[]   = "ipfn/in";
const char outTopic[]  = "ipfn/out";

WiFiClient wClient;
WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

//PubSubClient PmqttClient(wClient);
MqttClient mqttClient(wClient);

#define MSG_BUFFER_SIZE 50
char msg[MSG_BUFFER_SIZE];

bool led_state = false;
int count = 0;

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
        led_state = not led_state;
        digitalWrite(LED_BUILTIN, led_state);
    }
    led_state = false;
    timeClient.begin();
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void onMqttMessage(int messageSize) {
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

    // use the Stream interface to print the contents
    while (mqttClient.available()) {
        Serial.print((char)mqttClient.read());
    }
    Serial.println();

}
// The normal, core0 setup
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    // put your setup code here, to run once:
    setup_wifi();
    //PmqttClient.setServer(mqtt_broker, mqtt_port);  // Sets the server details.
    //PmqttClient.setCallback(callback);  // Sets the message callback function.
    // set a will message, used by the broker when the connection dies unexpectedly
    // you must know the size of the message beforehand, and it must be set before connecting
    String willPayload = "oh no!";
    bool willRetain = true;
    int willQos = 1;

    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqtt_broker);

    if (!mqttClient.connect(mqtt_broker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());

        while (1);
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

    delay(20);
}

void loop() {
    static  unsigned long lastMsg = 0;
    unsigned long now = millis();
    StaticJsonDocument<256> doc;

    //  char buffer[256];
    // serializeJson(doc, buffer);
    //client.publish("outTopic", buffer);

    /*
       if (!PmqttClient.connected()) {
       reConnect();
       }
       PmqttClient.loop();  // This function is called periodically to allow clients to
       */
    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();


    if (now - lastMsg > 2000) {
        lastMsg = now;
        snprintf(msg, MSG_BUFFER_SIZE, "Watering ADC value: %ld",
                now);
        Serial.println(msg);
        //      PmqttClient.publish("ipfn/rega/rp", msg);  // Publishes a message to the specified
        timeClient.update();

        //        Serial.println(timeClient.getFormattedTime());
        String payload;

        payload += "hello world!";
        payload += " ";
        payload += count;

        Serial.print("Sending message to topic: ");
        Serial.println(outTopic);
        Serial.println(payload);

        // send message, the Print interface can be used to set the message contents
        // in this case we know the size ahead of time, so the message payload can be streamed

        bool retained = false;
        int qos = 1;
        bool dup = false;

        float coreTemp =analogReadTemp();

        doc["temp"] = coreTemp;
        doc["count"]   = count++;
        doc["payload"]   = payload;

        //mqttClient.beginMessage(outTopic, payload.length(), retained, qos, dup);
        mqttClient.beginMessage(outTopic,  (unsigned long)measureJson(doc), retained, qos, dup);
        serializeJson(doc, mqttClient);
        //mqttClient.print(payload);
        mqttClient.endMessage();


        digitalWrite(LED_BUILTIN, led_state);
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
