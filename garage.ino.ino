#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// ---------------- PINS ----------------
#define LED_RED    21
#define TR_PIN     23
#define EC_PIN     22
#define SERVO_PIN  25

// ---------------- SERVO ----------------
#define SERVO_CLOSED 90
#define SERVO_OPEN   180
Servo myservo;
int servoPos    = SERVO_CLOSED;
int servoTarget = SERVO_CLOSED;

// ---------------- WIFI ----------------
const char* ssid     = "asa ani ba";
const char* password = "12345678";

// ---------------- MQTT ----------------
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- SENSOR ----------------
const float DETECT_DISTANCE = 20.0;
float distance_cm = 999;

// ---------------- STATE ----------------
bool carDetected    = false;
bool garageOpen     = false;
bool safeNoticeSent = false;

// ---------------- TIMING ----------------
unsigned long lastMeasure    = 0;
unsigned long lastServoStep  = 0;
const unsigned long measureInterval    = 1000;
const unsigned long servoStepInterval = 30;

// ---------------- WIFI SETUP ----------------
void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());
}

// ---------------- MQTT CALLBACK ----------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.print("MQTT ["); Serial.print(topic); Serial.print("] ");
    Serial.println(msg);

    if (String(topic) == "garage/control") {

        // ---- OPEN ONLY IF CAR IS DETECTED ----
        if (msg == "OPEN" && carDetected) {
            garageOpen = true;
            servoTarget = SERVO_OPEN;
            digitalWrite(LED_RED, HIGH);
            safeNoticeSent = false;
            client.publish("garage/status", "OPEN", true);
        }

        // ---- PREVENT CLOSING IF CAR IS STILL DETECTED ----
        if (msg == "CLOSE") {
            if (carDetected) {
                // 🚫 SAFETY BLOCK
                client.publish("garage/status", "CANNOT CLOSE - CAR STILL DETECTED", true);
            } else {
                // ✅ SAFE TO CLOSE
                garageOpen = false;
                servoTarget = SERVO_CLOSED;
                digitalWrite(LED_RED, LOW);
                safeNoticeSent = false;
                client.publish("garage/status", "CLOSED", true);
            }
        }
    }
}

// ---------------- MQTT RECONNECT ----------------
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        String clientId = "ESP32_GARAGE_" + String(random(0xffff), HEX);

        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            client.subscribe("garage/control");
            client.publish("garage/status", "CLOSED", true);
            client.publish("garage/distance", String(distance_cm).c_str(), true);
        } else {
            Serial.print("failed, rc=");
            Serial.println(client.state());
            delay(2000);
        }
    }
}

// ---------------- SETUP ----------------
void setup() {
    Serial.begin(115200);

    // Pins
    pinMode(TR_PIN, OUTPUT);
    pinMode(EC_PIN, INPUT);
    pinMode(LED_RED, OUTPUT);

    // Servo
    myservo.attach(SERVO_PIN, 500, 2400);
    myservo.write(servoPos);

    // WiFi
    setupWiFi();

    // MQTT
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);
}

// ---------------- LOOP ----------------
void loop() {
    // MQTT
    if (!client.connected()) reconnectMQTT();
    client.loop();

    // -------- SENSOR MEASUREMENT --------
    if (millis() - lastMeasure >= measureInterval) {
        lastMeasure = millis();

        // Trigger sensor
        digitalWrite(TR_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TR_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TR_PIN, LOW);

        // Read echo
        long duration = pulseIn(EC_PIN, HIGH, 30000);
        if (duration > 0) {
            float d = duration * 0.017;
            if (d > 2 && d < 400) distance_cm = d;
        }

        // ---- CAR DETECTED ----
        if (distance_cm <= DETECT_DISTANCE && !carDetected) {
            carDetected = true;
            digitalWrite(LED_RED, HIGH);
            safeNoticeSent = false;
            client.publish("garage/status", "CAR DETECTED", true);
        }

        // ---- CAR LEFT WHILE GARAGE IS OPEN ----
        if (distance_cm > DETECT_DISTANCE && carDetected && garageOpen && !safeNoticeSent) {
            carDetected = false;
            safeNoticeSent = true;
            client.publish("garage/status", "NO CAR DETECTED - SAFE TO CLOSE", true);
        }

        // ---- CAR LEFT WHILE GARAGE IS CLOSED ----
        if (distance_cm > DETECT_DISTANCE && carDetected && !garageOpen) {
            carDetected = false;
            digitalWrite(LED_RED, LOW);
            client.publish("garage/status", "CLOSED", true);
        }

        // Publish distance
        client.publish("garage/distance", String(distance_cm).c_str(), true);

        // Debug print
        Serial.print("Distance: "); Serial.print(distance_cm);
        Serial.print(" | Detected: "); Serial.print(carDetected);
        Serial.print(" | Garage: "); Serial.println(garageOpen ? "OPEN" : "CLOSED");
    }

    // -------- SERVO SMOOTH MOVE --------
    if (millis() - lastServoStep >= servoStepInterval) {
        lastServoStep = millis();
        if (servoPos < servoTarget) servoPos++;
        else if (servoPos > servoTarget) servoPos--;
        myservo.write(servoPos);
    }
}
