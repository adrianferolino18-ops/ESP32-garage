#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <WebServer.h>

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

// ---------------- WEB SERVER ----------------
WebServer server(80);

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
    int attempts = 0;
    const int maxAttempts = 20; // ~10 sec timeout
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi connection failed. Continuing without WiFi.");
    }
}

// ---------------- MQTT CALLBACK ----------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (int i = 0; i < length; i++) msg += (char)payload[i];

    Serial.print("MQTT ["); Serial.print(topic); Serial.print("] ");
    Serial.println(msg);

    if (String(topic) == "garage/control") {
        // OPEN ONLY IF CAR IS DETECTED
        if (msg == "OPEN" && carDetected) {
            garageOpen = true;
            servoTarget = SERVO_OPEN;
            digitalWrite(LED_RED, HIGH);
            safeNoticeSent = false;
            client.publish("garage/status", "OPEN", true);
        }

        // PREVENT CLOSING IF CAR IS STILL DETECTED
        if (msg == "CLOSE") {
            if (carDetected) {
                client.publish("garage/status", "CANNOT CLOSE - CAR STILL DETECTED", true);
            } else {
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
    if (WiFi.status() != WL_CONNECTED) return; // skip if no WiFi

    if (!client.connected()) {
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
            Serial.println(" will retry in loop");
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

    // Web server
    server.on("/", handleRoot);
    server.on("/open", handleOpen);
    server.on("/close", handleClose);
    server.on("/data", handleData);
    server.begin();
    Serial.println("Web server started");
}

// ---------------- LOOP ----------------
void loop() {
    // MQTT
    reconnectMQTT();
    client.loop();

    // Web server
    server.handleClient();

    // -------- SENSOR MEASUREMENT --------
    if (millis() - lastMeasure >= measureInterval) {
        lastMeasure = millis();

        digitalWrite(TR_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TR_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TR_PIN, LOW);

        long duration = pulseIn(EC_PIN, HIGH, 30000);
        if (duration > 0) {
            float d = duration * 0.017;
            if (d > 2 && d < 400) distance_cm = d;
        }

        // CAR DETECTED
        if (distance_cm <= DETECT_DISTANCE && !carDetected) {
            carDetected = true;
            digitalWrite(LED_RED, HIGH);
            safeNoticeSent = false;
            client.publish("garage/status", "CAR DETECTED", true);
        }

        // CAR LEFT WHILE GARAGE IS OPEN
        if (distance_cm > DETECT_DISTANCE && carDetected && garageOpen && !safeNoticeSent) {
            carDetected = false;
            safeNoticeSent = true;
            client.publish("garage/status", "NO CAR DETECTED - SAFE TO CLOSE", true);
        }

        // CAR LEFT WHILE GARAGE IS CLOSED
        if (distance_cm > DETECT_DISTANCE && carDetected && !garageOpen) {
            carDetected = false;
            digitalWrite(LED_RED, LOW);
            client.publish("garage/status", "CLOSED", true);
        }

        // Publish distance
        client.publish("garage/distance", String(distance_cm).c_str(), true);

        // Debug
        Serial.print("Distance: "); Serial.print(distance_cm);
        Serial.print(" | Detected: "); Serial.print(carDetected);
        Serial.print(" | Garage: "); Serial.println(garageOpen ? "OPEN" : "CLOSED");
    }

    // -------- SERVO SMOOTH MOVE --------
    if (millis() - lastServoStep >=  servoStepInterval) {
        lastServoStep = millis();
        if (servoPos < servoTarget) servoPos++;
        else if (servoPos > servoTarget) servoPos--;
        myservo.write(servoPos);
    }
}

// ---------------- WEB HANDLERS ----------------
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>ESP32 Garage</title>";
    html += "<script>";
    html += "function fetchData(){";
    html += "fetch('/data').then(response => response.json()).then(data => {";
    html += "document.getElementById('distance').innerText = data.distance + ' cm';";
    html += "document.getElementById('garage').innerText = data.garage;";
    html += "});}";
    html += "setInterval(fetchData, 1000);";
    html += "</script></head><body>";
    html += "<h2>ESP32 Garage Status</h2>";
    html += "<p>Distance: <span id='distance'>" + String(distance_cm) + " cm</span></p>";
    html += "<p>Garage: <span id='garage'>" + String(garageOpen ? "OPEN" : "CLOSED") + "</span></p>";
    html += "<button onclick=\"location.href='/open'\">OPEN</button>";
    html += "<button onclick=\"location.href='/close'\">CLOSE</button>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleData() {
    String json = "{";
    json += "\"distance\":" + String(distance_cm) + ",";
    json += "\"garage\":\"" + String(garageOpen ? "OPEN" : "CLOSED") + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleOpen() {
    if (carDetected) {
        garageOpen = true;
        servoTarget = SERVO_OPEN;
        digitalWrite(LED_RED, HIGH);
        safeNoticeSent = false;
        client.publish("garage/status", "OPEN", true);
    }
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleClose() {
    if (!carDetected) {
        garageOpen = false;
        servoTarget = SERVO_CLOSED;
        digitalWrite(LED_RED, LOW);
        safeNoticeSent = false;
    }
    server.sendHeader("Location", "/");
    server.send(303);
}
