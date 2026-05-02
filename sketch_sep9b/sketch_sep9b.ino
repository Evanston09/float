#include <Wire.h>
#include <MS5837.h>
#include <Servo.h>

#if !defined(ARDUINO_ARCH_SAMD)
#error "Wrong board selected. Use Tools > Board > Arduino SAMD Boards > Arduino MKR WiFi 1010."
#endif

#include <WiFiNINA.h>
#include <math.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

const int SERVO1_PIN = 2;
const int SERVO2_PIN = 3;
const int SERVO_UP_DEG = 110;
const int SERVO_NEUTRAL_DEG = 90;
const int SERVO_DOWN_DEG = 70;

const float DEEP_TARGET_M = 2.5;
const float SHALLOW_TARGET_M = 0.40;
const float DEEP_TOLERANCE_M = 0.10;
const float SHALLOW_TOLERANCE_M = 0.05;
const float MIN_SAFE_DEPTH_M = 0.30;

const unsigned long HOLD_TIME_MS = 30000;
const unsigned long LOOP_DELAY_MS = 50;
const unsigned long LOG_INTERVAL_MS = 1000;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;
const int MAX_LOG_POINTS = 1200;

const float KP = 20.0;
const int MAX_CONTROL = SERVO_NEUTRAL_DEG - SERVO_DOWN_DEG;
const int SAFETY_DESCEND_CONTROL = 15;

WiFiServer server(80);
MS5837 sensor;
Servo servo1;
Servo servo2;

enum MissionState {
  WAIT_FOR_START,
  GO_TO_DEEP_1,
  HOLD_DEEP_1,
  GO_TO_SHALLOW_1,
  HOLD_SHALLOW_1,
  GO_TO_DEEP_2,
  HOLD_DEEP_2,
  GO_TO_SHALLOW_2,
  HOLD_SHALLOW_2,
  MISSION_COMPLETE
};

struct DataPoint {
  unsigned long timeMs;
  float depth;
  byte state;
  int control;
};

MissionState state = WAIT_FOR_START;
DataPoint logData[MAX_LOG_POINTS];
int logIndex = 0;

unsigned long missionStartMs = 0;
unsigned long holdStartMs = 0;
unsigned long lastLogMs = 0;
unsigned long lastWifiRetryMs = 0;
int lastControl = 0;

const char* stateName(byte s) {
  switch ((MissionState)s) {
    case WAIT_FOR_START: return "WAIT_FOR_START";
    case GO_TO_DEEP_1: return "GO_TO_DEEP_1";
    case HOLD_DEEP_1: return "HOLD_DEEP_1";
    case GO_TO_SHALLOW_1: return "GO_TO_SHALLOW_1";
    case HOLD_SHALLOW_1: return "HOLD_SHALLOW_1";
    case GO_TO_DEEP_2: return "GO_TO_DEEP_2";
    case HOLD_DEEP_2: return "HOLD_DEEP_2";
    case GO_TO_SHALLOW_2: return "GO_TO_SHALLOW_2";
    case HOLD_SHALLOW_2: return "HOLD_SHALLOW_2";
    case MISSION_COMPLETE: return "MISSION_COMPLETE";
  }
  return "UNKNOWN";
}

float getDepth() {
  sensor.read();
  return sensor.depth();
}

void setBuoyancy(int control) {
  lastControl = constrain(control, -MAX_CONTROL, MAX_CONTROL);
  int servoDeg = constrain(SERVO_NEUTRAL_DEG - lastControl, SERVO_DOWN_DEG, SERVO_UP_DEG);
  servo1.write(servoDeg);
  servo2.write(servoDeg);
}

void holdDepth(float targetDepth, float depth) {
  float error = targetDepth - depth;
  int control = (int)(error * KP);
  setBuoyancy(control);
}

void logSample(float depth) {
  if (state == WAIT_FOR_START || logIndex >= MAX_LOG_POINTS) {
    return;
  }

  unsigned long now = millis();
  if (logIndex > 0 && now - lastLogMs < LOG_INTERVAL_MS) {
    return;
  }

  logData[logIndex].timeMs = now - missionStartMs;
  logData[logIndex].depth = depth;
  logData[logIndex].state = (byte)state;
  logData[logIndex].control = lastControl;
  logIndex++;
  lastLogMs = now;
}

void beginMission() {
  logIndex = 0;
  missionStartMs = millis();
  lastLogMs = 0;
  state = GO_TO_DEEP_1;
}

void ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();
  if (now - lastWifiRetryMs < WIFI_RETRY_INTERVAL_MS) {
    return;
  }

  lastWifiRetryMs = now;
  Serial.println("WiFi disconnected; retrying...");
  WiFi.begin(ssid, password);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi reconnected. IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
  }
}

String readClientLine(WiFiClient& client, unsigned long timeoutMs) {
  String line = "";
  unsigned long start = millis();

  while (client.connected() && millis() - start < timeoutMs) {
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        line.trim();
        return line;
      }
      if (c != '\r') {
        line += c;
      }
    }
    delay(5);
  }

  line.trim();
  return line;
}

void handleStartClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  client.println("FLOAT READY");
  client.println("SEND START TO BEGIN");
  client.flush();

  String command = readClientLine(client, 30000);
  if (command == "START") {
    client.println("START ACK");
    client.flush();
    beginMission();
  } else {
    client.println("NO START RECEIVED");
    client.flush();
  }

  client.stop();
}

void sendLoggedData(WiFiClient& client) {
  client.println("MISSION COMPLETE");
  client.println("time,depth,state,control");

  for (int i = 0; i < logIndex; i++) {
    client.print(logData[i].timeMs / 1000.0, 3);
    client.print(",");
    client.print(logData[i].depth, 3);
    client.print(",");
    client.print(stateName(logData[i].state));
    client.print(",");
    client.println(logData[i].control);
  }

  client.println("END_DATA");
  client.flush();
}

void handleDataClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  client.println("FLOAT DATA READY");
  client.println("SEND GET_DATA TO DOWNLOAD");
  client.flush();

  String command = readClientLine(client, 10000);
  if (command == "GET_DATA") {
    sendLoggedData(client);
  } else {
    client.println("NO DATA COMMAND RECEIVED");
    client.flush();
  }

  client.stop();
}

void updateMission(float depth) {
  if (depth < MIN_SAFE_DEPTH_M && state != MISSION_COMPLETE) {
    setBuoyancy(SAFETY_DESCEND_CONTROL);
    return;
  }

  switch (state) {
    case WAIT_FOR_START:
      setBuoyancy(0);
      break;

    case GO_TO_DEEP_1:
      holdDepth(DEEP_TARGET_M, depth);
      if (fabs(depth - DEEP_TARGET_M) < DEEP_TOLERANCE_M) {
        holdStartMs = millis();
        state = HOLD_DEEP_1;
      }
      break;

    case HOLD_DEEP_1:
      holdDepth(DEEP_TARGET_M, depth);
      if (millis() - holdStartMs >= HOLD_TIME_MS) {
        state = GO_TO_SHALLOW_1;
      }
      break;

    case GO_TO_SHALLOW_1:
      holdDepth(SHALLOW_TARGET_M, depth);
      if (fabs(depth - SHALLOW_TARGET_M) < SHALLOW_TOLERANCE_M) {
        holdStartMs = millis();
        state = HOLD_SHALLOW_1;
      }
      break;

    case HOLD_SHALLOW_1:
      holdDepth(SHALLOW_TARGET_M, depth);
      if (millis() - holdStartMs >= HOLD_TIME_MS) {
        state = GO_TO_DEEP_2;
      }
      break;

    case GO_TO_DEEP_2:
      holdDepth(DEEP_TARGET_M, depth);
      if (fabs(depth - DEEP_TARGET_M) < DEEP_TOLERANCE_M) {
        holdStartMs = millis();
        state = HOLD_DEEP_2;
      }
      break;

    case HOLD_DEEP_2:
      holdDepth(DEEP_TARGET_M, depth);
      if (millis() - holdStartMs >= HOLD_TIME_MS) {
        state = GO_TO_SHALLOW_2;
      }
      break;

    case GO_TO_SHALLOW_2:
      holdDepth(SHALLOW_TARGET_M, depth);
      if (fabs(depth - SHALLOW_TARGET_M) < SHALLOW_TOLERANCE_M) {
        holdStartMs = millis();
        state = HOLD_SHALLOW_2;
      }
      break;

    case HOLD_SHALLOW_2:
      holdDepth(SHALLOW_TARGET_M, depth);
      if (millis() - holdStartMs >= HOLD_TIME_MS) {
        state = MISSION_COMPLETE;
      }
      break;

    case MISSION_COMPLETE:
      holdDepth(SHALLOW_TARGET_M, depth);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!sensor.init()) {
    Serial.println("Sensor init failed!");
    delay(2000);
  }
  sensor.setFluidDensity(997);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  setBuoyancy(0);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  float depth = getDepth();

  if (state == WAIT_FOR_START) {
    ensureWifiConnected();
    handleStartClient();
  } else {
    updateMission(depth);
    logSample(depth);

    if (state == MISSION_COMPLETE) {
      ensureWifiConnected();
      handleDataClient();
    }
  }

  delay(LOOP_DELAY_MS);
}
