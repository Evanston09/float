#include <Wire.h>
#include <MS5837.h>
#include <Servo.h>

#if !defined(ARDUINO_ARCH_SAMD)
#error "Wrong board selected. Use Tools > Board > Arduino SAMD Boards > Arduino MKR WiFi 1010."
#endif

#include <WiFiNINA.h>
#include <math.h>

const char* ssid = "t480";
const char* password = "henryisachud";

const int SERVO1_PIN = 2;
const int SERVO2_PIN = 3;
const int WIFI_LED_PIN = LED_BUILTIN;
const int DEFAULT_SERVO_UP_DEG = 150;
const int DEFAULT_SERVO_NEUTRAL_DEG = 90;
const int DEFAULT_SERVO_DOWN_DEG = 30;

const int MAX_LOG_POINTS = 600;
const int SENSOR_INIT_MAX_ATTEMPTS = 10;
const unsigned long LOOP_DELAY_MS = 50;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;
const unsigned long MANUAL_MAX_MS = 10000;

struct MissionConfig {
  float deepTargetM;
  float shallowTargetM;
  float surfaceTargetM;
  float deepToleranceM;
  float shallowToleranceM;
  float surfaceToleranceM;
  float minSafeDepthM;
  float kp;
  unsigned long holdTimeMs;
  unsigned long logIntervalMs;
  unsigned long maxPhaseTimeMs;
  int profileCount;
};

struct DataPoint {
  unsigned long timeMs;
  float depth;
  const char* state;
  int control;
  int servoDeg;
};

enum Mode {
  IDLE,
  MANUAL_MOVE,
  DEPTH_RECORD,
  MISSION
};

enum MissionPhase {
  PHASE_GO_DEEP,
  PHASE_HOLD_DEEP,
  PHASE_GO_SHALLOW,
  PHASE_HOLD_SHALLOW,
  PHASE_RETURN_SURFACE,
  PHASE_COMPLETE
};

MissionConfig config = {
  2.5,
  0.40,
  0.05,
  0.10,
  0.05,
  0.05,
  0.30,
  20.0,
  30000,
  1000,
  180000,
  2
};

WiFiServer server(80);
MS5837 sensor;
Servo servo1;
Servo servo2;

Mode mode = IDLE;
MissionPhase missionPhase = PHASE_COMPLETE;

DataPoint logData[MAX_LOG_POINTS];
int logIndex = 0;

float depthOffsetM = 0.0;
unsigned long modeStartMs = 0;
unsigned long phaseStartMs = 0;
unsigned long holdStartMs = 0;
unsigned long lastLogMs = 0;
unsigned long manualEndMs = 0;
unsigned long lastWifiRetryMs = 0;
int currentProfile = 1;
int lastControl = 0;
int servoUpDeg = DEFAULT_SERVO_UP_DEG;
int servoNeutralDeg = DEFAULT_SERVO_NEUTRAL_DEG;
int servoDownDeg = DEFAULT_SERVO_DOWN_DEG;
int lastServoDeg = DEFAULT_SERVO_NEUTRAL_DEG;
bool missionComplete = false;
bool depthRecordComplete = false;
bool sensorReady = false;

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

float rawDepth() {
  if (!sensorReady) {
    return 0.0;
  }
  sensor.read();
  return sensor.depth();
}

float depthMeters() {
  return rawDepth() - depthOffsetM;
}

const char* modeName() {
  switch (mode) {
    case IDLE: return "IDLE";
    case MANUAL_MOVE: return "MANUAL_MOVE";
    case DEPTH_RECORD: return "DEPTH_RECORD";
    case MISSION: return "MISSION";
  }
  return "UNKNOWN";
}

const char* missionPhaseName() {
  switch (missionPhase) {
    case PHASE_GO_DEEP: return "GO_DEEP";
    case PHASE_HOLD_DEEP: return "HOLD_DEEP";
    case PHASE_GO_SHALLOW: return "GO_SHALLOW";
    case PHASE_HOLD_SHALLOW: return "HOLD_SHALLOW";
    case PHASE_RETURN_SURFACE: return "RETURN_SURFACE";
    case PHASE_COMPLETE: return "MISSION_COMPLETE";
  }
  return "UNKNOWN";
}

void setServoAngle(int servoDeg) {
  lastServoDeg = constrain(servoDeg, servoDownDeg, servoUpDeg);
  servo1.write(lastServoDeg);
  servo2.write(lastServoDeg);
}

void setControl(int control) {
  int maxControl = servoNeutralDeg - servoDownDeg;
  lastControl = constrain(control, -maxControl, maxControl);
  setServoAngle(servoNeutralDeg - lastControl);
}

void holdDepth(float targetDepth, float depth) {
  int control = (int)((targetDepth - depth) * config.kp);
  setControl(control);
}

void neutral() {
  lastControl = 0;
  setServoAngle(servoNeutralDeg);
}

void logPoint(float depth, const char* stateName) {
  if (logIndex >= MAX_LOG_POINTS) {
    return;
  }

  unsigned long now = millis();
  if (logIndex > 0 && now - lastLogMs < config.logIntervalMs) {
    return;
  }

  logData[logIndex].timeMs = now - modeStartMs;
  logData[logIndex].depth = depth;
  logData[logIndex].state = stateName;
  logData[logIndex].control = lastControl;
  logData[logIndex].servoDeg = lastServoDeg;
  logIndex++;
  lastLogMs = now;
}

void clearLog() {
  logIndex = 0;
  lastLogMs = 0;
}

void clearMissionLog() {
  clearLog();
  missionComplete = false;
}

void clearDepthLog() {
  clearLog();
  depthRecordComplete = false;
}

void abortToIdle() {
  neutral();
  mode = IDLE;
  missionPhase = PHASE_COMPLETE;
}

void ensureWifiConnected() {
  if (mode == MISSION) {
    digitalWrite(WIFI_LED_PIN, WiFi.status() == WL_CONNECTED ? HIGH : LOW);
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(WIFI_LED_PIN, HIGH);
    return;
  }

  digitalWrite(WIFI_LED_PIN, LOW);

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
    digitalWrite(WIFI_LED_PIN, HIGH);
    server.begin();
  }
}

float valueAfter(String command, String key, float fallback) {
  int start = command.indexOf(key + "=");
  if (start < 0) {
    return fallback;
  }

  start += key.length() + 1;
  int end = command.indexOf(' ', start);
  if (end < 0) {
    end = command.length();
  }
  return command.substring(start, end).toFloat();
}

int intValueAfter(String command, String key, int fallback) {
  return (int)valueAfter(command, key, fallback);
}

unsigned long msValueAfter(String command, String key, unsigned long fallback) {
  float seconds = valueAfter(command, key, fallback / 1000.0);
  return (unsigned long)(seconds * 1000.0);
}

void applyConfig(String command) {
  config.deepTargetM = valueAfter(command, "deep", config.deepTargetM);
  config.shallowTargetM = valueAfter(command, "shallow", config.shallowTargetM);
  config.surfaceTargetM = valueAfter(command, "surface", config.surfaceTargetM);
  config.deepToleranceM = valueAfter(command, "deep_tol", config.deepToleranceM);
  config.shallowToleranceM = valueAfter(command, "shallow_tol", config.shallowToleranceM);
  config.surfaceToleranceM = valueAfter(command, "surface_tol", config.surfaceToleranceM);
  config.minSafeDepthM = valueAfter(command, "min_safe", config.minSafeDepthM);
  config.kp = valueAfter(command, "kp", config.kp);
  config.holdTimeMs = msValueAfter(command, "hold", config.holdTimeMs);
  config.logIntervalMs = msValueAfter(command, "log", config.logIntervalMs);
  config.maxPhaseTimeMs = msValueAfter(command, "phase_timeout", config.maxPhaseTimeMs);
  config.profileCount = intValueAfter(command, "profiles", config.profileCount);

  if (config.profileCount < 1) {
    config.profileCount = 1;
  }
  if (config.logIntervalMs < 100) {
    config.logIntervalMs = 100;
  }
}

bool applyServoConfig(String command) {
  int up = intValueAfter(command, "up", servoUpDeg);
  int neutralDeg = intValueAfter(command, "neutral", servoNeutralDeg);
  int down = intValueAfter(command, "down", servoDownDeg);

  if (down < 0 || up > 180 || down >= neutralDeg || neutralDeg >= up) {
    return false;
  }

  servoUpDeg = up;
  servoNeutralDeg = neutralDeg;
  servoDownDeg = down;
  neutral();
  return true;
}

void setMissionPhase(MissionPhase nextPhase) {
  missionPhase = nextPhase;
  phaseStartMs = millis();

  if (nextPhase == PHASE_HOLD_DEEP || nextPhase == PHASE_HOLD_SHALLOW) {
    holdStartMs = phaseStartMs;
  }
}

void startMission() {
  clearMissionLog();
  mode = MISSION;
  setMissionPhase(PHASE_GO_DEEP);
  currentProfile = 1;
  modeStartMs = millis();
  neutral();
}

void startDepthRecord() {
  clearDepthLog();
  mode = DEPTH_RECORD;
  modeStartMs = millis();
  neutral();
}

void stopDepthRecord() {
  if (mode == DEPTH_RECORD) {
    neutral();
    mode = IDLE;
  }
  depthRecordComplete = true;
}

bool startManualMove(String command) {
  if (mode == MISSION) {
    return false;
  }

  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  if (firstSpace < 0 || secondSpace < 0) {
    return false;
  }

  String direction = command.substring(firstSpace + 1, secondSpace);
  direction.toUpperCase();
  float seconds = command.substring(secondSpace + 1).toFloat();
  unsigned long durationMs = (unsigned long)(seconds * 1000.0);

  if (durationMs == 0 || durationMs > MANUAL_MAX_MS) {
    return false;
  }

  if (direction == "DOWN") {
    setServoAngle(servoDownDeg);
    lastControl = servoNeutralDeg - servoDownDeg;
  } else if (direction == "UP") {
    setServoAngle(servoUpDeg);
    lastControl = servoNeutralDeg - servoUpDeg;
  } else {
    return false;
  }

  mode = MANUAL_MOVE;
  modeStartMs = millis();
  manualEndMs = modeStartMs + durationMs;
  return true;
}

void updateManual() {
  if (mode == MANUAL_MOVE && millis() >= manualEndMs) {
    neutral();
    mode = IDLE;
  }
}

void updateDepthRecord(float depth) {
  if (mode != DEPTH_RECORD) {
    return;
  }

  logPoint(depth, "DEPTH_RECORD");
}

void updateMission(float depth) {
  if (mode != MISSION) {
    return;
  }

  if (depth < config.minSafeDepthM && missionPhase != PHASE_RETURN_SURFACE && missionPhase != PHASE_COMPLETE) {
    setControl(servoNeutralDeg - servoDownDeg);
    logPoint(depth, "SURFACE_SAFETY");
    return;
  }

  if (config.maxPhaseTimeMs > 0 && millis() - phaseStartMs >= config.maxPhaseTimeMs) {
    if (missionPhase == PHASE_RETURN_SURFACE) {
      logPoint(depth, "RETURN_TIMEOUT");
      missionPhase = PHASE_COMPLETE;
      missionComplete = true;
      mode = IDLE;
      neutral();
      return;
    }

    setMissionPhase(PHASE_RETURN_SURFACE);
    logPoint(depth, "PHASE_TIMEOUT");
    return;
  }

  switch (missionPhase) {
    case PHASE_GO_DEEP:
      holdDepth(config.deepTargetM, depth);
      if (fabs(depth - config.deepTargetM) <= config.deepToleranceM) {
        setMissionPhase(PHASE_HOLD_DEEP);
      }
      break;

    case PHASE_HOLD_DEEP:
      holdDepth(config.deepTargetM, depth);
      if (millis() - holdStartMs >= config.holdTimeMs) {
        setMissionPhase(PHASE_GO_SHALLOW);
      }
      break;

    case PHASE_GO_SHALLOW:
      holdDepth(config.shallowTargetM, depth);
      if (fabs(depth - config.shallowTargetM) <= config.shallowToleranceM) {
        setMissionPhase(PHASE_HOLD_SHALLOW);
      }
      break;

    case PHASE_HOLD_SHALLOW:
      holdDepth(config.shallowTargetM, depth);
      if (millis() - holdStartMs >= config.holdTimeMs) {
        if (currentProfile >= config.profileCount) {
          setMissionPhase(PHASE_RETURN_SURFACE);
        } else {
          currentProfile++;
          setMissionPhase(PHASE_GO_DEEP);
        }
      }
      break;

    case PHASE_RETURN_SURFACE:
      holdDepth(config.surfaceTargetM, depth);
      if (fabs(depth - config.surfaceTargetM) <= config.surfaceToleranceM) {
        missionPhase = PHASE_COMPLETE;
        missionComplete = true;
        mode = IDLE;
        neutral();
      }
      break;

    case PHASE_COMPLETE:
      neutral();
      mode = IDLE;
      break;
  }

  logPoint(depth, missionPhaseName());
}

void sendLog(WiFiClient& client, const char* label) {
  client.println(label);
  client.println("time,depth,state,control,servo");

  for (int i = 0; i < logIndex; i++) {
    client.print(logData[i].timeMs / 1000.0, 3);
    client.print(",");
    client.print(logData[i].depth, 3);
    client.print(",");
    client.print(logData[i].state);
    client.print(",");
    client.print(logData[i].control);
    client.print(",");
    client.println(logData[i].servoDeg);
  }

  client.println("END_DATA");
  client.flush();
}

void sendStatus(WiFiClient& client) {
  client.print("OK STATUS mode=");
  client.print(modeName());
  client.print(" mission_complete=");
  client.print(missionComplete ? "1" : "0");
  client.print(" sensor=");
  client.print(sensorReady ? "1" : "0");
  client.print(" samples=");
  client.print(logIndex);
  client.print(" servo_down=");
  client.print(servoDownDeg);
  client.print(" servo_neutral=");
  client.print(servoNeutralDeg);
  client.print(" servo_up=");
  client.println(servoUpDeg);
  client.flush();
}

void handleCommand(WiFiClient& client, String command) {
  command.trim();
  String upper = command;
  upper.toUpperCase();

  if (upper == "PING") {
    client.println("OK PONG");
  } else if (upper.startsWith("CONFIG")) {
    applyConfig(command);
    client.println("OK CONFIG");
  } else if (upper.startsWith("SERVO_CONFIG")) {
    if (applyServoConfig(command)) {
      client.println("OK SERVO_CONFIG");
    } else {
      client.println("ERROR SERVO_CONFIG");
    }
  } else if (upper == "ZERO_DEPTH") {
    if (!sensorReady) {
      client.println("ERROR NO_SENSOR");
      client.flush();
      return;
    }
    depthOffsetM = rawDepth();
    client.println("OK ZERO_DEPTH");
  } else if (upper == "START_DEPTH_RECORD") {
    if (!sensorReady) {
      client.println("ERROR NO_SENSOR");
      client.flush();
      return;
    }
    startDepthRecord();
    client.println("OK START_DEPTH_RECORD");
  } else if (upper == "STOP_DEPTH_RECORD") {
    stopDepthRecord();
    client.println("OK STOP_DEPTH_RECORD");
  } else if (upper == "GET_DEPTH_DATA") {
    sendLog(client, "DEPTH_DATA");
    return;
  } else if (upper.startsWith("MANUAL")) {
    if (startManualMove(upper)) {
      client.println("OK MANUAL");
    } else {
      client.println("ERROR MANUAL");
    }
  } else if (upper == "NEUTRAL") {
    neutral();
    mode = IDLE;
    client.println("OK NEUTRAL");
  } else if (upper == "ABORT") {
    abortToIdle();
    client.println("OK ABORT");
  } else if (upper == "START_MISSION") {
    if (!sensorReady) {
      client.println("ERROR NO_SENSOR");
      client.flush();
      return;
    }
    startMission();
    client.println("OK START_MISSION");
  } else if (upper == "GET_MISSION_DATA") {
    sendLog(client, "MISSION_DATA");
    return;
  } else if (upper == "STATUS") {
    sendStatus(client);
    return;
  } else {
    client.println("ERROR UNKNOWN_COMMAND");
  }

  client.flush();
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  client.println("FLOAT READY");
  client.flush();

  String command = readClientLine(client, 30000);
  if (command.length() == 0) {
    client.println("ERROR NO_COMMAND");
    client.flush();
  } else {
    handleCommand(client, command);
  }

  client.stop();
}

void setup() {
  Serial.begin(115200);
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);
  Wire.begin();

  for (int attempt = 1; attempt <= SENSOR_INIT_MAX_ATTEMPTS; attempt++) {
    if (sensor.init()) {
      sensorReady = true;
      break;
    }
    Serial.println("Sensor init failed!");
    delay(2000);
  }

  if (sensorReady) {
    sensor.setFluidDensity(997);
    Serial.println("Sensor ready.");
  } else {
    Serial.println("Sensor unavailable after 10 attempts; continuing without depth modes.");
  }

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  neutral();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  digitalWrite(WIFI_LED_PIN, HIGH);

  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Master float firmware ready.");

  server.begin();
}

void loop() {
  float depth = sensorReady ? depthMeters() : 0.0;

  ensureWifiConnected();
  handleClient();
  updateManual();
  updateDepthRecord(depth);
  updateMission(depth);

  delay(LOOP_DELAY_MS);
}
