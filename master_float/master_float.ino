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
const unsigned long CLIENT_COMMAND_TIMEOUT_MS = 30000;
const unsigned long MISSION_CLIENT_COMMAND_TIMEOUT_MS = 500;

struct MissionConfig {
  float deepTargetM;
  float shallowTargetM;
  float surfaceTargetM;
  float deepToleranceM;
  float shallowToleranceM;
  float surfaceToleranceM;
  float minSafeDepthM;
  unsigned long holdTimeMs;
  unsigned long sinkPulseMs;
  unsigned long deepNeutralizePulseMs;
  unsigned long risePulseMs;
  unsigned long shallowNeutralizePulseMs;
  unsigned long returnSurfacePulseMs;
  unsigned long thresholdTimeoutMs;
  unsigned long logIntervalMs;
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
  PHASE_SINK_PULSE,
  PHASE_WAIT_DEEP,
  PHASE_DEEP_NEUTRALIZE_PULSE,
  PHASE_HOLD_DEEP,
  PHASE_RISE_PULSE,
  PHASE_WAIT_SHALLOW,
  PHASE_SHALLOW_NEUTRALIZE_PULSE,
  PHASE_HOLD_SHALLOW,
  PHASE_RETURN_SURFACE_PULSE,
  PHASE_WAIT_SURFACE,
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
  30000,
  5000,
  5000,
  5000,
  5000,
  5000,
  180000,
  1000,
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
int manualSingleServo = 0;
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
    case PHASE_SINK_PULSE: return "SINK_PULSE";
    case PHASE_WAIT_DEEP: return "WAIT_DEEP";
    case PHASE_DEEP_NEUTRALIZE_PULSE: return "DEEP_NEUTRALIZE_PULSE";
    case PHASE_HOLD_DEEP: return "HOLD_DEEP";
    case PHASE_RISE_PULSE: return "RISE_PULSE";
    case PHASE_WAIT_SHALLOW: return "WAIT_SHALLOW";
    case PHASE_SHALLOW_NEUTRALIZE_PULSE: return "SHALLOW_NEUTRALIZE_PULSE";
    case PHASE_HOLD_SHALLOW: return "HOLD_SHALLOW";
    case PHASE_RETURN_SURFACE_PULSE: return "RETURN_SURFACE_PULSE";
    case PHASE_WAIT_SURFACE: return "WAIT_SURFACE";
    case PHASE_COMPLETE: return "MISSION_COMPLETE";
  }
  return "UNKNOWN";
}

void setServoAngle(int servoDeg) {
  lastServoDeg = constrain(servoDeg, servoDownDeg, servoUpDeg);
  servo1.write(lastServoDeg);
  servo2.write(lastServoDeg);
}

void commandSink() {
  setServoAngle(servoUpDeg);
  lastControl = servoNeutralDeg - servoUpDeg;
}

void commandRise() {
  setServoAngle(servoDownDeg);
  lastControl = servoNeutralDeg - servoDownDeg;
}

void neutral() {
  lastControl = 0;
  setServoAngle(servoNeutralDeg);
}

void neutralSingleServo(int servoNumber) {
  if (servoNumber == 1) {
    servo1.write(servoNeutralDeg);
  } else if (servoNumber == 2) {
    servo2.write(servoNeutralDeg);
  }
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
  manualSingleServo = 0;
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
  config.holdTimeMs = msValueAfter(command, "hold", config.holdTimeMs);
  config.sinkPulseMs = msValueAfter(command, "sink_pulse", config.sinkPulseMs);
  config.deepNeutralizePulseMs = msValueAfter(command, "deep_neutralize", config.deepNeutralizePulseMs);
  config.risePulseMs = msValueAfter(command, "rise_pulse", config.risePulseMs);
  config.shallowNeutralizePulseMs = msValueAfter(command, "shallow_neutralize", config.shallowNeutralizePulseMs);
  config.returnSurfacePulseMs = msValueAfter(command, "return_surface", config.returnSurfacePulseMs);
  config.thresholdTimeoutMs = msValueAfter(command, "threshold_timeout", config.thresholdTimeoutMs);
  config.thresholdTimeoutMs = msValueAfter(command, "phase_timeout", config.thresholdTimeoutMs);
  config.logIntervalMs = msValueAfter(command, "log", config.logIntervalMs);
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

  switch (nextPhase) {
    case PHASE_SINK_PULSE:
    case PHASE_SHALLOW_NEUTRALIZE_PULSE:
      commandSink();
      break;

    case PHASE_DEEP_NEUTRALIZE_PULSE:
    case PHASE_RISE_PULSE:
    case PHASE_RETURN_SURFACE_PULSE:
      commandRise();
      break;

    case PHASE_WAIT_DEEP:
    case PHASE_WAIT_SHALLOW:
    case PHASE_WAIT_SURFACE:
      neutral();
      break;

    case PHASE_HOLD_DEEP:
    case PHASE_HOLD_SHALLOW:
      holdStartMs = phaseStartMs;
      neutral();
      break;

    case PHASE_COMPLETE:
      missionComplete = true;
      mode = IDLE;
      neutral();
      break;
  }
}

void startMission() {
  clearMissionLog();
  depthRecordComplete = false;
  currentProfile = 1;
  modeStartMs = millis();
  mode = MISSION;
  setMissionPhase(PHASE_SINK_PULSE);
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
  manualSingleServo = 0;
  modeStartMs = millis();
  manualEndMs = modeStartMs + durationMs;
  return true;
}

bool startManualSingleMove(String command) {
  if (mode == MISSION) {
    return false;
  }

  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  int thirdSpace = command.indexOf(' ', secondSpace + 1);
  if (firstSpace < 0 || secondSpace < 0 || thirdSpace < 0) {
    return false;
  }

  int servoNumber = command.substring(firstSpace + 1, secondSpace).toInt();
  String direction = command.substring(secondSpace + 1, thirdSpace);
  direction.toUpperCase();
  float seconds = command.substring(thirdSpace + 1).toFloat();
  unsigned long durationMs = (unsigned long)(seconds * 1000.0);

  if ((servoNumber != 1 && servoNumber != 2) || durationMs == 0 || durationMs > MANUAL_MAX_MS) {
    return false;
  }

  int angle = servoNeutralDeg;
  if (direction == "DOWN") {
    angle = servoDownDeg;
    lastControl = servoNeutralDeg - servoDownDeg;
  } else if (direction == "UP") {
    angle = servoUpDeg;
    lastControl = servoNeutralDeg - servoUpDeg;
  } else if (direction == "NEUTRAL") {
    angle = servoNeutralDeg;
    lastControl = 0;
  } else {
    return false;
  }

  if (servoNumber == 1) {
    servo1.write(angle);
  } else {
    servo2.write(angle);
  }

  lastServoDeg = angle;
  manualSingleServo = servoNumber;
  mode = MANUAL_MOVE;
  modeStartMs = millis();
  manualEndMs = modeStartMs + durationMs;
  return true;
}

void updateManual() {
  if (mode == MANUAL_MOVE && millis() >= manualEndMs) {
    if (manualSingleServo == 0) {
      neutral();
    } else {
      neutralSingleServo(manualSingleServo);
      lastControl = 0;
      lastServoDeg = servoNeutralDeg;
    }
    manualSingleServo = 0;
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

  unsigned long elapsedMs = millis() - phaseStartMs;
  bool thresholdTimedOut = config.thresholdTimeoutMs > 0 && elapsedMs >= config.thresholdTimeoutMs;
  bool returningSurface = missionPhase == PHASE_RETURN_SURFACE_PULSE || missionPhase == PHASE_WAIT_SURFACE;
  if (depth < config.minSafeDepthM && !returningSurface && missionPhase != PHASE_SINK_PULSE && missionPhase != PHASE_COMPLETE) {
    if (thresholdTimedOut) {
      setMissionPhase(PHASE_RETURN_SURFACE_PULSE);
      logPoint(depth, "SURFACE_SAFETY_TIMEOUT");
    } else {
      commandSink();
      logPoint(depth, "SURFACE_SAFETY");
    }
    return;
  }

  switch (missionPhase) {
    case PHASE_SINK_PULSE:
      commandSink();
      if (elapsedMs >= config.sinkPulseMs) {
        setMissionPhase(PHASE_WAIT_DEEP);
      }
      break;

    case PHASE_WAIT_DEEP:
      neutral();
      if (depth >= config.deepTargetM - config.deepToleranceM) {
        setMissionPhase(PHASE_DEEP_NEUTRALIZE_PULSE);
      } else if (thresholdTimedOut) {
        setMissionPhase(PHASE_RETURN_SURFACE_PULSE);
        logPoint(depth, "DEEP_TIMEOUT");
      }
      break;

    case PHASE_DEEP_NEUTRALIZE_PULSE:
      commandRise();
      if (elapsedMs >= config.deepNeutralizePulseMs) {
        setMissionPhase(PHASE_HOLD_DEEP);
      }
      break;

    case PHASE_HOLD_DEEP:
      if (millis() - holdStartMs >= config.holdTimeMs) {
        setMissionPhase(PHASE_RISE_PULSE);
      }
      break;

    case PHASE_RISE_PULSE:
      commandRise();
      if (elapsedMs >= config.risePulseMs) {
        setMissionPhase(PHASE_WAIT_SHALLOW);
      }
      break;

    case PHASE_WAIT_SHALLOW:
      neutral();
      if (depth <= config.shallowTargetM + config.shallowToleranceM) {
        setMissionPhase(PHASE_SHALLOW_NEUTRALIZE_PULSE);
      } else if (thresholdTimedOut) {
        setMissionPhase(PHASE_RETURN_SURFACE_PULSE);
        logPoint(depth, "SHALLOW_TIMEOUT");
      }
      break;

    case PHASE_SHALLOW_NEUTRALIZE_PULSE:
      commandSink();
      if (elapsedMs >= config.shallowNeutralizePulseMs) {
        setMissionPhase(PHASE_HOLD_SHALLOW);
      }
      break;

    case PHASE_HOLD_SHALLOW:
      if (millis() - holdStartMs >= config.holdTimeMs) {
        if (currentProfile >= config.profileCount) {
          setMissionPhase(PHASE_RETURN_SURFACE_PULSE);
        } else {
          currentProfile++;
          setMissionPhase(PHASE_SINK_PULSE);
        }
      }
      break;

    case PHASE_RETURN_SURFACE_PULSE:
      commandRise();
      if (elapsedMs >= config.returnSurfacePulseMs) {
        setMissionPhase(PHASE_WAIT_SURFACE);
      }
      break;

    case PHASE_WAIT_SURFACE:
      neutral();
      if (depth <= config.surfaceTargetM + config.surfaceToleranceM) {
        setMissionPhase(PHASE_COMPLETE);
      } else if (thresholdTimedOut) {
        logPoint(depth, "SURFACE_TIMEOUT");
        setMissionPhase(PHASE_COMPLETE);
      }
      break;

    case PHASE_COMPLETE:
      setMissionPhase(PHASE_COMPLETE);
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
  client.print(" phase=");
  client.print(missionPhaseName());
  client.print(" sensor=");
  client.print(sensorReady ? "1" : "0");
  client.print(" samples=");
  client.print(logIndex);
  client.print(" servo_down=");
  client.print(servoDownDeg);
  client.print(" servo_neutral=");
  client.print(servoNeutralDeg);
  client.print(" servo_up=");
  client.print(servoUpDeg);
  client.print(" sink_pulse_s=");
  client.print(config.sinkPulseMs / 1000.0, 2);
  client.print(" deep_neutralize_s=");
  client.print(config.deepNeutralizePulseMs / 1000.0, 2);
  client.print(" rise_pulse_s=");
  client.print(config.risePulseMs / 1000.0, 2);
  client.print(" shallow_neutralize_s=");
  client.print(config.shallowNeutralizePulseMs / 1000.0, 2);
  client.print(" return_surface_s=");
  client.print(config.returnSurfacePulseMs / 1000.0, 2);
  client.print(" threshold_timeout_s=");
  client.println(config.thresholdTimeoutMs / 1000.0, 2);
  client.flush();
}

void handleCommand(WiFiClient& client, String command) {
  command.trim();
  String upper = command;
  upper.toUpperCase();

  if (mode == MISSION && upper != "STATUS" && upper != "ABORT") {
    client.println("ERROR BUSY_MISSION");
    client.flush();
    return;
  }

  if (upper == "PING") {
    client.println("OK PONG");
  } else if (upper.startsWith("SERVO_CONFIG")) {
    if (applyServoConfig(command)) {
      client.println("OK SERVO_CONFIG");
    } else {
      client.println("ERROR SERVO_CONFIG");
    }
  } else if (upper.startsWith("CONFIG")) {
    applyConfig(command);
    client.println("OK CONFIG");
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
  } else if (upper.startsWith("MANUAL_ONE")) {
    if (startManualSingleMove(upper)) {
      client.println("OK MANUAL_ONE");
    } else {
      client.println("ERROR MANUAL_ONE");
    }
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

  unsigned long timeoutMs = mode == MISSION ? MISSION_CLIENT_COMMAND_TIMEOUT_MS : CLIENT_COMMAND_TIMEOUT_MS;
  String command = readClientLine(client, timeoutMs);
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
  updateManual();
  updateDepthRecord(depth);
  updateMission(depth);
  handleClient();

  delay(LOOP_DELAY_MS);
}
