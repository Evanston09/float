#if !defined(ARDUINO_ARCH_SAMD)
#error "Wrong board selected. Use Tools > Board > Arduino SAMD Boards > Arduino MKR WiFi 1010."
#endif

#include <Servo.h>
#include <WiFiNINA.h>

const char* ssid = "t480";
const char* password = "henryisachud";

const int SERVO1_PIN = 2;
const int SERVO2_PIN = 3;

const int SERVO_UP_DEG = 150;
const int SERVO_NEUTRAL_DEG = 90;
const int SERVO_DOWN_DEG = 30;

const int TEST_CONTROL_DEG = SERVO_NEUTRAL_DEG - SERVO_DOWN_DEG;

const unsigned long SAMPLE_INTERVAL_MS = 500;
const unsigned long PHASE_TIME_MS = 5000;
const int MAX_LOG_POINTS = 80;

WiFiServer server(80);
Servo servo1;
Servo servo2;

struct DataPoint {
  unsigned long timeMs;
  float depth;
  const char* state;
  int control;
};

DataPoint logData[MAX_LOG_POINTS];
int logIndex = 0;

bool missionStarted = false;
bool missionComplete = false;
unsigned long missionStartMs = 0;
unsigned long lastSampleMs = 0;
int lastControl = 0;

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

void setBuoyancyCommand(int control) {
  lastControl = constrain(control, -TEST_CONTROL_DEG, TEST_CONTROL_DEG);
  int servoDeg = constrain(SERVO_NEUTRAL_DEG - lastControl, SERVO_DOWN_DEG, SERVO_UP_DEG);
  servo1.write(servoDeg);
  servo2.write(servoDeg);
}

int phaseIndex(unsigned long elapsedMs) {
  int phase = elapsedMs / PHASE_TIME_MS;
  if (phase > 7) {
    return 8;
  }
  return phase;
}

const char* phaseState(int phase) {
  switch (phase) {
    case 0: return "TIMED_DEEP_1";
    case 1: return "TIMED_HOLD_DEEP_1";
    case 2: return "TIMED_SHALLOW_1";
    case 3: return "TIMED_HOLD_SHALLOW_1";
    case 4: return "TIMED_DEEP_2";
    case 5: return "TIMED_HOLD_DEEP_2";
    case 6: return "TIMED_SHALLOW_2";
    case 7: return "TIMED_HOLD_SHALLOW_2";
  }
  return "MISSION_COMPLETE";
}

int phaseControl(int phase) {
  switch (phase) {
    case 0:
    case 4:
      return TEST_CONTROL_DEG;
    case 2:
    case 6:
      return -TEST_CONTROL_DEG;
  }
  return 0;
}

float simulatedDepth(int phase) {
  switch (phase) {
    case 0:
    case 1:
    case 4:
    case 5:
      return 2.5;
    case 2:
    case 3:
    case 6:
    case 7:
      return 0.4;
  }
  return 0.4;
}

void beginTimedMission() {
  missionStarted = true;
  missionComplete = false;
  missionStartMs = millis();
  lastSampleMs = 0;
  logIndex = 0;
  setBuoyancyCommand(0);
  Serial.println("TIMED SERVO TEST STARTED");
}

void updateTimedMission() {
  if (!missionStarted || missionComplete) {
    return;
  }

  unsigned long now = millis();
  unsigned long elapsedMs = now - missionStartMs;
  int phase = phaseIndex(elapsedMs);

  setBuoyancyCommand(phaseControl(phase));

  if (logIndex < MAX_LOG_POINTS && (logIndex == 0 || now - lastSampleMs >= SAMPLE_INTERVAL_MS)) {
    logData[logIndex].timeMs = elapsedMs;
    logData[logIndex].depth = simulatedDepth(phase);
    logData[logIndex].state = phaseState(phase);
    logData[logIndex].control = lastControl;
    logIndex++;
    lastSampleMs = now;
  }

  if (phase >= 8) {
    setBuoyancyCommand(0);
    missionComplete = true;
    Serial.println("TIMED SERVO TEST COMPLETE");
  }
}

void sendLoggedData(WiFiClient& client) {
  client.println("MISSION COMPLETE");
  client.println("time,depth,state,control");

  for (int i = 0; i < logIndex; i++) {
    client.print(logData[i].timeMs / 1000.0, 3);
    client.print(",");
    client.print(logData[i].depth, 3);
    client.print(",");
    client.print(logData[i].state);
    client.print(",");
    client.println(logData[i].control);
  }

  client.println("END_DATA");
  client.flush();
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  if (!missionStarted) {
    client.println("FLOAT READY");
    client.println("SEND START TO BEGIN");
    client.flush();

    String command = readClientLine(client, 30000);
    if (command == "START") {
      client.println("START ACK");
      client.flush();
      beginTimedMission();
    } else {
      client.println("NO START RECEIVED");
      client.flush();
    }
  } else if (!missionComplete) {
    client.println("FLOAT BUSY");
    client.println("MISSION STILL RUNNING");
    client.flush();
  } else {
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
  }

  client.stop();
}

void setup() {
  Serial.begin(115200);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  setBuoyancyCommand(0);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Upload this sketch to test timed servo movement above ground.");

  server.begin();
}

void loop() {
  updateTimedMission();
  handleClient();
  delay(20);
}
