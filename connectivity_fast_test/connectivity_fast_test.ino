#if !defined(ARDUINO_ARCH_SAMD)
#error "Wrong board selected. Use Tools > Board > Arduino SAMD Boards > Arduino MKR WiFi 1010."
#endif

#include <WiFiNINA.h>

const char* ssid = "t480";
const char* password = "henryisachud";

const unsigned long SAMPLE_INTERVAL_MS = 500;
const int MAX_LOG_POINTS = 80;

WiFiServer server(80);

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

float fakeDepth(unsigned long elapsedMs) {
  float t = elapsedMs / 1000.0;

  if (t < 2.0) {
    return 0.4 + (2.1 * (t / 2.0));
  }
  if (t < 4.0) {
    return 2.5;
  }
  if (t < 6.0) {
    return 2.5 - (2.1 * ((t - 4.0) / 2.0));
  }
  if (t < 8.0) {
    return 0.4;
  }
  if (t < 10.0) {
    return 0.4 + (2.1 * ((t - 8.0) / 2.0));
  }
  if (t < 12.0) {
    return 2.5;
  }
  if (t < 14.0) {
    return 2.5 - (2.1 * ((t - 12.0) / 2.0));
  }
  return 0.4;
}

const char* fakeState(unsigned long elapsedMs) {
  float t = elapsedMs / 1000.0;

  if (t < 2.0) return "GO_TO_DEEP_1";
  if (t < 4.0) return "HOLD_DEEP_1";
  if (t < 6.0) return "GO_TO_SHALLOW_1";
  if (t < 8.0) return "HOLD_SHALLOW_1";
  if (t < 10.0) return "GO_TO_DEEP_2";
  if (t < 12.0) return "HOLD_DEEP_2";
  if (t < 14.0) return "GO_TO_SHALLOW_2";
  if (t < 16.0) return "HOLD_SHALLOW_2";
  return "MISSION_COMPLETE";
}

int fakeControl(unsigned long elapsedMs) {
  const char* state = fakeState(elapsedMs);

  if (String(state).startsWith("GO_TO_DEEP")) return 15;
  if (String(state).startsWith("GO_TO_SHALLOW")) return -15;
  return 0;
}

void beginFakeMission() {
  missionStarted = true;
  missionComplete = false;
  missionStartMs = millis();
  lastSampleMs = 0;
  logIndex = 0;
  Serial.println("FAST TEST MISSION STARTED");
}

void updateFakeMission() {
  if (!missionStarted || missionComplete) {
    return;
  }

  unsigned long now = millis();
  unsigned long elapsedMs = now - missionStartMs;

  if (logIndex < MAX_LOG_POINTS && (logIndex == 0 || now - lastSampleMs >= SAMPLE_INTERVAL_MS)) {
    logData[logIndex].timeMs = elapsedMs;
    logData[logIndex].depth = fakeDepth(elapsedMs);
    logData[logIndex].state = fakeState(elapsedMs);
    logData[logIndex].control = fakeControl(elapsedMs);
    logIndex++;
    lastSampleMs = now;
  }

  if (elapsedMs >= 16000) {
    missionComplete = true;
    Serial.println("FAST TEST MISSION COMPLETE");
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
      beginFakeMission();
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

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Upload this sketch to test WiFi/client protocol only.");

  server.begin();
}

void loop() {
  updateFakeMission();
  handleClient();
  delay(20);
}
