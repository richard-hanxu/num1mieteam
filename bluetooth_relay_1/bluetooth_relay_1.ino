#include <SoftwareSerial.h>

// --- HM-10 Bluetooth ---
const int BT_RX = 11;   // receives from HM-10 TX
const int BT_TX = 10;   // transmits to HM-10 RX
SoftwareSerial btSerial(BT_RX, BT_TX);

// --- Link to Arduino #2 ---
const int LINK_RX = A0;  // receives from Arduino #2 TX
const int LINK_TX = A1;  // transmits to Arduino #2 RX
SoftwareSerial linkSerial(LINK_RX, LINK_TX);

// --- Buffers & flags ---
String btBuffer = "";
String linkBuffer = "";
bool waitingForPython   = true;
bool waitingForResponse = false;

// --- Ultrasonic sensor pins ---
const int s0Trig = 3;   const int s0Echo = 2;
const int s1Trig = A2;  const int s1Echo = A3;
const int s2Trig = 12;  const int s2Echo = 13;
const int s3Trig = 9;   const int s3Echo = 8;
const int s4Trig = 6;   const int s4Echo = 7;
const int s5Trig = 4;   const int s5Echo = 5;

// ------------------------------------------------------------
// Helper: read one ultrasonic sensor (cm)
// ------------------------------------------------------------
float readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2000);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);  // 30 ms timeout
  // if (duration == 0) return -1;                // no echo
  return duration * 0.0343 / (2.0 * 2.54);
  return duration;
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  linkSerial.begin(9600);
  Serial.println("Arduino #1 ready (r/w → #2, s → local ultrasonic)");

  // Configure trig pins as outputs, echo pins as inputs
  pinMode(s0Trig, OUTPUT); pinMode(s0Echo, INPUT);
  pinMode(s1Trig, OUTPUT); pinMode(s1Echo, INPUT);
  pinMode(s2Trig, OUTPUT); pinMode(s2Echo, INPUT);
  pinMode(s3Trig, OUTPUT); pinMode(s3Echo, INPUT);
  pinMode(s4Trig, OUTPUT); pinMode(s4Echo, INPUT);
  pinMode(s5Trig, OUTPUT); pinMode(s5Echo, INPUT);
}

// ------------------------------------------------------------
void loop() {

  // =======================================================
  // 1️⃣  WAITING FOR PYTHON → read Bluetooth input
  // =======================================================
  if (waitingForPython) {
    btSerial.listen();
    while (btSerial.available()) {
      char c = btSerial.read();

      if (c == '\n') {
        btBuffer.trim();
        if (btBuffer.length()) {

          char prefix = tolower(btBuffer.charAt(0)); // first char

          // ---------- Case A: "r" or "w" → forward to Arduino #2 ----------
          if (prefix == 'r' || prefix == 'w') {
            Serial.print("Forwarding command to Arduino #2: ");
            Serial.println(btBuffer);

            linkSerial.listen();
            linkSerial.print(btBuffer);
            linkSerial.print('\n');

            waitingForPython   = false;
            waitingForResponse = true;
          }

          // ---------- Case B: "s" → local ultrasonic reading ----------
          else if (prefix == 's') {
            Serial.println("Local sensor request received (s command)");

            float d[6];
            d[0] = readUltrasonic(s0Trig, s0Echo);
            d[1] = readUltrasonic(s1Trig, s1Echo);
            d[2] = readUltrasonic(s2Trig, s2Echo);
            d[3] = readUltrasonic(s3Trig, s3Echo);
            d[4] = readUltrasonic(s4Trig, s4Echo);
            d[5] = readUltrasonic(s5Trig, s5Echo);

            String msg = "s:";
            for (int i = 0; i < 6; i++) {
              msg += String(d[i], 1);
              if (i < 5) msg += ",";
            }

            Serial.print("Ultrasonic (cm.): ");
            Serial.println(msg);

            btSerial.listen();
            btSerial.print(msg);
            btSerial.print('\n');
            btSerial.flush();  // ensure the HM-10 actually transmits immediately

            // stay in waitingForPython (no link comms)
          }

          // ---------- Case C: unknown prefix ----------
          else {
            Serial.print("Unknown command: ");
            Serial.println(btBuffer);
            btSerial.listen();
            btSerial.print("ERR:UNKNOWN\n");
          }
        }
        btBuffer = "";
      } else {
        btBuffer += c;
      }
    }
  }

  // =======================================================
  // 2️⃣  WAITING FOR RESPONSE → read Arduino #2 input
  // =======================================================
  if (waitingForResponse) {
    linkSerial.listen();
    while (linkSerial.available()) {
      char c = linkSerial.read();
      if (c == '\n') {
        linkBuffer.trim();
        if (linkBuffer.length()) {
          Serial.print("From Arduino #2: ");
          Serial.println(linkBuffer);

          btSerial.listen();
          btSerial.print(linkBuffer);  // include Arduino #2’s response
          btSerial.print('\n');

          waitingForResponse = false;
          waitingForPython   = true;
        }
        linkBuffer = "";
      } else {
        linkBuffer += c;
      }
    }
  }
}
