#include <SoftwareSerial.h>

volatile long motCount = 0;
volatile bool isrStopNow = false;

// --- Serial Link to Arduino #1 ---
const int RX_PIN = A0;  // from #1 TX (A1)
const int TX_PIN = A1;  // to   #1 RX (A0)
SoftwareSerial linkSerial(RX_PIN, TX_PIN);
String buffer = "";

// --- Motor A Pins (L298N) ---
const int enA = 10;
const int in1 = 9;
const int in2 = 4;

// --- Motor B Pins (L298N) ---
const int enB = 5;
const int in3 = 7;
const int in4 = 6;

// --- Motor C Pins (L298N) ---
const int enC = 11;
const int in5 = 12;
const int in6 = 13;

// --- Encoder Pins ---
const int motBEncA = 2;   // interrupt-capable
const int motBEncB = A5;

// --- Control constants ---
const int SPEED_MAX     = 50;
const int SPEED_MIN     = 20;
const int DECEL_WINDOW  = 250;
const int STOP_OFFSET   = 20;

enum State { IDLE, RUNNING, DONE };
State state = IDLE;

long startCount = 0;
long targetCount = 0;
String currentCmd = "";
int motionDir = 1;   // +1 forward, -1 backward
char motionType = ' '; // 'w' or 'r'

void setup() {
  // Motor pins
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(enC, OUTPUT); pinMode(in5, OUTPUT); pinMode(in6, OUTPUT);

  // Encoder pins
  pinMode(motBEncA, INPUT_PULLUP);
  pinMode(motBEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motBEncA), EncoderEvent, CHANGE);

  Serial.begin(9600);
  linkSerial.begin(9600);
  Serial.println("Arduino #2 ready (bidirectional triple-motor FSM)");
}

// ============================================================
// Encoder interrupt: quadrature decode
// ============================================================
void EncoderEvent() {
  if (digitalRead(motBEncA) == HIGH) {
    if (digitalRead(motBEncB) == LOW) motCount++;
    else motCount--;
  } else {
    if (digitalRead(motBEncB) == LOW) motCount--;
    else motCount++;
  }

  if (state == RUNNING && abs(motCount) >= (abs(targetCount) - STOP_OFFSET)) {
    isrStopNow = true;
  }
}

// ============================================================
// Motor helpers
// ============================================================
void driveMotorA(int pwm, int dir) {
  if (dir > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  }
  analogWrite(enA, pwm);
}

void driveMotorB(int pwm, int dir) {
  if (dir > 0) {
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  }
  analogWrite(enB, pwm);
}

void driveMotorC(int pwm, int dir) {
  if (dir > 0) {
    digitalWrite(in5, HIGH); digitalWrite(in6, LOW);
  } else {
    digitalWrite(in5, LOW);  digitalWrite(in6, HIGH);
  }
  analogWrite(enC, pwm);
}

// drive specific motors based on command type
void driveMotors(int pwm, int dir, char type) {
  if (type == 'w') {
    driveMotorB(pwm, dir);
    driveMotorC(pwm, dir);
  } else if (type == 'r') {
    driveMotorA(pwm, dir);
    driveMotorB(pwm, dir);
    driveMotorC(pwm, dir);
  }
}

// braking helpers
void shortBrakeMotors(char type) {
  if (type == 'w' || type == 'r') {
    digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
    analogWrite(enB, 255);
    digitalWrite(in5, HIGH); digitalWrite(in6, HIGH);
    analogWrite(enC, 255);
  }
  if (type == 'r') {
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
    analogWrite(enA, 255);
  }
}

void coastMotors(char type) {
  if (type == 'w' || type == 'r') {
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
    analogWrite(enB, 0);
    digitalWrite(in5, LOW); digitalWrite(in6, LOW);
    analogWrite(enC, 0);
  }
  if (type == 'r') {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}

unsigned long lastPrintMs = 0;

// ============================================================
// FSM
// ============================================================
void loop() {
  switch (state) {

    // --------------------------------------------------------
    // IDLE: wait for command
    // --------------------------------------------------------
    case IDLE: {
      linkSerial.listen();
      while (linkSerial.available()) {
        char c = linkSerial.read();
        if (c == '\n') {
          buffer.trim();
          if (buffer.length()) {
            char prefix = tolower(buffer.charAt(0));
            String numPart = buffer.substring(1);
            long rotations = numPart.toInt();

            if (prefix == 'w' || prefix == 'r') {
              motionType = prefix;
              motionDir = (rotations >= 0) ? 1 : -1;
              long targetTicks = abs(rotations) * 100L;

              noInterrupts();
              startCount = motCount;
              interrupts();
              targetCount = targetTicks * motionDir;
              currentCmd = buffer;

              Serial.print("Cmd "); Serial.print(currentCmd);
              Serial.print("  start "); Serial.print(startCount);
              Serial.print("  target "); Serial.println(targetCount);

              driveMotors(SPEED_MAX, motionDir, motionType);
              isrStopNow = false;
              state = RUNNING;
            } else {
              Serial.print("Unknown command: "); Serial.println(buffer);
            }
          }
          buffer = "";
        } else {
          buffer += c;
        }
      }
    } break;

    // --------------------------------------------------------
    // RUNNING: decel + braking
    // --------------------------------------------------------
    case RUNNING: {
      long mc;
      noInterrupts(); mc = motCount; interrupts();
      long remaining = abs(targetCount) - abs(mc);

      if (remaining <= DECEL_WINDOW && remaining > 0) {
        int span = max(1, DECEL_WINDOW);
        int pwm  = map(remaining, 0, span, SPEED_MIN, SPEED_MAX);
        pwm = constrain(pwm, SPEED_MIN, SPEED_MAX);
        driveMotors(pwm, motionDir, motionType);
      }

      if (isrStopNow || abs(mc) >= (abs(targetCount) - STOP_OFFSET)) {
        shortBrakeMotors(motionType);
        delay(30);
        coastMotors(motionType);
        Serial.print("Braked at count "); Serial.println(mc);
        state = DONE;
      } else if (millis() - lastPrintMs > 100) {
        lastPrintMs = millis();
        Serial.print("mc="); Serial.println(mc);
      }
    } break;

    // --------------------------------------------------------
    // DONE: send back actual encoder count
    // --------------------------------------------------------
    case DONE: {
      long finalCount;
      noInterrupts();
      finalCount = motCount;
      motCount = 0;  // reset
      interrupts();

      // Response: "r-3:-3012" or "w2:1995"
      linkSerial.print(currentCmd);
      linkSerial.print(":");
      linkSerial.print(finalCount);
      linkSerial.print('\n');

      Serial.print("Sent back actual count → ");
      Serial.print(currentCmd);
      Serial.print(":");
      Serial.println(finalCount);

      state = IDLE;
    } break;
  }
}
