#include <SoftwareSerial.h>

volatile long motCount = 0;
volatile bool isrStopNow = false;   // set in ISR when target reached

// --- Serial Link to Arduino #1 ---
const int RX_PIN = A0;  // from #1 TX (A1)
const int TX_PIN = A1;  // to   #1 RX (A0)
SoftwareSerial linkSerial(RX_PIN, TX_PIN);
String buffer = "";

// --- Motor B Pins (L298N) ---
const int enB = 5;
const int in3 = 7;
const int in4 = 6;

// --- Encoder Pins ---
const int motBEncA = 2;   // interrupt-capable
const int motBEncB = A5;

// --- Control constants ---
const int SPEED_MAX     = 50;   // your original SPEED
const int SPEED_MIN     = 20;   // minimum PWM during decel
const int DECEL_WINDOW  = 250;  // ticks before target to start tapering
const int STOP_OFFSET   = 20;   // stop this many ticks early, then short-brake
const int TARGET_TICKS  = 2000;

enum State { IDLE, RUNNING, DONE };
State state = IDLE;

long startCount = 0;
long targetCount = 0;
int  currentCmd  = 0;

void setup() {
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(motBEncA, INPUT_PULLUP);
  pinMode(motBEncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motBEncA), EncoderEvent, CHANGE);

  Serial.begin(9600);
  linkSerial.begin(9600);
  Serial.println("Arduino #2 ready (FSM motor/encoder with braking & decel)");
}

void EncoderEvent() {
  // Quadrature decode on channel A
  if (digitalRead(motBEncA) == HIGH) {
    if (digitalRead(motBEncB) == LOW) motCount++;
    else motCount--;
  } else {
    if (digitalRead(motBEncB) == LOW) motCount--;
    else motCount++;
  }

  // Early ISR stop trigger (don’t do I/O here)
  if (state == RUNNING && motCount >= (targetCount - STOP_OFFSET)) {
    isrStopNow = true;
  }
}

void driveMotorForward(uint8_t pwm) {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, pwm);
}

// Short-brake (active braking): both inputs HIGH, EN HIGH
void shortBrakeMotor() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 255);        // ensure bridge is actively shorting
}

// Coast: disables torque (optional utility)
void coastMotor() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}

unsigned long lastPrintMs = 0;

void loop() {
  switch (state) {
    case IDLE: {
      linkSerial.listen();
      while (linkSerial.available()) {
        char c = linkSerial.read();
        if (c == '\n') {
          buffer.trim();
          if (buffer.length()) {
            currentCmd  = buffer.toInt();
            noInterrupts();
            startCount  = motCount;
            interrupts();
            targetCount = startCount + TARGET_TICKS;

            Serial.print("Cmd "); Serial.print(currentCmd);
            Serial.print("  start "); Serial.print(startCount);
            Serial.print("  target "); Serial.println(targetCount);

            driveMotorForward(SPEED_MAX);
            isrStopNow = false;
            state = RUNNING;
          }
          buffer = "";
        } else {
          buffer += c;
        }
      }
    } break;

    case RUNNING: {
      long mc;
      noInterrupts(); mc = motCount; interrupts();
      long remaining = (targetCount - STOP_OFFSET) - mc;

      // Decelerate when within window
      if (remaining <= DECEL_WINDOW && remaining > 0) {
        int span = max(1, DECEL_WINDOW);
        int pwm  = map(remaining, 0, span, SPEED_MIN, SPEED_MAX);
        pwm = constrain(pwm, SPEED_MIN, SPEED_MAX);
        driveMotorForward((uint8_t)pwm);
      }

      // Stop condition (either ISR flagged or we’re past early stop)
      if (isrStopNow || mc >= (targetCount - STOP_OFFSET)) {
        shortBrakeMotor();                 // active brake now
        delay(30);                         // brief hold for braking (tune 10–50 ms)
        coastMotor();                      // optionally release torque
        Serial.print("Braked at count "); Serial.println(mc);
        state = DONE;
        motCount = 0;
      } else {
        // Throttle prints to avoid loop jitter
        if (millis() - lastPrintMs > 100) {
          lastPrintMs = millis();
          Serial.print("mc="); Serial.println(mc);
        }
      }
    } break;

    case DONE: {
      int sendVal = currentCmd + 1;
      // small pause to let #1 switch listeners
      delay(30);
      linkSerial.print(sendVal);
      linkSerial.print('\n');
      Serial.print("Sent back: "); Serial.println(sendVal);
      state = IDLE;
    } break;
  }
}
