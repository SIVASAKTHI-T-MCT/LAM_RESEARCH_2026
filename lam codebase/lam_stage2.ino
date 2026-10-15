/*
 * LAM Research Challenge 3.0 - Stage 2 Controller
 * Team Practical Engineering Challenge
 * Hardware: ESP32 NodeMCU-32S, Custom PCB, 3x BTS7960 Drivers, FT-01 Sensor, OV-01 Solenoid
 *
 * Operating Modes:
 *   Case 1: Process fluid transfer (P1=50%, P2=100%, OV-01 Open) with automatic sump level protection
 *   Case 2: Flush sequence (OV-01 Closed, P1=100%, P2=100%, P3=100%) -> Safe auto-shutdown at 55s
 *
 * Baud Rate: 115200 baud
 */

#include <Arduino.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// Motor driver pins (BTS7960)
struct Pump {
  int lEn, rEn, rPwm, lPwm; // Driver pins
  int ch;                   // PWM channel (Core 2.x)
  int target;               // Target duty cycle (0-100%)
  unsigned long kickEnd;    // Startup kick timer
};

//                    L_EN  R_EN  R_PWM  L_PWM  ch  target  kickEnd
Pump P1 = {13, 26, 33, 32, 0, 0, 0}; // Pump-01: Process fluid from 10L tank
Pump P2 = {16, 17, 18, 19, 1, 0, 0}; // Pump-02: Recirculation from sump through FT-01
Pump P3 = {21, 22, 25, 23, 2, 0, 0}; // Pump-03: Flush fluid from 4.2L tank

// Sensors and actuators
const int PIN_SOLENOID = 27; // OV-01 Solenoid valve (HIGH = Open)
const int PIN_FLOAT    = 35; // Sump float switch (LOW = High liquid level)
const int PIN_ONOFF    = 34; // Rig push button switch (LOW = Pressed)
const int PIN_FLOW     = 14; // FT-01 Flow sensor pulse signal (Yellow wire)

const bool SOLENOID_ACTIVE_HIGH = true;
const int FLOAT_TRIGGERED_LEVEL = LOW;
const int ONOFF_ON_LEVEL        = LOW;

// Calibration and settings
// FT-01 Calibrated factor: 5266 pulses per Liter
float calA = 60.0 / 5266.0; // Q (L/min) = calA * frequency(Hz)
float calB = 0.0;

int cfgP1 = 50;       // Case 1 Pump-01 duty cycle: 50%
int cfgP2 = 100;      // Case 1 Pump-02 duty cycle: 100%
int cfgP3 = 100;      // Case 2 Pump-03 duty cycle: 100%
int cfgCase1 = 300;   // Max Case 1 timeout (5 minutes fallback)
float cfgEndQ = 0.10;
int cfgEndHold = 5;
int cfgMinFlush = 15;
int cfgFloatLow = 1;

// Timing parameters
const unsigned long KICK_MS              = 300;    // 300 ms kick at pump startup
const unsigned long SUMP_FILL_MS         = 60000;  // 60 s delay for P1 to fill sump before P2 starts
const unsigned long FLOAT_DEBOUNCE_HI_MS = 500;    // 500 ms debounce for float HIGH
const unsigned long FLOAT_DEBOUNCE_LO_MS = 2000;   // 2 s debounce to confirm sump level reduced
const unsigned long CASE2_DELAY_MS       = 120000; // 120 s countdown to Case 2 after P1 resumes

const unsigned long C2_VALVE_DELAY_MS    = 5000;   // 5 s valve settling delay before P3 starts
const unsigned long C2_P3_RUN_MS         = 55000;  // 55 s P3 flush duration (4.2L tank empty)
const unsigned long C2_SHUTDOWN_MS       = C2_VALVE_DELAY_MS + C2_P3_RUN_MS; // Total Case 2 time: 60 s

const int PWM_FREQ = 20000;
const int PWM_BITS = 8;
const unsigned long DATA_INTERVAL_MS = 500; // Dashboard telemetry every 500 ms

// State machine variables
enum State { ST_IDLE, ST_CASE1, ST_CASE2, ST_DONE, ST_MANUAL };
State state = ST_IDLE;
unsigned long stateStart = 0;

// Case 1 Tracking
bool p2Started = false;
bool floatTriggered = false;
unsigned long floatTriggeredAt = 0;
bool p1Restarted = false;
unsigned long p1RestartedAt = 0;
bool warned30s = false;

// Case 2 Tracking
bool c2P3Started = false;
bool c2ShutdownLogged = false;

// I/O Status
bool valveOpen = false;
bool levelHigh = false;
bool switchRun = false;
bool serialRun = false;

// Flow Measurement
volatile unsigned long totalPulses = 0;
volatile unsigned long lastPulseUs = 0;
unsigned long prevPulses = 0, lastTick = 0;
float flowHz = 0, flowLpm = 0, totalL = 0;

// Calibration Trial Variables (for 'TRIAL' command)
bool trialActive = false;
unsigned long trialP0 = 0;
unsigned long trialT0 = 0;

// Flow sensor interrupt (FT-01 on Pin 14)
void IRAM_ATTR countPulse() {
  unsigned long now = micros();
  if (now - lastPulseUs > 200) { // 200 µs debounce filter
    totalPulses++;
    lastPulseUs = now;
  }
}

// Helper functions
const char *stateStr() {
  switch (state) {
    case ST_IDLE:   return "IDLE";
    case ST_CASE1:  return "CASE1";
    case ST_CASE2:  return "CASE2";
    case ST_DONE:   return "DONE";
    case ST_MANUAL: return "MANUAL";
  }
  return "?";
}

void ev(const char *msg) {
  Serial.printf("E,%lu,%s\n", millis(), msg);
  float el = (state != ST_IDLE) ? (millis() - stateStart) / 1000.0 : 0.0;
  Serial.printf(">>> [EVENT @ %5.1fs] %s\n", el, msg);
}

void evf(const char *fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.printf("E,%lu,%s\n", millis(), buf);
  float el = (state != ST_IDLE) ? (millis() - stateStart) / 1000.0 : 0.0;
  Serial.printf(">>> [EVENT @ %5.1fs] %s\n", el, buf);
}

void sendConfig() {
  Serial.printf("C,%.6f,%.6f,%d,%d,%d,%d,%.2f,%d,%d,%d\n", calA, calB, cfgP1,
                cfgP2, cfgP3, cfgCase1, cfgEndQ, cfgEndHold, cfgMinFlush,
                cfgFloatLow);
}

void printSetupInfo() {
  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.println("  LAM Research Challenge 3.0 - Stage 2 Controller");
  Serial.println("--------------------------------------------------");
  Serial.printf ("  Calibration : Q = %.6f * f  (5266 pulses/L)\n", calA);
  Serial.println("  Flow Target : 3.0 L/min (Range: 1.0 - 6.0 L/min)");
  Serial.println("  Case 1: P1=50%, P2=100%, Sump protection 20s pause");
  Serial.println("  Case 2: P1/P2/P3=100%, 55s flush -> auto-shutdown");
  Serial.println("  Status: READY - Press ON/OFF push button to start");
  Serial.println("--------------------------------------------------");
  Serial.println();
}

// PWM and pump control
void pwmBegin(Pump &p) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(p.rPwm, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(p.ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(p.rPwm, p.ch);
#endif
}

void pwmWrite(Pump &p, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(p.rPwm, duty);
#else
  ledcWrite(p.ch, duty);
#endif
}

void pumpInit(Pump &p) {
  if (p.rPwm < 0) return;
  pinMode(p.lEn, OUTPUT);
  pinMode(p.rEn, OUTPUT);
  pinMode(p.lPwm, OUTPUT);
  digitalWrite(p.lPwm, LOW); // Forward rotation only
  pwmBegin(p);
  pwmWrite(p, 0);            // Start OFF
  digitalWrite(p.lEn, HIGH);
  digitalWrite(p.rEn, HIGH);
}

void pumpSet(Pump &p, int pct) {
  pct = constrain(pct, 0, 100);
  if (pct > 0 && pct < 100 && p.target == 0) {
    p.kickEnd = millis() + KICK_MS;
  }
  p.target = pct;
}

void pumpApply(Pump &p) {
  if (p.rPwm < 0) return;
  bool kicking = p.target > 0 && (long)(p.kickEnd - millis()) > 0;
  pwmWrite(p, kicking ? 255 : (p.target * 255) / 100);
}

void valve(bool open) {
  valveOpen = open;
  if (PIN_SOLENOID >= 0) {
    digitalWrite(PIN_SOLENOID, (open == SOLENOID_ACTIVE_HIGH) ? HIGH : LOW);
  }
}

void allOff() {
  pumpSet(P1, 0);
  pumpSet(P2, 0);
  pumpSet(P3, 0);
  valve(false);
}

// Read ON/OFF push button (starts/stops sequence)
void updateSwitch() {
  if (PIN_ONOFF < 0) return;
  static bool lastRaw = false, stable = false;
  static unsigned long since = 0;
  bool pressed = (digitalRead(PIN_ONOFF) == ONOFF_ON_LEVEL);

  if (pressed != lastRaw) {
    lastRaw = pressed;
    since = millis();
  }
  if (pressed != stable && millis() - since >= 50) {
    stable = pressed;
    if (stable) {
      switchRun = !switchRun;
      ev(switchRun ? "BUTTON PRESSED: Sequence Started" : "BUTTON PRESSED: Sequence Stopped");
    }
  }
}

// Read sump float switch (protects against overflow)
void updateLevel() {
  if (PIN_FLOAT < 0) {
    levelHigh = false;
    return;
  }
  static bool lastRaw = false;
  static unsigned long rawSince = 0;
  bool raw = (digitalRead(PIN_FLOAT) == (cfgFloatLow ? LOW : HIGH));

  if (raw != lastRaw) {
    lastRaw = raw;
    rawSince = millis();
  }
  unsigned long need = raw ? FLOAT_DEBOUNCE_HI_MS : FLOAT_DEBOUNCE_LO_MS;
  if (raw != levelHigh && millis() - rawSince >= need) {
    bool prev = levelHigh;
    levelHigh = raw;
    if (levelHigh && !prev) {
      ev("FLOAT: Sump full - Pump-01 paused for 20s");
    }
    if (!levelHigh && prev) {
      ev("FLOAT: Sump level normal - Pump-01 resumed");
    }
  }
}

// Flow Telemetry Reporting
static int telemetryRowCount = 0;
static unsigned long lastSerialTelemetry = 0;
static unsigned long lastIdleStatus = 0;

void printTelemetryHeader() {
  Serial.println();
  Serial.println("----------------------------------------------------------------------------------------------------------");
  Serial.println(" TIME(s) | STATE | FLOW(L/m) | FLOW(mL/s) | VEL(m/s) | REYNOLDS | FREQ(Hz) | TOTAL(L) | P1  P2  P3 | OV-01 | FLOAT");
  Serial.println("----------------------------------------------------------------------------------------------------------");
}

void updateFlowAndSend() {
  unsigned long now = millis();
  if (now - lastTick < DATA_INTERVAL_MS) return;
  float dt = (now - lastTick) / 1000.0;
  lastTick = now;

  noInterrupts();
  unsigned long p = totalPulses;
  interrupts();
  unsigned long d = p - prevPulses;
  prevPulses = p;

  flowHz = d / dt;
  float q = calA * flowHz + calB;
  flowLpm = (d > 0 && q > 0) ? q : 0;

  // Totalize volume only while process runs
  if (state == ST_CASE1 || state == ST_CASE2) {
    totalL += flowLpm * dt / 60.0;
  }

  float elapsed = (state != ST_IDLE) ? (now - stateStart) / 1000.0 : 0;

  // 1. Dashboard data packet
  Serial.printf("D,%lu,%s,%.1f,%.3f,%.2f,%lu,%.3f,%d,%d,%d,%d,%d,%d\n", now,
                stateStr(), elapsed, flowLpm, flowHz, p, totalL, P1.target,
                P2.target, P3.target, valveOpen ? 1 : 0, levelHigh ? 1 : 0,
                switchRun ? 1 : 0);

  // 2. Serial Monitor telemetry table (every 1 second)
  if (state == ST_CASE1 || state == ST_CASE2 || state == ST_MANUAL) {
    if (now - lastSerialTelemetry >= 1000) {
      lastSerialTelemetry = now;
      if (telemetryRowCount % 15 == 0) {
        printTelemetryHeader();
      }
      telemetryRowCount++;

      float flowMls = flowLpm * 1000.0 / 60.0;
      float velocity = flowMls / 50.2655;  // 8 mm pipe diameter -> Area = 50.2655 mm²
      float reynolds = 8000.0 * velocity;  // Re = (rho * v * D) / mu = (1000 * v * 0.008) / 1e-3

      Serial.printf(" %6.1fs | %-5s |  %6.3f   |   %6.2f   |  %5.3f   |  %6.0f  |  %6.1f  |  %6.3f  |%3d%% %3d%% %2d%%| %-4s  | %-5s\n",
                    elapsed, stateStr(), flowLpm, flowMls, velocity, reynolds, flowHz, totalL,
                    P1.target, P2.target, P3.target,
                    valveOpen ? "OPEN" : "SHUT",
                    levelHigh ? "HIGH!" : "NORM");
    }
  } else if (state == ST_IDLE) {
    if (now - lastIdleStatus >= 4000) {
      lastIdleStatus = now;
      Serial.println("[IDLE] System ready. Press the ON/OFF push button on the rig to start.");
    }
  }
}

// State machine logic
void startCase1() {
  totalL = 0;
  p2Started = false;
  floatTriggered = false;
  floatTriggeredAt = 0;
  p1Restarted = false;
  p1RestartedAt = 0;
  warned30s = false;
  state = ST_CASE1;
  stateStart = millis();
  telemetryRowCount = 0;

  Serial.println();
  Serial.println(">>> -----------------------------------------------------------");
  Serial.println(">>> CASE 1: PROCESS FLUID TRANSFER & SUMP LEVEL PROTECTION");
  Serial.println(">>>         OV-01 Open, Pump-01 at 50%, filling sump (60 s)...");
  Serial.println(">>> -----------------------------------------------------------");
  printTelemetryHeader();

  ev("CASE 1 START: OV-01 open, Pump-01 50%, sump filling (60 s)...");
}

void startCase2() {
  c2ShutdownLogged = false;
  c2P3Started = false;
  state = ST_CASE2;
  stateStart = millis();
  telemetryRowCount = 0;
  valve(false); // Isolate 10L process tank immediately

  Serial.println();
  Serial.println(">>> -----------------------------------------------------------");
  Serial.println(">>> CASE 2: LINE FLUSH SEQUENCE & AUTO SHUTDOWN");
  Serial.println(">>>         Solenoid OV-01 CLOSED (10L process tank isolated)");
  Serial.println(">>>         Auto valve delay: 5 seconds before Pump-03 starts");
  Serial.printf (">>>         Pump-03 flush runs for %lu s -> Safe Shutdown\n", C2_P3_RUN_MS / 1000);
  Serial.println(">>> -----------------------------------------------------------");
  printTelemetryHeader();

  ev("CASE 2 START: OV-01 closed (10L tank isolated). 5 s delay before P3 starts.");
}

void systemDone(const char *reason) {
  allOff();
  state = ST_DONE;
  stateStart = millis();
  evf("SEQUENCE COMPLETE: %s. Total through FT-01: %.3f L", reason, totalL);

  Serial.println();
  Serial.println(">>> -----------------------------------------------------------");
  Serial.printf (">>> SEQUENCE COMPLETE: %s\n", reason);
  Serial.printf (">>> Total Fluid Measured through FT-01: %.3f L\n", totalL);
  Serial.println(">>> ALL PUMPS OFF | OV-01 CLOSED | SYSTEM SAFELY SHUT DOWN");
  Serial.println(">>> -----------------------------------------------------------");

  serialRun = false;
  switchRun = false;
}

void runControl() {
  bool runReq = switchRun || serialRun;
  unsigned long now = millis();

  switch (state) {

    case ST_IDLE:
      allOff();
      if (runReq) startCase1();
      break;

    case ST_CASE1: {
      if (!runReq) {
        allOff();
        state = ST_IDLE;
        ev("STOPPED by user");
        break;
      }

      valve(true); // OV-01 Open
      unsigned long elapsed = now - stateStart;

      // Phase A: Sump filling (P1 only, P2 waits 60s or until float HIGH)
      if (!p2Started) {
        if (elapsed >= SUMP_FILL_MS || levelHigh) {
          p2Started = true;
          ev(levelHigh ? "Pump-02 starts (sump level HIGH, ready)" : "Pump-02 starts (60 s fill delay complete)");
        }
      }

      // Step 1: Detect float switch HIGH -> stop Pump-01 while Pump-02 drains sump
      bool p1Active = false;
      if (levelHigh) {
        if (!floatTriggered) {
          floatTriggered = true;
          p1Restarted = false;
          ev("FLOAT HIGH: External fluid detected! Pump-01 STOPPED. Pump-02 draining sump at 100%...");
        }
        p1Active = false;
      }
      // Step 2: Fluid level has reduced in the sump (float sensor reading back to normal)
      else if (floatTriggered && !p1Restarted) {
        p1Restarted = true;
        p1RestartedAt = now;
        p1Active = true;
        ev("FLOAT CLEARED: Sump level reduced. Pump-01 RESUMED at 50%. Case 2 in 120 s.");
      }
      // Step 3: Normal operation (before trigger or after restart)
      else {
        p1Active = true;
      }

      pumpSet(P1, p1Active ? cfgP1 : 0);
      pumpSet(P2, p2Started ? cfgP2 : 0);
      pumpSet(P3, 0);

      // Step 4: Countdown to Case 2 (120 s after P1 resumes)
      if (p1Restarted) {
        unsigned long timeSinceRestart = now - p1RestartedAt;
        if (!warned30s && (timeSinceRestart >= CASE2_DELAY_MS - 30000)) {
          warned30s = true;
          ev("Case 2 in 30 seconds. Ensure flush line ball valve is OPEN!");
        }
        if (timeSinceRestart >= CASE2_DELAY_MS) {
          startCase2();
        }
      } else if (elapsed >= (unsigned long)cfgCase1 * 1000UL) {
        ev("Max Case 1 time reached - transitioning to Case 2");
        startCase2();
      }
      break;
    }

    case ST_CASE2: {
      if (!runReq) {
        allOff();
        state = ST_IDLE;
        ev("STOPPED by user");
        break;
      }

      valve(false); // OV-01 Closed (isolate 10L tank)
      unsigned long elapsed = now - stateStart;

      // Phase 1: 5 s settling delay
      if (elapsed < C2_VALVE_DELAY_MS) {
        pumpSet(P1, 0);
        pumpSet(P2, 0);
        pumpSet(P3, 0);
      }
      // Phase 2: All 3 pumps run at 100% for 55 s
      else if (elapsed < C2_SHUTDOWN_MS) {
        if (!c2P3Started) {
          c2P3Started = true;
          ev("5 s valve delay complete: Pump-03 STARTED at 100% (Flush underway)");
        }
        pumpSet(P1, 100);
        pumpSet(P2, 100);
        pumpSet(P3, 100);
      }
      // Phase 3: Flush finished -> Auto safe shutdown
      else {
        pumpSet(P1, 0);
        pumpSet(P2, 0);
        pumpSet(P3, 0);
        valve(false);

        if (!c2ShutdownLogged) {
          c2ShutdownLogged = true;
          Serial.println();
          Serial.println(">>> -----------------------------------------------------------");
          Serial.println(">>> Flush complete: 4.2L delivered in ~55s. System stopped.");
          Serial.println(">>> -----------------------------------------------------------");
          ev("Flush tank (4.2L) empty - whole system shut down");
        }
        systemDone("4.2L flush tank empty (55 s) - safe shutdown");
      }
      break;
    }

    case ST_DONE:
      allOff();
      if (!runReq) state = ST_IDLE;
      break;

    case ST_MANUAL:
      break;
  }
}

// Serial command handler
void handleSerial() {
  if (Serial.available() == 0) return;
  String raw = Serial.readStringUntil('\n');
  raw.trim();
  if (raw.length() == 0) return;

  String c = raw;
  c.toUpperCase();
  int sp1 = c.indexOf(' ');
  String cmd = (sp1 > 0) ? c.substring(0, sp1) : c;

  if (cmd == "START") {
    serialRun = true;
    if (state == ST_MANUAL || state == ST_DONE) {
      allOff();
      state = ST_IDLE;
    }
    ev("START requested");
  } else if (cmd == "STOP") {
    serialRun = false;
    switchRun = false;
    allOff();
    state = ST_DONE;
    ev("STOP - all outputs OFF");
  } else if (cmd == "FLUSH") {
    serialRun = true;
    if (state != ST_CASE2) startCase2();
  } else if (cmd == "RESETTOT") {
    totalL = 0;
    ev("Totalizer reset to 0");
  } else if (cmd == "PUMP") {
    if (state == ST_CASE1 || state == ST_CASE2) {
      ev("Cannot use manual pump while auto sequence is running. Press STOP first.");
      return;
    }
    if (state != ST_MANUAL) { allOff(); state = ST_MANUAL; }
    int n = c.substring(sp1 + 1, c.indexOf(' ', sp1 + 1)).toInt();
    int pct = c.substring(c.lastIndexOf(' ') + 1).toInt();
    if (n == 1) pumpSet(P1, pct);
    else if (n == 2) pumpSet(P2, pct);
    else if (n == 3) pumpSet(P3, pct);
    evf("Manual: Pump-%d set to %d%%", n, pct);
  } else if (cmd == "VALVE") {
    if (state == ST_CASE1 || state == ST_CASE2) return;
    if (state != ST_MANUAL) { allOff(); state = ST_MANUAL; }
    valve(c.substring(sp1 + 1).toInt() != 0);
    evf("Manual: valve %s", valveOpen ? "OPEN" : "CLOSED");
  } else if (cmd == "CAL") {
    int sp2 = raw.indexOf(' ', sp1 + 1);
    calA = raw.substring(sp1 + 1, sp2).toFloat();
    calB = raw.substring(sp2 + 1).toFloat();
    sendConfig();
    evf("Calibration saved: Q = %.6f * f + %.6f", calA, calB);
  } else if (cmd == "SET") {
    int sp2 = c.indexOf(' ', sp1 + 1);
    String key = c.substring(sp1 + 1, sp2);
    float val = c.substring(sp2 + 1).toFloat();
    if (key == "P1") cfgP1 = (int)val;
    else if (key == "P2") cfgP2 = (int)val;
    else if (key == "P3") cfgP3 = (int)val;
    else if (key == "CASE1") cfgCase1 = (int)val;
    sendConfig();
  } else if (cmd == "GETCFG") {
    sendConfig();
  } else if (cmd == "TRIAL") {
    String sub = c.substring(sp1 + 1);
    sub.trim();
    if (sub == "START") {
      noInterrupts();
      trialP0 = totalPulses;
      interrupts();
      trialT0 = millis();
      trialActive = true;
      ev("Calibration trial started - counting pulses...");
    } else if (sub == "STOP" && trialActive) {
      noInterrupts();
      unsigned long p = totalPulses;
      interrupts();
      unsigned long dn = p - trialP0;
      float dt = (millis() - trialT0) / 1000.0;
      trialActive = false;
      Serial.printf("T,%lu,%.3f\n", dn, dt);
      evf("Trial done: %lu pulses in %.3f s (f = %.2f Hz)", dn, dt, dt > 0 ? dn / dt : 0);
    }
  } else {
    ev("Unknown command. Use: START STOP FLUSH PUMP VALVE CAL SET GETCFG TRIAL RESETTOT");
  }
}

// Hardware setup
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pumpInit(P1);
  pumpInit(P2);
  pumpInit(P3);

  if (PIN_SOLENOID >= 0) {
    pinMode(PIN_SOLENOID, OUTPUT);
    valve(false);
  }
  if (PIN_FLOAT >= 0) {
    pinMode(PIN_FLOAT, INPUT);
  }
  if (PIN_ONOFF >= 0) {
    pinMode(PIN_ONOFF, INPUT);
  }

  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), countPulse, RISING);

  printSetupInfo();
  sendConfig();
  lastTick = millis();
}

// Main loop
void loop() {
  handleSerial();
  updateSwitch();
  updateLevel();
  runControl();
  pumpApply(P1);
  pumpApply(P2);
  pumpApply(P3);
  updateFlowAndSend();
}
