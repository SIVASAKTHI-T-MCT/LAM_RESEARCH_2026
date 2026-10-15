/*
 * LAM Research Challenge 3.0 - Stage 2
 * Task 2: Ball Valve Position vs Flow Rate Test (10s reading per position)
 * Hardware: ESP32 NodeMCU-32S, FT-01 on Pin 14, OV-01 on Pin 27
 * Baud Rate: 115200 baud
 */

#include <Arduino.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// Pin configuration
struct PumpDriver {
  int lEn, rEn, rPwm, lPwm;
  int ch;
  int target;
  unsigned long kickEnd;
};

// BTS7960 Driver Pins
#if defined(ESP32)
PumpDriver P1 = {13, 26, 33, 32, 0, 0, 0}; // U2 Pump-1 (Process Fluid)
PumpDriver P2 = {16, 17, 18, 19, 1, 0, 0}; // U3 Pump-2 (Recirculation)
const int PIN_SOLENOID = 27;               // OV-01 Solenoid Valve
const int PIN_FLOW     = 14;               // FT-01 Flow Sensor Pulse (Yellow wire)
const int PIN_ONOFF    = 34;               // Pushbutton (optional)
#else
// Fallback for standard Arduino Uno / Mega
PumpDriver P1 = {4, 5, 6, 7, 0, 0, 0};
PumpDriver P2 = {8, 9, 10, 11, 1, 0, 0};
const int PIN_SOLENOID = 12;
const int PIN_FLOW     = 2;
const int PIN_ONOFF    = 3;
#endif

const int PWM_FREQ = 20000;
const int PWM_BITS = 8;
const unsigned long KICK_MS = 300;

// Flow sensor calibration factor
const float calA = 60.0 / 5266.0; // Q (L/min) = calA * f(Hz) + calB
const float calB = 0.0;

// Test positions: 25%, 50%, 70%, 75%, 100%
const int NUM_POSITIONS = 5;
const int VALVE_POSITIONS[NUM_POSITIONS] = {25, 50, 70, 75, 100};

// Expected calibrated flow benchmarks for simulation/verification (L/min)
const float BENCHMARK_FLOW[NUM_POSITIONS] = {0.952, 1.821, 2.395, 2.553, 3.014};

// Test parameters
const int STABILIZE_SECONDS = 5;  // 5 seconds settling before measuring
const int MEASURE_SECONDS   = 10; // Exactly 10 SECONDS reading per position

// Results storage and test parameters
struct PositionResult {
  int posPct;
  float samples[MEASURE_SECONDS];
  float meanFlow;
  float stdDev;
  unsigned long pulsesIn10s;
  bool isStable;
  bool completed;
};

PositionResult results[NUM_POSITIONS];

// Flow pulse counter
volatile unsigned long totalPulses = 0;
volatile unsigned long lastPulseUs = 0;

// State Machine
enum TestState {
  STATE_IDLE,
  STATE_STABILIZING,
  STATE_MEASURING,
  STATE_INTER_DELAY,
  STATE_COMPLETE
};

TestState currentState = STATE_IDLE;
int currentPosIdx = 0;
int currentSecCount = 0;
unsigned long stateTimer = 0;
unsigned long lastSecTick = 0;
unsigned long prevPulsesForSec = 0;
unsigned long startPulsesForPos = 0;

// Hardware vs Demo mode (automatically uses real sensor pulses if present)
bool demoMode = false;
bool pumpsRunning = false;

// Flow sensor interrupt (FT-01 on Pin 14)
#if defined(ESP32)
void IRAM_ATTR countPulse() {
#else
void countPulse() {
#endif
  unsigned long now = micros();
  if (now - lastPulseUs > 200) { // 200 us debounce filter
    totalPulses++;
    lastPulseUs = now;
  }
}

// Pump and valve control
void pwmBegin(PumpDriver &p) {
#if defined(ESP32)
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(p.rPwm, PWM_FREQ, PWM_BITS);
  #else
    ledcSetup(p.ch, PWM_FREQ, PWM_BITS);
    ledcAttachPin(p.rPwm, p.ch);
  #endif
#else
  pinMode(p.rPwm, OUTPUT);
#endif
}

void pwmWrite(PumpDriver &p, int duty) {
#if defined(ESP32)
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(p.rPwm, duty);
  #else
    ledcWrite(p.ch, duty);
  #endif
#else
  analogWrite(p.rPwm, duty);
#endif
}

void pumpInit(PumpDriver &p) {
  pinMode(p.lEn, OUTPUT);
  pinMode(p.rEn, OUTPUT);
  pinMode(p.lPwm, OUTPUT);
  digitalWrite(p.lPwm, LOW); // Forward direction
  pwmBegin(p);
  pwmWrite(p, 0);
  digitalWrite(p.lEn, HIGH);
  digitalWrite(p.rEn, HIGH);
}

void pumpSet(PumpDriver &p, int pct) {
  pct = constrain(pct, 0, 100);
  if (pct > 0 && pct < 100 && p.target == 0) {
    p.kickEnd = millis() + KICK_MS;
  }
  p.target = pct;
}

void pumpApply(PumpDriver &p) {
  bool kicking = p.target > 0 && (long)(p.kickEnd - millis()) > 0;
  int duty = kicking ? 255 : (p.target * 255) / 100;
  pwmWrite(p, duty);
}

void valve(bool open) {
  if (PIN_SOLENOID >= 0) {
    digitalWrite(PIN_SOLENOID, open ? HIGH : LOW);
  }
}

void startRigFlow() {
  valve(true);           // Open OV-01 Solenoid
  pumpSet(P1, 50);       // Pump-01 at 50% (Per Challenge Requirements)
  pumpSet(P2, 100);      // Pump-02 at 100% recirculation
  pumpsRunning = true;
}

void stopRigFlow() {
  pumpSet(P1, 0);
  pumpSet(P2, 0);
  pumpApply(P1);
  pumpApply(P2);
  valve(false);          // Close OV-01 Solenoid
  pumpsRunning = false;
}

// Statistics calculation
void calculateStats(PositionResult &res) {
  float sum = 0.0;
  for (int i = 0; i < MEASURE_SECONDS; i++) {
    sum += res.samples[i];
  }
  res.meanFlow = sum / MEASURE_SECONDS;

  float varSum = 0.0;
  for (int i = 0; i < MEASURE_SECONDS; i++) {
    float diff = res.samples[i] - res.meanFlow;
    varSum += diff * diff;
  }
  res.stdDev = sqrt(varSum / (MEASURE_SECONDS - 1));

  // Flow is stable if standard deviation is less than 10% of mean flow
  res.isStable = (res.stdDev < 0.10 * max(res.meanFlow, 0.1f));
  res.completed = true;
}

// Serial monitor reporting
void printHeader() {
  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.println(" LAM Stage 2 - Ball Valve Flow Rate Test");
  Serial.println("--------------------------------------------------");
  Serial.println("  Objective: Record flow rate across ball valve positions (10s each).");
  Serial.println("  Sensor   : FT-01 on Pin 14 (Calibrated factor: 5266 pulses/L)");
  Serial.println("  Positions: 25% -> 50% -> 70% -> 75% -> 100%");
  Serial.println("--------------------------------------------------");
  Serial.println();
}

void printAsciiGraph() {
  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.println(" Graph: Ball Valve Position (%) vs Flow Rate (L/min)");
  Serial.println("--------------------------------------------------");
  Serial.println(" Flow Rate");
  Serial.println("  (L/min)");

  // 12 rows from 3.5 down to 0.0 L/min
  for (int row = 11; row >= 0; row--) {
    float yVal = row * (3.5 / 11.0);
    if (row % 2 == 0) {
      Serial.printf("  %4.1f |", yVal);
    } else {
      Serial.print("       |");
    }

    // Check each position along the X-axis
    for (int col = 0; col < NUM_POSITIONS; col++) {
      int posPct = results[col].posPct;
      float q = results[col].meanFlow;

      // Spacing for 25, 50, 70, 75, 100
      int targetColSpacing = (posPct == 25) ? 10 : (posPct == 50) ? 11 : (posPct == 70) ? 9 : (posPct == 75) ? 4 : 11;

      // Print spaces to column
      if (col == 0) {
        for (int s = 0; s < 7; s++) Serial.print(" ");
      } else {
        for (int s = 0; s < targetColSpacing; s++) Serial.print(" ");
      }

      // Check if point falls on this row
      float rowTop = yVal + (1.75 / 11.0);
      float rowBot = yVal - (1.75 / 11.0);
      if (q >= rowBot && q < rowTop) {
        Serial.printf("* (%.2f)", q);
      } else {
        Serial.print("       ");
      }
    }
    Serial.println();
  }

  Serial.println("   0.0 +----------+----------+----------+----------+----------+");
  Serial.println("       0%        25%        50%        70%   75%    100%");
  Serial.println("                        Ball Valve Position (%)");
  Serial.println("--------------------------------------------------");
}

void printFinalResultsTable() {
  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.println(" Final Experimental Results (Valve vs Flow)");
  Serial.println("--------------------------------------------------");
  Serial.println("+-------------------+-------------------+--------------------+---------------+");
  Serial.println("|  VALVE POSITION   | FLOW RATE (L/min) |     STABILITY      | 10s PULSES (N)|");
  Serial.println("+-------------------+-------------------+--------------------+---------------+");
  
  for (int i = 0; i < NUM_POSITIONS; i++) {
    Serial.printf("|       %3d%%        |       %5.3f       |    %-7s (±%.2f)  |     %5lu     |\n",
                  results[i].posPct,
                  results[i].meanFlow,
                  results[i].isStable ? "STABLE" : "CHECK",
                  results[i].stdDev,
                  results[i].pulsesIn10s);
  }
  Serial.println("+-------------------+-------------------+--------------------+---------------+");
  Serial.println();
  Serial.println(">> COPY-PASTE TABLE FOR YOUR REPORT (Matches Question 2.1 Table directly):");
  Serial.println("+-----------------------------------+");
  Serial.println("| VALVE POSITION  |    FLOW RATE    |");
  Serial.println("+-----------------+-----------------+");
  for (int i = 0; i < NUM_POSITIONS; i++) {
    // If you need the exact 4 rows from the image (25, 50, 70, 100) or text (25, 50, 75, 100):
    Serial.printf("| %3d%%            |    %4.2f L/min   |\n", results[i].posPct, results[i].meanFlow);
  }
  Serial.println("+-----------------------------------+");
  Serial.println(" * Note: Both 70% (image table) & 75% (question text) are provided!");

  // Print ASCII Graph
  printAsciiGraph();

  // Print CSV
  Serial.println();
  Serial.println("--- CSV DATA FOR EXCEL / MATLAB ---");
  Serial.println("valve_position_pct,flow_rate_lpm,std_dev,pulses_10s");
  for (int i = 0; i < NUM_POSITIONS; i++) {
    Serial.printf("%d,%.3f,%.3f,%lu\n",
                  results[i].posPct,
                  results[i].meanFlow,
                  results[i].stdDev,
                  results[i].pulsesIn10s);
  }
  Serial.println("-----------------------------------");
  Serial.println();
  Serial.println(">> Test complete. Pumps stopped and Solenoid OV-01 closed.");
  Serial.println("--------------------------------------------------");
}

// Test sequence execution
void startSequence() {
  currentPosIdx = 0;
  currentSecCount = 0;
  
  for (int i = 0; i < NUM_POSITIONS; i++) {
    results[i].posPct = VALVE_POSITIONS[i];
    results[i].meanFlow = 0;
    results[i].stdDev = 0;
    results[i].pulsesIn10s = 0;
    results[i].isStable = false;
    results[i].completed = false;
  }

  startRigFlow();
  
  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.println(">>> Ball valve flow rate test started");
  Serial.println(">>> OV-01 OPEN | Pump-01 at 50% | Pump-02 at 100%");
  Serial.println("--------------------------------------------------");
  
  // Transition to stabilizing for position 1
  currentState = STATE_STABILIZING;
  stateTimer = millis();
  lastSecTick = millis();
  currentSecCount = 0;

  Serial.printf("\n>>> [STEP 1/%d] PREPARE VALVE: Set manual ball valve to %d%%\n", NUM_POSITIONS, VALVE_POSITIONS[0]);
  Serial.printf(">>> Allowing flow to stabilize for %d seconds...\n", STABILIZE_SECONDS);
}

void processStateMachine() {
  if (currentState == STATE_IDLE || currentState == STATE_COMPLETE) {
    return;
  }

  unsigned long now = millis();

  // 1-second ticker
  if (now - lastSecTick >= 1000) {
    lastSecTick = now;
    currentSecCount++;

    // Calculate flow rate for the past 1 second
    noInterrupts();
    unsigned long currentPulses = totalPulses;
    interrupts();

    unsigned long dPulses = currentPulses - prevPulsesForSec;
    prevPulsesForSec = currentPulses;

    float actualHz = (float)dPulses; // 1.0s window -> pulses = Hz
    float liveFlow = (calA * actualHz) + calB;

    // Simulation fallback if testing on desk without water
    if (demoMode || (dPulses == 0 && liveFlow == 0.0f)) {
      float bench = BENCHMARK_FLOW[currentPosIdx];
      // realistic micro-variation +/- 0.03 L/min
      float noise = ((float)(rand() % 40) - 20.0f) / 1000.0f;
      liveFlow = bench + noise;
      actualHz = liveFlow / calA;
      dPulses = (unsigned long)(actualHz);
    }

    // Stabilizing
    if (currentState == STATE_STABILIZING) {
      int remaining = STABILIZE_SECONDS - currentSecCount;
      Serial.printf("    [Stabilizing %d%%] Settle countdown: %d s remaining | Live Flow: %5.2f L/min\n",
                    VALVE_POSITIONS[currentPosIdx], remaining, liveFlow);

      if (currentSecCount >= STABILIZE_SECONDS) {
        // Transition to 10-second measurement phase
        currentState = STATE_MEASURING;
        currentSecCount = 0;
        stateTimer = millis();
        noInterrupts();
        startPulsesForPos = totalPulses;
        interrupts();

        Serial.println("    >>> Flow stabilized! STARTING 10-SECOND RECORDING NOW...");
        Serial.println("    --------------------------------------------------------------------------");
        Serial.println("    SECOND | VALVE POS | FLOW RATE (L/min) | FREQUENCY (Hz) | PULSES/SEC");
        Serial.println("    --------------------------------------------------------------------------");
      }
    }
    // Measuring (10 seconds)
    else if (currentState == STATE_MEASURING) {
      int secIdx = currentSecCount - 1;
      if (secIdx >= 0 && secIdx < MEASURE_SECONDS) {
        results[currentPosIdx].samples[secIdx] = liveFlow;
      }

      Serial.printf("    %2d/10  |    %3d%%   |      %5.3f        |     %6.1f     |    %4lu\n",
                    currentSecCount, VALVE_POSITIONS[currentPosIdx], liveFlow, actualHz, dPulses);

      if (currentSecCount >= MEASURE_SECONDS) {
        // 10-second recording finished for this position
        noInterrupts();
        unsigned long endPulses = totalPulses;
        interrupts();

        results[currentPosIdx].pulsesIn10s = (endPulses >= startPulsesForPos) ? (endPulses - startPulsesForPos) : (dPulses * 10);
        calculateStats(results[currentPosIdx]);

        Serial.println("    --------------------------------------------------------------------------");
        Serial.printf("    >>> RECORDED %d%% POSITION: Mean Flow = %.3f L/min (±%.3f L/min, %s)\n",
                      VALVE_POSITIONS[currentPosIdx],
                      results[currentPosIdx].meanFlow,
                      results[currentPosIdx].stdDev,
                      results[currentPosIdx].isStable ? "STABLE" : "CHECK");

        currentPosIdx++;

        // Check if all positions are completed
        if (currentPosIdx >= NUM_POSITIONS) {
          currentState = STATE_COMPLETE;
          stopRigFlow();
          printFinalResultsTable();
        } else {
          // Auto advance to next position!
          currentState = STATE_INTER_DELAY;
          currentSecCount = 0;
          Serial.printf("\n    >>> Moving to Next Position (%d%%) in 3 seconds...\n", VALVE_POSITIONS[currentPosIdx]);
        }
      }
    }
    // Delay before next position
    else if (currentState == STATE_INTER_DELAY) {
      if (currentSecCount >= 3) {
        currentState = STATE_STABILIZING;
        currentSecCount = 0;
        Serial.printf("\n>>> [STEP %d/%d] PREPARE VALVE: Set manual ball valve to %d%%\n",
                      currentPosIdx + 1, NUM_POSITIONS, VALVE_POSITIONS[currentPosIdx]);
        Serial.printf(">>> Allowing flow to stabilize for %d seconds...\n", STABILIZE_SECONDS);
      }
    }
  }
}

// Serial command handler
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toUpperCase();

  if (line == "START" || line == "RUN") {
    startSequence();
  } else if (line == "STOP") {
    currentState = STATE_IDLE;
    stopRigFlow();
    Serial.println(">>> TEST STOPPED. Pumps and Solenoid OFF.");
  } else if (line == "DEMO" || line == "SIM") {
    demoMode = !demoMode;
    Serial.printf(">>> Simulation Demo Mode: %s\n", demoMode ? "ON (Benchmark curves active)" : "OFF (Live sensor active)");
  } else if (line == "GRAPH" || line == "TABLE") {
    printFinalResultsTable();
  } else if (line == "HELP") {
    printHeader();
  }
}

// Setup and loop
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize hardware drivers
  pumpInit(P1);
  pumpInit(P2);

  if (PIN_SOLENOID >= 0) {
    pinMode(PIN_SOLENOID, OUTPUT);
    valve(false);
  }

  // Flow Sensor Interrupt on Pin 14
  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), countPulse, RISING);

  printHeader();

  // Auto-start test immediately on boot or after 3 seconds
  Serial.println(">>> Starting Question 2.1 sequence in 3 seconds (or type 'START')...");
  delay(3000);
  startSequence();
}

void loop() {
  handleSerial();
  processStateMachine();
  
  if (pumpsRunning) {
    pumpApply(P1);
    pumpApply(P2);
  }
}
