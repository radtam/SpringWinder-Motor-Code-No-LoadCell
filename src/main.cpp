/*
  Motor ESP32
  ESP32 Dual-Core Stepper + HX711 Test Rig
  - Core 0: High-priority stepper loop (AccelStepper::run())
  - Core 1: Control loop (HX711 reading, serial commands, printing, state machine)

  Libraries:
    AccelStepper
    HX711

*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <HX711_MP.h>
#include <vector>
#include <Preferences.h>

// ----------------------- USER PINS (EDIT) -----------------------
static const int STEP_PIN    = 13;  //14 
static const int DIR_PIN     = 12;  //13
static const int ENABLE_PIN  = 14;  //12  set -1 if not used (LOW = enabled)

// Display Communication
constexpr int LINK_RX_PIN = 16;   // receives from display TX
constexpr int LINK_TX_PIN = 17;   // optional, for replies
#define LINK_UART Serial2

// ----------------------- DEFAULT MOTOR CONFIG -------------------------
struct Config {        // The varibles under Config are being saved to NVS 
  // Motion
  float maxSpeed_sps   = 6400.0f;   // steps/second
  float accel_sps2     = 8000.0f;    // steps/second^2
  long  posA_steps     = 0;         // first endpoint
  long  posB_steps     = -6400;      // second endpoint
  uint32_t dwell_ms    = 5000;       // pause at endpoints
  uint32_t cycles_goal = 10;        // number of A->B->A cycles

  // Convenience
  float steps_per_mm   = 1600.0f;    // if you want to think in mm, trying to use it as steprs per rotation sometimes
} cfg;

// ----------------------- PERSISTENT STORAGE (NVS / "EEPROM") -----------------------
Preferences prefs;
// bump these if you change Config layout later
static const uint32_t CFG_MAGIC   = 0xC0FFEE01;
static const uint16_t CFG_VERSION = 1;
struct PersistHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t length;   // bytes of payload that follow
};
bool saveConfig();
bool loadConfig();
bool wipeConfig();

// ----------------------- MODES & STATES -------------------------
enum class Mode : uint8_t { SETUP=0, LOADCELL=1, LIVETEST=2 };
enum class RunState : uint8_t { IDLE=0, RUNNING=1, PAUSED=2, STOPPED=3 };

volatile Mode currentMode = Mode::SETUP;
volatile RunState runState = RunState::IDLE;

// Live test progress
volatile uint32_t cycles_completed = 0;
volatile bool atA = true;    // last “anchor” side we consider as start of cycle

#define LC_INDEX_SIZE 10
// Matrix to store values [user_input, load_cell_reading]
std::vector<std::vector<float>> CalibValuesMatrix;

// ----------------------- DEVICES -------------------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
HX711_MP lc = HX711_MP(LC_INDEX_SIZE);

// ----------------------- RTOS / SYNC ---------------------------
TaskHandle_t taskStepperHandle = nullptr;
TaskHandle_t taskControlHandle = nullptr;

SemaphoreHandle_t cfgMutex;     // protect cfg and shared state

// Timing
volatile uint32_t dwell_until_ms = 0; // used only by stepper task
volatile bool in_dwell = false;

// ----------------------- HELPERS -------------------------------
void enableMotor(bool en) {
  if (ENABLE_PIN >= 0) {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, en ? LOW : HIGH); // assume LOW = enable
  }
}

void lockCfg()   { xSemaphoreTake(cfgMutex, portMAX_DELAY); }
void unlockCfg() { xSemaphoreGive(cfgMutex); }


// ----------------------- NVS SAVE/LOAD --------------------------
bool saveConfig()
{
  lockCfg();
  PersistHeader h;
  h.magic   = CFG_MAGIC;
  h.version = CFG_VERSION;
  h.length  = sizeof(Config);
  prefs.begin("motor", false); // RW namespace "motor"
  bool ok1 = (prefs.putBytes("hdr", &h, sizeof(h)) == sizeof(h));
  bool ok2 = (prefs.putBytes("cfg", &cfg, sizeof(cfg)) == sizeof(cfg));
  prefs.end();
  unlockCfg();
  return ok1 && ok2;
}
bool loadConfig()
{
  prefs.begin("motor", true); // RO
  PersistHeader h;
  size_t n = prefs.getBytes("hdr", &h, sizeof(h));
  if (n != sizeof(h) || h.magic != CFG_MAGIC || h.version != CFG_VERSION || h.length != sizeof(Config)) {
    prefs.end();
    return false; // not found or incompatible
  }
  Config tmp;
  n = prefs.getBytes("cfg", &tmp, sizeof(tmp));
  prefs.end();
  if (n != sizeof(tmp)) return false;
  // Apply atomically
  lockCfg();
  cfg = tmp;
  stepper.setMaxSpeed(cfg.maxSpeed_sps);
  stepper.setAcceleration(cfg.accel_sps2);
  unlockCfg();
  return true;
}
bool wipeConfig()
{
  prefs.begin("motor", false);
  bool ok = prefs.clear(); // clears ONLY the "motor" namespace
  prefs.end();
  return ok;
}



String modeToStr(Mode m) {
  switch (m) {
    case Mode::SETUP:    return "SETUP";
    case Mode::LOADCELL: return "LOADCELL";
    case Mode::LIVETEST: return "LIVETEST";
  }
  return "?";
}
String stateToStr(RunState s) {
  switch (s) {
    case RunState::IDLE:    return "IDLE";
    case RunState::RUNNING: return "RUNNING";
    case RunState::PAUSED:  return "PAUSED";
    case RunState::STOPPED: return "STOPPED";
  }
  return "?";
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  status"));
  Serial.println(F("  mode setup|load|live"));
  Serial.println(F("  start | pause | stop"));
  Serial.println(F("  set speed <steps_per_sec>"));
  Serial.println(F("  set accel <steps_per_sec2>"));
  Serial.println(F("  set posa <steps>"));
  Serial.println(F("  set posb <steps>"));
  Serial.println(F("  set cycles <N>"));
  Serial.println(F("  set dwell <ms>"));
  Serial.println(F("  set spmm <steps_per_mm>"));
  Serial.println(F("  go mm <pos_mm>     (SETUP mode convenience)"));
  Serial.println(F("  tare"));
  Serial.println(F("  cal weight <weight>"));
  Serial.println(F("  lc preset"));
  Serial.println(F("  rate <Hz>          (print & read rate)"));
  Serial.println(F("  save        - save cfg settings to NVS"));
  Serial.println(F("  load        - load cfg settings from NVS"));
  Serial.println(F("  wipe        - erase saved settings (motor namespace)"));
}

void printStatus() {
  lockCfg();
  Serial.println(F("---- STATUS ----"));
  Serial.printf("Mode: %s   State: %s\n", modeToStr(currentMode).c_str(), stateToStr(runState).c_str());
  Serial.printf("A: %ld  B: %ld  (steps)\n", cfg.posA_steps, cfg.posB_steps);
  Serial.printf("Speed: %.2f sps   Accel: %.2f sps^2\n", cfg.maxSpeed_sps, cfg.accel_sps2);
  Serial.printf("Cycles: %lu (goal)   Completed: %lu\n", (unsigned long)cfg.cycles_goal, (unsigned long)cycles_completed);
  Serial.printf("Dwell: %lu ms\n", (unsigned long)cfg.dwell_ms);
  Serial.printf("Steps/mm: %.3f\n", cfg.steps_per_mm);
  Serial.printf("Motor pos: %ld (steps)\n", stepper.currentPosition());
  unlockCfg();
}

// Simple line reader
bool readLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        out = buf;
        buf = "";
        return true;
      }
    } else {
      buf += c;
    }
  }
  return false;
}

bool readLink(String &out) {
  static String buf;
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;           // ignore CR
    if (c == '\n') {                    // newline ends a message
      if (buf.length() > 0) {
        out = buf;
        buf = "";
        return true;
      }
    } else {
      buf += c;
      if (buf.length() > 256) {         // avoid runaway buffers
        buf = "";                       // flush on overflow
      }
    }
  }
  return false;
}

//------------------------SET MATRIX & CALIBRATION ---------------


// ----------------------- COMMAND PARSER ------------------------
void handleCommand(const String &line) {
  String s = line;
  s.trim();
  s.toLowerCase();

  if (s == "help") { printHelp(); return; }
  if (s == "status") { printStatus(); return; }

  if (s.startsWith("mode ")) {
    if (s.endsWith("setup")) {
      currentMode = Mode::SETUP;
      runState = RunState::IDLE;
      Serial.println(F("Mode -> SETUP"));
    } else if (s.endsWith("load")) {
      currentMode = Mode::LOADCELL;
      runState = RunState::IDLE;
      Serial.println(F("Mode -> LOADCELL"));
    } else if (s.endsWith("live")) {
      currentMode = Mode::LIVETEST;
      Serial.println(F("Mode -> LIVETEST (use 'start' to begin)"));
    } else {
      Serial.println(F("Unknown mode."));
    }
    return;
  }

  if (s == "start") {
    if (currentMode != Mode::LIVETEST) {
      Serial.println(F("Switch to 'mode live' first."));
      return;
    }
    cycles_completed = 0;
    runState = RunState::RUNNING;
    atA = true; // treat starting at A
    Serial.println(F("LIVE TEST: START"));
    return;
  }
  if (s == "pause") {
    if (runState == RunState::RUNNING) {
      runState = RunState::PAUSED;
      Serial.println(F("LIVE TEST: PAUSE requested"));
    } else {
      Serial.println(F("Not running."));
    }
    return;
  }
  if (s == "stop") {
    stepper.stop();
    runState = RunState::STOPPED;
    Serial.println(F("LIVE TEST: STOP"));
    return;
  }

  if (s.startsWith("set ")) {
    // tokens
    // set speed <val>, set accel <val>, set posa <val>, set posb <val>, set cycles <N>, set dwell <ms>, set spmm <val>
    String key = s.substring(4);
    int sp = key.indexOf(' ');
    if (sp < 0) { Serial.println(F("Bad set command. Try 'help'.")); return; }
    String what = key.substring(0, sp);
    String val  = key.substring(sp + 1);

    lockCfg();
    if (what == "speed") {
      cfg.maxSpeed_sps = val.toFloat();
      stepper.setMaxSpeed(cfg.maxSpeed_sps);
      Serial.printf("Speed -> %.2f sps\n", cfg.maxSpeed_sps);
    } else if (what == "accel") {
      cfg.accel_sps2 = val.toFloat();
      stepper.setAcceleration(cfg.accel_sps2);
      Serial.printf("Accel -> %.2f sps^2\n", cfg.accel_sps2);
    } else if (what == "posa") {
      cfg.posA_steps = val.toInt();
      Serial.printf("PosA -> %ld steps\n", cfg.posA_steps);
    } else if (what == "posb") {
      cfg.posB_steps = val.toInt();
      Serial.printf("PosB -> %ld steps\n", cfg.posB_steps);
    } else if (what == "cycles") {
      cfg.cycles_goal = (uint32_t)val.toInt();
      Serial.printf("Cycles -> %lu\n", (unsigned long)cfg.cycles_goal);
    } else if (what == "dwell") {
      cfg.dwell_ms = (uint32_t)val.toInt();
      Serial.printf("Dwell -> %lu ms\n", (unsigned long)cfg.dwell_ms);
    } else if (what == "spmm") {
      cfg.steps_per_mm = val.toFloat();
      Serial.printf("Steps/mm -> %.3f\n", cfg.steps_per_mm);
    } else {
      Serial.println(F("Unknown 'set' field."));
    }
    unlockCfg();
    return;
  }

  if (s.startsWith("go mm ")) {
    if (currentMode != Mode::SETUP) {
      Serial.println(F("Use 'go mm' only in SETUP mode."));
      return;
    }
    float mm = s.substring(6).toFloat();
    long tgt = lroundf(mm * cfg.steps_per_mm);
    long curr = stepper.currentPosition();
    lockCfg();
    stepper.setMaxSpeed(6400);
    stepper.setAcceleration(8000);
    stepper.moveTo(curr+tgt);
    unlockCfg();
    //stepper.move(tgt);
    
    //stepper.setMaxSpeed(cfg.maxSpeed_sps);
    //stepper.moveTo(stepper.currentPosition()+tgt);
    Serial.printf("Moving to ~%0.3f mm\n", mm);
    return;
  }
  
  if (s.startsWith("rate ")) {
    uint16_t hz = (uint16_t)s.substring(5).toInt();
    if (hz < 1) hz = 1;
    if (hz > 200) hz = 200;
    lockCfg();
    unlockCfg();
    Serial.printf("Print/read rate -> %u Hz\n", hz);
    return;
  }

  if (s == "save") {
    if (saveConfig()) {
      Serial.println(F("Configuration saved to NVS."));
    } else {
      Serial.println(F("Failed to save configuration."));
    }
    return;
  }
  if (s == "load") {
    if (loadConfig()) {
      Serial.println(F("Configuration loaded from NVS."));
    } else {
      Serial.println(F("Failed to load configuration."));
    }
    return;
  }
  if (s == "wipe") {
    if (wipeConfig()) {
      Serial.println(F("Configuration wiped from NVS."));
    } else {
      Serial.println(F("Failed to wipe configuration."));
    }
    return;
  }

  Serial.println(F("Unknown command. Type 'help'."));
}

// ----------------------- LIVE TEST LOGIC ------------------------
void setNextTarget_FromAtoB(bool toB) {
  lockCfg();
  long tgt = toB ? cfg.posB_steps : cfg.posA_steps;
  stepper.moveTo(tgt);
  unlockCfg();
}

bool atTarget() {
  return stepper.distanceToGo() == 0;
}

// ----------------------- TASK: STEPPER (Core 0) -----------------
void taskStepper(void *pv) {
    // Allow idle to run periodically to keep task watchdog happy
  uint32_t lastIdleGiveMs = millis();
  // High-priority loop: DO NOT print here.
  for (;;) {
    RunState localState = runState;
    Mode     localMode  = currentMode;

    // Always keep AccelStepper ticking to avoid missed steps
    stepper.run();

    if (localMode == Mode::LIVETEST) {
      if (localState == RunState::RUNNING) {
        // If in dwell, wait out dwell (without calling run? -> motor is stopped).
        if (in_dwell) {
          if (millis() >= dwell_until_ms) {
            in_dwell = false;
            // leave dwell: decide next move
            // If we just reached A, go to B; if B, go to A
            bool goToB = atA; // after reaching A, head to B; after reaching B, head to A
            setNextTarget_FromAtoB(goToB);
          } 
        } else {
          // Not in dwell, check if reached the target
          if (atTarget()) {
            // We hit an endpoint
            long pos = stepper.currentPosition();
            lockCfg();
            long a = cfg.posA_steps;
            long b = cfg.posB_steps;
            uint32_t dwellms = cfg.dwell_ms;
            unlockCfg();

            if (labs(pos - a) < labs(pos - b)) {
              // At side A
              if (!atA) {
                // Completed a B->A half. If we define cycle as A->B->A, then arriving at A completes a cycle.
                atA = true;
                cycles_completed++;
              }
            } else {
              // At side B
              atA = false;
            }

            // Check cycles done
            if ((uint32_t)cycles_completed >= cfg.cycles_goal) {
              runState = RunState::IDLE;
              // Stop gently
              stepper.stop();
              // Ensure target = current
              stepper.moveTo(stepper.currentPosition());
            } else {
              // Start dwell
              in_dwell = true;
              dwell_until_ms = millis() + dwellms;
            }
          }
        }
      } else if (localState == RunState::PAUSED) {
        // Decelerate to stop and hold
        stepper.stop(); // plan to stop smoothly
        // Keep run() ticking until we actually reach stop
      } else if (localState == RunState::STOPPED) {
        // Cancel everything and hold current
        stepper.stop();
        stepper.moveTo(stepper.currentPosition());
        in_dwell = false;
        runState = RunState::IDLE;
      } 
    } 
    // ---- Watchdog-friendly yielding strategy ----
    // Give the idle task one tick every ~100 ms (tunable) so the T-WDT never trips.
    uint32_t nowMs = millis();
    if (nowMs - lastIdleGiveMs >= 100) {
      vTaskDelay(1);              // lets lower-priority (idle) run for 1 tick
      lastIdleGiveMs = nowMs;
    } else {
      taskYIELD();                // micro-yield between stepper.run() calls
    }

    // When not actively running a live test, we can afford to block more.
    if (currentMode != Mode::LIVETEST || runState != RunState::RUNNING) {
      vTaskDelay(1);
    }
  }
}

// ----------------------- TASK: CONTROL (Core 1) -----------------
void taskControl(void *pv) {
  uint32_t lastPrint = 0;

  for (;;) {

    // 1) Read Serial
    String line;
    if (readLine(line)) {
      handleCommand(line);
    }

    // 1.5) Read Tx/Rx link
    String linkLine;
    if(readLink(linkLine)){
      handleCommand(linkLine);
    }

    uint32_t now = millis();

      // Position
      long pos = stepper.currentPosition();
      

      if (currentMode == Mode::LOADCELL) {
        Serial.printf("[LOADCELL] t=%lu ms, pos=%ld\n", (unsigned long)now, pos);
      } else if (currentMode == Mode::LIVETEST) {
        // Stream during motion too
        Serial.printf("[LIVE] t=%lu ms, cyc=%lu/%lu, state=%s, %ld\n",
                      (unsigned long)now,
                      (unsigned long)cycles_completed, (unsigned long)cfg.cycles_goal,
                      stateToStr(runState).c_str(),
                      pos);
           
      } 
    }

    // 3) START handling (one-time edge when RUNNING begins)
    static bool armed = false;
    if (currentMode == Mode::LIVETEST) {
      if (runState == RunState::RUNNING && !armed) {
        // Initialize first leg: A -> B
        atA = true;
        in_dwell = false;
        setNextTarget_FromAtoB(true /*toB*/);
        armed = true;
      } else if (runState != RunState::RUNNING) {
        armed = false;
      }
    } else {
      armed = false;
    }

    vTaskDelay(1); // tiny breather for control core
}

// ----------------------- SETUP -------------------------------
void setup() {
  Serial.begin(115200);
  LINK_UART.begin(115200, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
 
  while (!Serial) { delay(10); }

  Serial.println(F("\nESP32 Stepper + HX711 Dual-Core Test Rig"));
  Serial.println(F("Type 'help' for commands."));

  // Mutex
  cfgMutex = xSemaphoreCreateMutex();

 // Load saved settings (if present)
  if (loadConfig()) {
    Serial.println(F("Loaded cfg from NVS."));
  } else {
    Serial.println(F("No saved cfg found (using defaults)."));
  }

  // Stepper
  if (ENABLE_PIN >= 0) {
    pinMode(ENABLE_PIN, OUTPUT);
    enableMotor(true);
  }
  
  stepper.setMinPulseWidth(20);
  stepper.setMaxSpeed(cfg.maxSpeed_sps);
  stepper.setAcceleration(cfg.accel_sps2);
  stepper.setCurrentPosition(0);

  // HX711
  // lc.begin(LC_DT_PIN, LC_SCK_PIN, false);
  // lc.reset();
  
  // Initialize a 10x2 matrix filled with 0.0
  CalibValuesMatrix.resize(LC_INDEX_SIZE, std::vector<float>(2, 0.0f));
  delay(200);
 
  printHelp();
  printStatus();
  // Tasks
  // Core 0: stepper (higher priority)
  xTaskCreatePinnedToCore(taskStepper, "taskStepper", 4096, nullptr, 5, &taskStepperHandle, 0);

  // Core 1: control (serial, hx711, prints)
  xTaskCreatePinnedToCore(taskControl, "taskControl", 8192, nullptr, 3, &taskControlHandle, 1);

  printHelp();
  printStatus();
}

void loop() {
  // Unused. All work in tasks.
}
