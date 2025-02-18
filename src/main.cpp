#include <DebugBuilder.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <RestCommandHandler.h>
#include <SerialCommandHandler.h>
#include <Servo.h>
#include <secrets.h>
#include <map>
#include <set>

enum AppState {
  EMERGENCY_STOPPED,
  STOPPED,
  AUTO_CALIBRATING,
  CALIBRATING,
  MOVING_TO_TARGET,
  MOVING_TO_TARGET_SLOW,
};

// Prototypes declaration
int getCurrentPulses();
int getCurrentPanel();
int getTargetPulses();
float calculateSpeedMovingToTarget();
int getRemainingPulses();
void setMotorSpeed(float normalizedSpeed);
void setTargetPanel(int panel);
int getTargetPanel();
void emergencyStop(String message);
String stateToString(AppState state);
void setCurrentState(AppState newState);
void saveDefaultPulse();
void loadDefaultPulse();
int getDefaultPanel();
int getDefaultPulseOffset();
int getCurrentRpm();
float normalizeSpeed(int speed);
int denormalizeSpeed(float normalizedSpeed);
int computePulsesForwardDistance(int from, int to);
int computePulsesMinDistance(int from, int to);
int computeRpm(int lastPulses, int currentPulses, unsigned long lastTime, unsigned long currentTime);

#define DEBUG_ENABLED

// Pins configuration
#define SERVO_PIN D1
#define ENCODER_PIN_A D6       // GREEN
#define ENCODER_PIN_B D7       // WHITE
#define OPTICAL_SENSOR_PIN D4  // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;
const int PULSES_PER_PANEL = 24;                           // ENCODER_RESOLUTION / ENCODER_GEAR_TEETH * ENCODER_PULSES_PER_STEP = 360 / 60 * 4 = 24
const int PULSES_COUNT = PANELS_COUNT * PULSES_PER_PANEL;  // Nombre total d'impulsions
const int ENCODER_DIRECTION_SIGN = 1;
const int TARGET_PULSE_OFFSET = 12;  // When setting a target panel, use the nth pulse of this panel for centering
static_assert(TARGET_PULSE_OFFSET >= 0 && TARGET_PULSE_OFFSET < PULSES_PER_PANEL, "OFFSET must be in range [0-PULSES_PER_PANEL]");

// Globals
AsyncWebServer server(80);
AsyncCorsMiddleware cors;
AsyncWebSocket ws("/panel");

bool errorFlag = false;    // Emergency stop flag
String errorMessage = "";  // Emergency stop message
volatile AppState currentState = STOPPED;
volatile bool calibrated = false;
SerialCommandHandler serialCommandHandler;
RestCommandHandler restCommandHandler(server);

// Encoder
int defaultPulse = 0;
int targetPulses = 0;
volatile int encoderPulsesRaw = 0;
const int8_t ENCODER_STATE_TABLE[16] = {0, 1, -1, -0, -1, 0, -0, 1, 1, -0, 0, -1, -0, -1, 1, 0};  // Encoder state table for natural debouncing (-0 are non valid states)

// Optical sensor
const int OPTICAL_DETECTED_EDGE = RISING;  // Capteur à 1 quand coupé / 0 quand trou / ralentit quand trou, et calibre sur rising vers coupé
volatile bool opticalState = HIGH;         // Assume not on slot

// Servo
Servo servo;
const int STOP_SPEED = 90;  // Valeur pour arrêter le servo
const int RUN_SPEED = 140;  // Vitesse du servo pour avancer (91-180) 140 c'est bien (met 6 secondes à faire un tour)
static_assert(RUN_SPEED > 90, "RUN_SPEED must be greater than 90 or everything will break !");
const unsigned long BLOCKAGE_TIMEOUT = 250;          // Timeout (ms) between blockage detection checks
const unsigned long BLOCKAGE_TIMEOUT_WARMUP = 1000;  // Time to wait before starting blockage detection when motor is starting //TODO revoir ces valeurs et celui-ci pourrait être moins
const int BLOCKAGE_RPM_AT_MAX_SPEED = 10;            // Expected RPM (panels) when servo is at max speed
const float BLOCKAGE_RPM_TOLERANCE = 0.90;           // Tolerance for blockage detection speed
float blockageActualRPM = 0.0;
float blockageExpectedRPM = 0.0;

#ifdef DEBUG_ENABLED
volatile int encoderInterruptCallCount = 0;
volatile int opticalDetectedEdgesCount = 0;
#endif

std::vector<DebugField> debugFields = {
    {"currentPanel", false, [] { return getCurrentPanel(); }},
    {"currentPulses", true, [] { return getCurrentPulses(); }},
    {"targetPanel", false, [] { return getTargetPanel(); }},
    {"targetPulses", true, [] { return getTargetPulses(); }},
    {"optical", true, [] { return opticalState; }},
    {"calibrated", true, [] { return calibrated; }},
    {"dist", false, [] { return getRemainingPulses(); }},
    {"servo", true, [] { return servo.read(); }},  // TODO à voir parfois une valeur ne change pas mais le hash est différent (se produit dans les états ou le servo tourne)
    {"dfltPulse", true, [] { return defaultPulse; }},
    {"dfltPanel", false, [] { return getDefaultPanel(); }},
    {"dfltPulseOffset", false, [] { return getDefaultPulseOffset(); }},
#ifdef DEBUG_ENABLED
    {"rpm", false, [] { return getCurrentRpm(); }},
    {"bActRpm", false, [] { return blockageActualRPM; }},
    {"bExpRpm", false, [] { return blockageExpectedRPM; }},
    {"encPulsesRaw", false, [] { return encoderPulsesRaw; }},
    {"optEdgeCount", true, [] { return opticalDetectedEdgesCount; }},
    {"encIntCount", false, [] { return encoderInterruptCallCount; }},
#endif
    {"state", true, [] { return stateToString(currentState); }},
    {"errorMessage", true, [] { return errorMessage; }},
};

DebugBuilder debugBuilder(debugFields);

void assertError(bool condition, const std::function<String()>& messageBuilder) {
  if (!condition) {
    emergencyStop(messageBuilder());
  }
}

void assertWarn(bool condition, const std::function<String()>& messageBuilder) {
  if (!condition) {
    errorMessage = messageBuilder();
  }
}

void emergencyStop(String message) {
  servo.write(STOP_SPEED);  // First things first, stop the motor
  errorFlag = true;         // Error flag will prevent the motor to run, until manually reset
  setCurrentState(EMERGENCY_STOPPED);
  errorMessage = message;
}

int IRAM_ATTR getCurrentPulses() {
  return (encoderPulsesRaw + defaultPulse) % PULSES_COUNT;
}

void setTargetPulses(int pulses) {
  assertError(pulses >= 0 && pulses < PULSES_COUNT, [pulses] { return "pulses " + String(pulses) + " out of bounds"; });
  targetPulses = pulses;
}

int getTargetPulses() {
  return targetPulses;
}

int getCurrentPanel() {
  return getCurrentPulses() / PULSES_PER_PANEL;
}

int getTargetPanel() {
  return targetPulses / PULSES_PER_PANEL;
}

void setTargetPanel(int panel) {
  assertError(panel < PANELS_COUNT, [panel] { return "panel " + String(panel) + " > " + PANELS_COUNT; });
  setTargetPulses((panel * PULSES_PER_PANEL) + TARGET_PULSE_OFFSET);
}

int getDefaultPanel() {
  return defaultPulse / PULSES_PER_PANEL;
}

int getDefaultPulseOffset() {
  return defaultPulse % PULSES_PER_PANEL;
}

String stateToString(AppState state) {
  switch (state) {
    case EMERGENCY_STOPPED:
      return "EMERGENCY_STOPPED";
    case STOPPED:
      return "STOPPED";
    case CALIBRATING:
      return "CALIBRATING";
    case AUTO_CALIBRATING:
      return "AUTO_CALIBRATING";
    case MOVING_TO_TARGET:
      return "MOVING_TO_TARGET";
    case MOVING_TO_TARGET_SLOW:
      return "MOVING_TO_TARGET_SLOW";
    default:
      emergencyStop("Unknown app state: " + String((int)state));
      return "UNKNOWN_STATE";
  }
}

void setCurrentState(AppState newState) {
  currentState = newState;
}

void saveDefaultPulse() {
  EEPROM.write(0, defaultPulse);
  EEPROM.commit();
}

void loadDefaultPulse() {
  defaultPulse = EEPROM.read(0);
}

int getCurrentRpm() {
  static int lastPulses = 0;
  static unsigned long lastTime = 0;
  int currentPulses = getCurrentPulses();
  unsigned long currentTime = millis();
  int rpm = computeRpm(lastPulses, currentPulses, lastTime, currentTime);
  lastPulses = currentPulses;
  lastTime = currentTime;
  return rpm;
}

int computeRpm(int lastPulses, int currentPulses, unsigned long lastTime, unsigned long currentTime) {
  unsigned long elapsedTime = currentTime - lastTime;
  if (elapsedTime == 0)
    return 0;  // Avoid divide by 0
  int pulsesDelta = computePulsesForwardDistance(lastPulses, currentPulses);
  return (pulsesDelta * 60000.0) / (elapsedTime * PULSES_COUNT);
}

// Functions

String doGetRestRoutes() {
  return restCommandHandler.getRoutesList();
}

String doGetSerialCommands() {
  return serialCommandHandler.getCommandsList();
}

String doSetDefaultPulse(int pulse) {
  if (pulse < 0 || pulse >= PULSES_COUNT) {
    return "Error: default pulse " + String(pulse) + " out of bounds [0-" + String(PULSES_COUNT) + "]";
  }
  defaultPulse = pulse;
  return "Default pulse set to " + String(defaultPulse) + ". Default panel is " + String(getDefaultPanel()) + " with offset " + String(getDefaultPulseOffset());
}

String doSetDefaultPanelAndOffset(int panel, int offset) {
  if (panel < 0 || panel >= PANELS_COUNT) {
    return "Error: default panel " + String(panel) + " out of bounds [0-" + String(PANELS_COUNT) + "]";
  }
  if (offset < 0 || offset >= PULSES_PER_PANEL) {
    return "Error: default offset " + String(offset) + " out of bounds [0-" + String(PULSES_PER_PANEL) + "]"; //TODO fonction pour checks bounds homogène
  }
  defaultPulse = panel * PULSES_PER_PANEL + offset;
  return "Default pulse set to " + String(defaultPulse) + ". Default panel is " + String(getDefaultPanel()) + " with offset " + String(getDefaultPulseOffset());
}

String doSaveSettings() {
  saveDefaultPulse();
  return "Settings saved !";
}

String doLoadSettings() {
  loadDefaultPulse();
  return "Settings loaded !";
}

String doGetDebug() {
  return debugBuilder.buildJson();
}

String doGetCurrentPanel() {
  return String(getCurrentPanel());
}

String doMoveToPanel(int panel) {
  if (panel > PANELS_COUNT) {
    return "Panel " + String(panel) + " out of bounds [0-" + String(PANELS_COUNT) + "]";
  }
  setTargetPanel(panel);
  setCurrentState(MOVING_TO_TARGET);
  return "Moving to panel " + String(getTargetPanel());
}

String doAdvancePanels(int count) {
  int currentPanel = getCurrentPanel();
  setTargetPanel((currentPanel + count) % PANELS_COUNT);
  setCurrentState(MOVING_TO_TARGET);
  return "From " + String(currentPanel) + ", Advancing " + String(count) + " panels to " + String(getTargetPanel());
}

String doAdvancePulses(int pulseCount) {
  int currentPulses = getCurrentPulses();
  setTargetPulses((currentPulses + pulseCount) % PULSES_COUNT);
  setCurrentState(MOVING_TO_TARGET_SLOW);
  return "Advancing " + String(pulseCount) + " pulses to " + String(targetPulses);
}

String doCalibrate() {
  calibrated = false;
  setTargetPanel(0);  // After on-demand calibration, will go to panel 0
  setCurrentState(CALIBRATING);
  return "Calibration started. Rotating until optical sensor edge is detected, then going to panel 0";
}

String doStop() {
  setMotorSpeed(0);
  setCurrentState(STOPPED);
  setTargetPulses(getCurrentPulses());  // Consider target as reached
  return "Stopped. Current panel set to " + String(getCurrentPanel());
}

String doReset() {
  setMotorSpeed(0);  // Set everything as stopped
  setCurrentState(STOPPED);
  calibrated = false;  // Reset all flags
  errorFlag = false;
  errorMessage = "";
  loadDefaultPulse();  // Load config from EEPROM
  return "Reset";
}

int IRAM_ATTR computePulsesForwardDistance(int from, int to) {
  return (to - from + PULSES_COUNT) % PULSES_COUNT;
}

int IRAM_ATTR computePulsesMinDistance(int from, int to) {
  int distance = computePulsesForwardDistance(from, to);
  return min(distance, PULSES_COUNT - distance);
}

int getRemainingPulses() {
  int currentPulses = getCurrentPulses();
  return computePulsesForwardDistance(currentPulses, targetPulses);
}

bool isTargetPulseReached() {
  return getRemainingPulses() == 0;
}

float calculateSpeedCalibration() {
  if (OPTICAL_DETECTED_EDGE == RISING) {
    return opticalState == LOW ? 0.3f : 1.f;  // Half speed when we are about to detect the 2nd edge
  } else {
    return opticalState == HIGH ? 0.3f : 1.f;
  }
}

float calculateSpeedMovingToTarget() {
  int remainingPulses = getRemainingPulses();
  if (remainingPulses == 0) {
    return 0.f;  // Stop if target is reached (should be already stopped by encoder interrupt)
  } else if (remainingPulses <= PULSES_PER_PANEL) {
    return 0.3f;  // Slow down as we approach target
  } else if (remainingPulses <= PULSES_PER_PANEL * 2) {
    return 0.5f;
  } else if (remainingPulses <= PULSES_PER_PANEL * 3) {
    return 0.7f;
  } else {
    return 1.0f;  // Constant speed accross all panels
  }
}

float calculateSpeedMovingToTargetSlow() {
  return 0.15f;
}

/**
 * Make state transitions based on current state and conditions
 * ! The currentState is the only variable that should be modified in this function !
 */
void evaluateStateTransitions() {
  switch (currentState) {
    case EMERGENCY_STOPPED:
    case STOPPED:
      // Nothing to do, wait for command
      break;
    case CALIBRATING: {
      if (calibrated) {
        setCurrentState(STOPPED);
      }
      break;
    }
    case AUTO_CALIBRATING: {
      if (calibrated) {
        setCurrentState(MOVING_TO_TARGET);
      }
      break;
    } 
    case MOVING_TO_TARGET: {
      if (!calibrated) {
        setCurrentState(AUTO_CALIBRATING);
      }
      if (isTargetPulseReached()) {
        setCurrentState(STOPPED);
      }
      break;
    }
    case MOVING_TO_TARGET_SLOW: {
      if (isTargetPulseReached()) {
        setCurrentState(STOPPED);
      }
      break;
    }
  }
}

void processStateActions() {
  switch (currentState) {
    case EMERGENCY_STOPPED:
    case STOPPED:
      setMotorSpeed(0);
      break;
    case CALIBRATING:
    case AUTO_CALIBRATING: {
      float speed = calculateSpeedCalibration();
      setMotorSpeed(speed);
      break;
    }
    case MOVING_TO_TARGET: {
      float speed = calculateSpeedMovingToTarget();
      setMotorSpeed(speed);
    } break;
    case MOVING_TO_TARGET_SLOW: {
      float speed = calculateSpeedMovingToTargetSlow();
      setMotorSpeed(speed);
      break;
    } 
  }
}

void readSensors() {
  opticalState = digitalRead(OPTICAL_SENSOR_PIN);
}

void connectToWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    setMotorSpeed(0);  // Stop the motor if we are offline
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi " + WiFi.SSID() + "...");
    }
    Serial.println("Connected to WiFi " + WiFi.SSID() + "!");
    Serial.println("IP Address: " + WiFi.localIP().toString() + " RSSI: " + WiFi.RSSI());
  }
}

void checkForRunningErrors() {
  int motorSpeed = servo.read();

  // static unsigned long lastCheckTime = 0;  // TODO marche pas bien détecte faux positifs lors d'accélérations décélérations
  // static int lastEncoderPulses = 0;
  // static unsigned long warmupTimeout = BLOCKAGE_TIMEOUT_WARMUP;

  // if (motorSpeed > STOP_SPEED) {
  //   unsigned long currentTime = millis();                     // Temps actuel
  //   unsigned long elapsedTime = currentTime - lastCheckTime;  // Temps écoulé depuis la dernière vérification

  //   if (elapsedTime > BLOCKAGE_TIMEOUT + warmupTimeout) {
  //     int currentPulses = getCurrentPulses();
  //     blockageExpectedRPM = BLOCKAGE_RPM_AT_MAX_SPEED * normalizeSpeed(motorSpeed) * BLOCKAGE_RPM_TOLERANCE;
  //     blockageActualRPM = computeRpm(lastEncoderPulses, currentPulses, lastCheckTime, currentTime);
  //     if (blockageActualRPM < blockageExpectedRPM) {
  //       emergencyStop("Blockage detected: motorSpeed=" + String(motorSpeed)    //
  //                     + " blockageActualRPM=" + String(blockageActualRPM)      //
  //                     + " blockageExpectedRPM=" + String(blockageExpectedRPM)  //
  //                     + " lastEncoderPulses=" + String(lastEncoderPulses)      //
  //                     + " currentPulses=" + String(currentPulses)              //
  //                     + " elapsedTime=" + String(elapsedTime));
  //     }

  //     // Mettre à jour les valeurs de référence
  //     lastCheckTime = currentTime;
  //     lastEncoderPulses = currentPulses;
  //     warmupTimeout = 0l;  // No warmup before next checks
  //   }
  // } else {
  //   // Réinitialise les valeurs quand le moteur est à l’arrêt
  //   lastCheckTime = millis();
  //   lastEncoderPulses = getCurrentPulses();
  //   warmupTimeout = BLOCKAGE_TIMEOUT_WARMUP;  // Set warmup for next motor start before checks
  // }

  // Detect motor speed out of bounds
  if (motorSpeed < 90) {
    emergencyStop("Motor speed should not be lower than 90");
  } else if (motorSpeed > 180) {
    emergencyStop("Motor speed should not be greater than 180");
  }
}

void IRAM_ATTR handleEncoderInterrupt() {  // 4us

  // Read encoder pins and update the pulses count
  static uint8_t encoderState = 0;
  byte pinA = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> ENCODER_PIN_A) & 1;
  byte pinB = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> ENCODER_PIN_B) & 1;
  encoderState = ((encoderState << 2) | (pinA << 1) | pinB) & 15;  // Décale et masque les bits
  int8_t pulseInc = ENCODER_STATE_TABLE[encoderState] * ENCODER_DIRECTION_SIGN;
  encoderPulsesRaw += pulseInc;
#ifdef DEBUG_ENABLED
  encoderInterruptCallCount++;
#endif

  // Stop here if the encoder did not move forward
  if (pulseInc < 1)
    return;

  // Check if the optical sensor detected the edge
  // If so, zero the encoder + consider calibration as done
  static bool opticalLastState = HIGH;  // Initialize to HIGH (not detected)
  opticalState = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> OPTICAL_SENSOR_PIN) & 1;
  if ((OPTICAL_DETECTED_EDGE == RISING && opticalLastState == LOW && opticalState == HIGH) ||
      (OPTICAL_DETECTED_EDGE == FALLING && opticalLastState == HIGH && opticalState == LOW)) {
    assertWarn(!calibrated || computePulsesMinDistance(0, encoderPulsesRaw) <= 1, [] {
      return "Missing steps... Optical edge detected at pulse " + String(getCurrentPulses()) + " instead of " + String(defaultPulse);
    });
    encoderPulsesRaw = 0;
    calibrated = true;
#ifdef DEBUG_ENABLED
    opticalDetectedEdgesCount++;
#endif
  }
  opticalLastState = opticalState;

  // Check if the target is reached
  // Note: cannot be done in main loop because the motor can overshoot the target before the loop is executed
  if (currentState == MOVING_TO_TARGET && targetPulses == getCurrentPulses()) {
    currentState = STOPPED;
  }
}

void notifyPanelChanges() {
  static int lastPanel = -1;
  int currentPanel = getCurrentPanel();
  if (currentPanel != lastPanel) {
    ws.textAll(String(currentPanel));
    lastPanel = currentPanel;
  }
}

void setup() {
  assert(!WiFi.getPersistent());
  Serial.begin(115200);
  Serial.setTimeout(10);  // TODO ça sert encore ça ?

  // Register Serial commands
  serialCommandHandler.registerCommand("stop", doStop);
  serialCommandHandler.registerCommand("reset", doReset);
  serialCommandHandler.registerCommand("calibrate", doCalibrate);
  serialCommandHandler.registerCommand("debug", doGetDebug);
  serialCommandHandler.registerCommand<int>("moveToPanel", {"panel"}, doMoveToPanel);
  serialCommandHandler.registerCommand<int>("advancePanels", {"count"}, doAdvancePanels);
  serialCommandHandler.registerCommand<int>("advancePulses", {"count"}, doAdvancePulses);
  serialCommandHandler.registerCommand("panel", doGetCurrentPanel);
  serialCommandHandler.registerCommand("saveSettings", doSaveSettings);
  serialCommandHandler.registerCommand("loadSettings", doLoadSettings);
  serialCommandHandler.registerCommand<int>("setDefaultPulse", {"pulse"}, doSetDefaultPulse);
  serialCommandHandler.registerCommand("help", doGetSerialCommands);
#ifdef DEBUG_ENABLED
  serialCommandHandler.registerCommand("incEncoder", [] { encoderPulsesRaw++; return ""; });
  serialCommandHandler.registerCommand("decEncoder", [] { encoderPulsesRaw--; return ""; });
#endif

  // Register REST API routes
  restCommandHandler.registerCommand("stop", HTTP_GET, doStop);
  restCommandHandler.registerCommand("reset", HTTP_GET, doReset);
  restCommandHandler.registerCommand("calibrate", HTTP_GET, doCalibrate);
  restCommandHandler.registerCommand("debug", HTTP_GET, doGetDebug);
  restCommandHandler.registerCommand<int>("moveToPanel", HTTP_POST, {"panel"}, doMoveToPanel);
  restCommandHandler.registerCommand<int>("advancePanels", HTTP_POST, {"count"}, doAdvancePanels);
  restCommandHandler.registerCommand<int>("advancePulses", HTTP_POST, {"count"}, doAdvancePulses);
  restCommandHandler.registerCommand("panel", HTTP_GET, doGetCurrentPanel);
  restCommandHandler.registerCommand("saveSettings", HTTP_GET, doSaveSettings);
  restCommandHandler.registerCommand("loadSettings", HTTP_GET, doLoadSettings);
  restCommandHandler.registerCommand<int>("setDefaultPulse", HTTP_POST, {"pulse"}, doSetDefaultPulse);
  restCommandHandler.registerCommand<int, int>("setDefaultPanelAndOffset", HTTP_POST, {"panel", "offset"}, doSetDefaultPanelAndOffset);
  restCommandHandler.registerCommand("help", HTTP_GET, doGetRestRoutes);

  cors.setOrigin("*");
  server.addMiddleware(&cors);
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP server started");

  if (!LittleFS.begin()) {
    Serial.println("Erreur lors du montage de LittleFS !");
    return;
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  if (MDNS.begin("cff")) {  // TODO faire fonctionner
    Serial.println("mDNS démarré avec succès !");
  } else {
    Serial.println("Erreur lors de l'initialisation de mDNS");
  }

// WebSocket setup
#ifdef DEBUG_ENABLED
  ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      Serial.println("WebSocket client connected");
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.println("WebSocket client disconnected");
    }
  });
#endif

  // Read setup values from EEPROM
  EEPROM.begin(256);
  loadDefaultPulse();

  // Servo setup
  servo.attach(SERVO_PIN);
  servo.write(STOP_SPEED);  // Ensure the servo starts stopped using the centralized function

  // Optical sensor setup
  pinMode(OPTICAL_SENSOR_PIN, INPUT);  // Configure le capteur optique en entrée

  // Encoder setup
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderInterrupt, CHANGE);
}

void loop() {
  connectToWiFi();  // Keep it alive //TODO Wifi non bloquant + voir si problèmes en cas de déconnexion avec le serveur http ou autre
  readSensors();    // Read sensors and handle edge detection
  evaluateStateTransitions();
  processStateActions();
  checkForRunningErrors();  // Check for blockages and co
  serialCommandHandler.handleSerial();
  restCommandHandler.handleClient();
  notifyPanelChanges();

#ifdef DEBUG_ENABLED
  if (debugBuilder.hasChanged()) {  // 110 us
    Serial.println(debugBuilder.buildJson());
  }
#endif
}

void setMotorSpeed(float normalizedSpeed) {
  // If error was raised, stop the motor
  if (errorFlag) {
    servo.write(STOP_SPEED);
    return;
  }

  // Constrain normalized speed and map it to servo speed values
  normalizedSpeed = constrain(normalizedSpeed, 0.f, 1.f);
  int speed = denormalizeSpeed(normalizedSpeed);
  if (servo.read() != speed) {
    servo.write(speed);  // Skip if speed did not change (~20ms)
  }
}

float normalizeSpeed(int speed) {
  return map(speed, STOP_SPEED, RUN_SPEED, 0, 100) / 100.f;
}

int denormalizeSpeed(float normalizedSpeed) {
  return map(normalizedSpeed * 100, 0, 100, STOP_SPEED, RUN_SPEED);
}