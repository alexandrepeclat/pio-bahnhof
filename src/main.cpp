#include <DebugBuilder.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <RestCommandHandler.h>
#include <SerialCommandHandler.h>
#include <Servo.h>
#include <secrets.h>
#include <map>

enum AppState {
  EMERGENCY_STOPPED,
  STOPPED,
  AUTO_CALIBRATING,
  MOVING_TO_TARGET,
  CALIBRATING,
  SETUP_GO_TO_ZERO,
  SETUP_MOVE_PULSE,
  SETUP_WAITING_COMMAND
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

#define DEBUG_ENABLED

// Pins configuration
#define SERVO_PIN D3
#define ENCODER_PIN_A D1       // CLK
#define ENCODER_PIN_B D2       // DT
#define OPTICAL_SENSOR_PIN D6  // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;
const int PULSES_PER_PANEL = 24;                           // ENCODER_RESOLUTION / ENCODER_GEAR_TEETH * ENCODER_PULSES_PER_STEP = 360 / 60 * 4 = 24
const int PULSES_COUNT = PANELS_COUNT * PULSES_PER_PANEL;  // Nombre total d'impulsions
const int ENCODER_DIRECTION_SIGN = 1;
const int TARGET_PULSE_OFFSET = 12;  // When setting a target panel, use the nth pulse of this panel for centering
static_assert(TARGET_PULSE_OFFSET >= 0 && TARGET_PULSE_OFFSET < PULSES_PER_PANEL, "OFFSET must be in range [0-PULSES_PER_PANEL]");

// Globals
unsigned long loopMicros = 0;
unsigned long lastLoopMicros = 0;  // Variable to store the last loop time
ESP8266WebServer server(80);
bool errorFlag = false;    // Emergency stop flag
String errorMessage = "";  // Emergency stop message
volatile AppState currentState = STOPPED;
volatile bool calibrated = false;
SerialCommandHandler serialCommandHandler;
RestCommandHandler restCommandHandler(server);

// Encoder
int defaultPulse = 0;
int targetPulses = 0;
volatile uint8_t encoderState = 0;
volatile int encoderPulses = 0;
const int8_t ENCODER_STATE_TABLE[16] = {0, 1, -1, -0, -1, 0, -0, 1, 1, -0, 0, -1, -0, -1, 1, 0};  // Encoder state table for natural debouncing (-0 are non valid states)

// Optical sensor
const int OPTICAL_DETECTED_EDGE = RISING;  // Capteur à 1 quand coupé / 0 quand trou / ralentit quand trou, et calibre sur rising vers coupé
volatile bool opticalState = LOW;
volatile bool opticalLastState = HIGH;  // Initialize to HIGH (not detected)

// Servo
Servo servo;
const int STOP_SPEED = 90;  // Valeur pour arrêter le servo
const int RUN_SPEED = 140;  // Vitesse du servo pour avancer (91-180) 140 c'est bien (met 6 secondes à faire un tour)
static_assert(RUN_SPEED > 90, "RUN_SPEED must be greater than 90 or everything will break !");
const unsigned long BLOCKAGE_TIMEOUT = 500;  // Timeout in milliseconds to detect blockage

#ifdef DEBUG_ENABLED
volatile int encoderPulsesRaw = 0;
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
    {"encPulsesRaw", false, [] { return encoderPulsesRaw; }},
    {"optEdgeCount", true, [] { return opticalDetectedEdgesCount; }},
    {"encIntCount", false, [] { return encoderInterruptCallCount; }},
#endif
    {"state", true, [] { return stateToString(currentState); }},
    {"loopDur(us)", false, [] { return loopMicros - lastLoopMicros; }},
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
  setCurrentState(EMERGENCY_STOPPED);
  errorFlag = true;  // TODO à voir mais à priori pas redondant avec le state car les états peuvent être changés par les commandes (sinon faut que chaque commande vérifie que l'état est pas erreur avant de se lancer (ou que la fonction setState() s'en charge !))
  errorMessage = message;
  Serial.println(String(loopMicros) + " EMERGENCY STOP: " + message);
}

int getCurrentPulses() {
  // noInterrupts(); //TODO à voir ça embete le port série de fonctionner pourtant c'est quand même séquentiel
  return encoderPulses;
  // interrupts();
}

void setTargetPulses(int pulses) {
  assertError(pulses >= 0 && pulses < PULSES_COUNT, [pulses] { return "pulses " + String(pulses) + " out of bounds"; });  // TODO à voir si la création du string n'est pas appelée (appeler ici une fonction qui retourne string et print un truc dans serial)
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
    case AUTO_CALIBRATING:
      return "AUTO_CALIBRATING";
    case MOVING_TO_TARGET:
      return "MOVING_TO_TARGET";
    case CALIBRATING:
      return "CALIBRATING";
    case SETUP_GO_TO_ZERO:
      return "SETUP_GO_TO_ZERO";
    case SETUP_MOVE_PULSE:
      return "SETUP_MOVE_PULSE";
    case SETUP_WAITING_COMMAND:
      return "SETUP_WAITING_COMMAND";
    default:
      return "UNKNOWN";
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

// Functions

String doGetRestRoutes() {
  return restCommandHandler.getRoutesList();
}

String doGetSerialCommands() {
  return serialCommandHandler.getCommandsList();
}

String doSetupManual(int pulse) {
  defaultPulse = pulse;
  calibrated = false;
  assertError(defaultPulse >= 0 && defaultPulse < PULSES_COUNT, [] { return "defaultPulse " + String(defaultPulse) + " out of bounds [0-" + String(PULSES_COUNT) + "]"; });
  saveDefaultPulse();
  return "Setup completed. Default pulse set to " + String(defaultPulse) + ". Default panel is " + String(getDefaultPanel()) + " with offset " + String(getDefaultPulseOffset());
}

String doSetupGoToZero() {
  calibrated = false;  // TODO Voir si c'ets bien de réutiliser la même var
  defaultPulse = 0;    // Reset default pulse
  setCurrentState(SETUP_GO_TO_ZERO);
  return "Going to zero. Wait for the motor to stop, then call 'setupNext'.";
}

String doSetupNextPulse() {
  if (currentState != SETUP_WAITING_COMMAND) {
    return "Invalid state for this command. Call 'setupStart' first.";
  }
  setCurrentState(SETUP_MOVE_PULSE);
  targetPulses = (getCurrentPulses() + 1) % PULSES_COUNT;
  return "Moving to next pulse. Call 'setupNext' if the panel did not change, or call 'setupEnd x' if it did where x is the panel number.";
}

String doSetupSetPanelNb(int panel) {
  if (currentState != SETUP_WAITING_COMMAND) {
    return "Invalid state for this command. Call 'setupStart' first.";
  }

  setCurrentState(STOPPED);
  defaultPulse = (panel * PULSES_PER_PANEL) - getCurrentPulses();
  assertError(defaultPulse >= 0 && defaultPulse < PULSES_COUNT, [] { return "defaultPulse " + String(defaultPulse) + " out of bounds [0-" + String(PULSES_COUNT) + "]"; });
  saveDefaultPulse();
  return "Setup completed. Default pulse set to " + String(defaultPulse) + ". Default panel is " + String(getDefaultPanel()) + " with offset " + String(getDefaultPulseOffset());
}

String doSetupCancel() {
  setCurrentState(STOPPED);
  loadDefaultPulse();
  return "Setup canceld. Default pulse retored to " + String(defaultPulse);
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
  setCurrentState(MOVING_TO_TARGET);
  return "Advancing " + String(pulseCount) + " pulses to " + String(targetPulses);
}

String doCalibrate() {
  setCurrentState(CALIBRATING);  // Set motor mode for calibration
  calibrated = false;
  setTargetPanel(0);
  return "Calibration started. Rotating until optical sensor edge is detected, then going to panel 0";
}

String doStop() {
  setMotorSpeed(0);          // Use the centralized function to stop the motor
  setCurrentState(STOPPED);  // Set motor mode to stopped
  setTargetPulses(getCurrentPulses());
  return "Stopped. Current panel set to " + String(getCurrentPanel());
}

String doReset() {
  setMotorSpeed(0);  // Stop the motor
  calibrated = false;
  errorFlag = false;
  errorMessage = "";
  loadDefaultPulse();
  setCurrentState(STOPPED);
  return "Reset";
}

int getRemainingPulses() {
  int currentPulses = getCurrentPulses();
  return (targetPulses - currentPulses + PULSES_COUNT) % PULSES_COUNT;  // Calcule la distance en avance (horaire)
}

bool isTargetPulseReached() {
  return getRemainingPulses() == 0;
}

float calculateSpeedSetup() {
  return 0.15f;
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
    return 0.f;
  } else if (remainingPulses <= PULSES_PER_PANEL) {
    return 0.3f;
  } else if (remainingPulses <= PULSES_PER_PANEL * 2) {
    return 0.5f;
  } else if (remainingPulses <= PULSES_PER_PANEL * 3) {
    return 0.7f;
  } else {
    return 1.0f;  // Constant speed accross all panels
  }
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
    case AUTO_CALIBRATING: {
      if (calibrated) {
        setCurrentState(MOVING_TO_TARGET);
      }
    } break;
    case MOVING_TO_TARGET: {
      if (!calibrated) {
        setCurrentState(AUTO_CALIBRATING);
      }
      if (isTargetPulseReached()) {
        setCurrentState(STOPPED);
      }
      break;
    }
    case CALIBRATING: {
      if (calibrated) {
        setCurrentState(MOVING_TO_TARGET);
      }
      break;
    }
    case SETUP_GO_TO_ZERO: {
      if (calibrated) {
        setCurrentState(SETUP_WAITING_COMMAND);
      }
      break;
    }
    case SETUP_MOVE_PULSE: {
      if (isTargetPulseReached()) {
        setCurrentState(SETUP_WAITING_COMMAND);
      }
      break;
    }
    case SETUP_WAITING_COMMAND: {
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
    case AUTO_CALIBRATING: {
      float speed = calculateSpeedCalibration();
      setMotorSpeed(speed);
      break;
    }
    case MOVING_TO_TARGET: {
      float speed = calculateSpeedMovingToTarget();
      setMotorSpeed(speed);
    } break;
    case CALIBRATING: {
      float speed = calculateSpeedCalibration();
      setMotorSpeed(speed);
    } break;
    case SETUP_GO_TO_ZERO: {
      float speed = calculateSpeedCalibration();
      setMotorSpeed(speed);
    } break;
    case SETUP_MOVE_PULSE: {
      float speed = calculateSpeedSetup();
      setMotorSpeed(speed);
    } break;
    case SETUP_WAITING_COMMAND: {
      setMotorSpeed(0);
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

  // Detect blockage
  static unsigned long lastBlockageCheckTime = 0;  // Time of the last encoder check for blockage
  static int lastBlockageCheckPulses = 0;
  if (motorSpeed > STOP_SPEED) {
    if (millis() - lastBlockageCheckTime > BLOCKAGE_TIMEOUT) {
      if (getCurrentPulses() == lastBlockageCheckPulses) {
        emergencyStop("Blockage detected: Encoder value did not change for " + String(BLOCKAGE_TIMEOUT) + " ms");
      } else if (getCurrentPulses() < lastBlockageCheckPulses) {
        emergencyStop("Blockage detected: Encoder value decreased");
      }
      // When motor is running, update values only after each timed check
      lastBlockageCheckTime = millis();
      lastBlockageCheckPulses = getCurrentPulses();
    }
  } else {
    // When motor is stopped, always update values so when it starts, we wait for a full timeout before 1st check (grace period)
    lastBlockageCheckTime = millis();
    lastBlockageCheckPulses = getCurrentPulses();
  }

  // Detect motor speed out of bounds
  if (motorSpeed < 90) {
    emergencyStop("Motor speed should not be lower than 90");
  } else if (motorSpeed > 180) {
    emergencyStop("Motor speed should not be greater than 180");
  }
}

void IRAM_ATTR handleEncoderInterrupt() {  // 4us
  byte pinA = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> ENCODER_PIN_A) & 1;
  byte pinB = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> ENCODER_PIN_B) & 1;
  encoderState = ((encoderState << 2) | (pinA << 1) | pinB) & 15;  // Décale et masque les bits
  int8_t pulseInc = ENCODER_STATE_TABLE[encoderState] * ENCODER_DIRECTION_SIGN;
  encoderPulsesRaw += pulseInc;
  encoderPulses = (encoderPulsesRaw + defaultPulse) % PULSES_COUNT;

#ifdef DEBUG_ENABLED
  encoderInterruptCallCount++;
#endif

  if (pulseInc < 1)
    return;

  // Check if the optical sensor detected an edge
  opticalState = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> OPTICAL_SENSOR_PIN) & 1;
  if ((OPTICAL_DETECTED_EDGE == RISING && opticalLastState == LOW && opticalState == HIGH) ||
      (OPTICAL_DETECTED_EDGE == FALLING && opticalLastState == HIGH && opticalState == LOW)) {
    assertWarn(!calibrated || abs(encoderPulses - defaultPulse) <= 1, [] { return "Missing steps ? Optical edge detected at pulse " + String(encoderPulses) + " instead of " + String(PULSES_COUNT); });
    encoderPulsesRaw = 0;
    encoderPulses = (encoderPulsesRaw + defaultPulse) % PULSES_COUNT;
    calibrated = true;

#ifdef DEBUG_ENABLED
    opticalDetectedEdgesCount++;
#endif
  }
  opticalLastState = opticalState;

  // Check if the target is reached
  if (currentState == MOVING_TO_TARGET && targetPulses == encoderPulses) {
    currentState = STOPPED;  // TODO pas fan de traiter la transition d'état ici... et on le fait aussi dans la loop et attention car faut protéger la lecture de currentState contre les interruptions
  }
}

void setup() {
  assert(!WiFi.getPersistent());
  Serial.begin(115200);
  Serial.setTimeout(10);

  // Register Serial commands
  serialCommandHandler.registerCommand("stop", doStop);
  serialCommandHandler.registerCommand("reset", doReset);
  serialCommandHandler.registerCommand("calibrate", doCalibrate);
  serialCommandHandler.registerCommand("debug", doGetDebug);
  serialCommandHandler.registerCommand<int>("moveToPanel", {"panel"}, doMoveToPanel);
  serialCommandHandler.registerCommand<int>("advancePanels", {"count"}, doAdvancePanels);
  serialCommandHandler.registerCommand<int>("advancePulses", {"count"}, doAdvancePulses);
  serialCommandHandler.registerCommand("panel", doGetCurrentPanel);
  serialCommandHandler.registerCommand("setupStart", doSetupGoToZero);
  serialCommandHandler.registerCommand("setupNext", doSetupNextPulse);
  serialCommandHandler.registerCommand<int>("setupEnd", {"panel"}, doSetupSetPanelNb);
  serialCommandHandler.registerCommand("setupCancel", doSetupCancel);
  serialCommandHandler.registerCommand<int>("setupManual", {"pulse"}, doSetupManual);
  serialCommandHandler.registerCommand("help", doGetSerialCommands);

  // Register REST API routes
  restCommandHandler.registerCommand("stop", HTTP_GET, doStop);
  restCommandHandler.registerCommand("reset", HTTP_GET, doReset);
  restCommandHandler.registerCommand("calibrate", HTTP_GET, doCalibrate);
  restCommandHandler.registerCommand("debug", HTTP_GET, doGetDebug);
  restCommandHandler.registerCommand<int>("moveToPanel", HTTP_POST, {"panel"}, doMoveToPanel);
  restCommandHandler.registerCommand<int>("advancePanels", HTTP_POST, {"count"}, doAdvancePanels);
  restCommandHandler.registerCommand<int>("advancePulses", HTTP_POST, {"count"}, doAdvancePulses);
  restCommandHandler.registerCommand("panel", HTTP_GET, doGetCurrentPanel);
  restCommandHandler.registerCommand("setupStart", HTTP_GET, doSetupGoToZero);
  restCommandHandler.registerCommand("setupNext", HTTP_GET, doSetupNextPulse);
  restCommandHandler.registerCommand<int>("setupEnd", HTTP_POST, {"panel"}, doSetupSetPanelNb);
  restCommandHandler.registerCommand("setupCancel", HTTP_GET, doSetupCancel);
  restCommandHandler.registerCommand<int>("setupManual", HTTP_POST, {"pulse"}, doSetupManual);
  restCommandHandler.registerCommand("help", HTTP_GET, doGetRestRoutes);

  server.begin();
  Serial.println("HTTP server started");

  // Read setup values from EEPROM
  EEPROM.begin(256);
  loadDefaultPulse();

  // Servo setup
  servo.attach(SERVO_PIN);
  servo.write(STOP_SPEED);  // Ensure the servo starts stopped using the centralized function

  // Optical sensor setup
  pinMode(OPTICAL_SENSOR_PIN, INPUT_PULLDOWN_16);  // Configure le capteur optique en entrée

  // Encoder setup
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderInterrupt, CHANGE);
}

void loop() {
  loopMicros = micros();
  connectToWiFi();  // Keep it alive //TODO Wifi non bloquant + voir si problèmes en cas de déconnexion avec le serveur http ou autre
  readSensors();    // Read sensors and handle edge detection
  evaluateStateTransitions();
  processStateActions();
  checkForRunningErrors();  // Check for blockages anc co
  serialCommandHandler.handleSerial();
  restCommandHandler.handleClient();

#ifdef DEBUG_ENABLED
  if (debugBuilder.hasChanged()) {  // 110 us
    Serial.println(debugBuilder.buildJson());
  }
#endif
  lastLoopMicros = loopMicros;  // Update last loop time
}

void setMotorSpeed(float normalizedSpeed) {
  // If error was raised, stop the motor
  if (errorFlag) {
    servo.write(STOP_SPEED);
    return;
  }

  // Constrain normalized speed and map it to servo speed values
  normalizedSpeed = constrain(normalizedSpeed, 0.f, 1.f);
  int speed = map(normalizedSpeed * 100, 0, 100, STOP_SPEED, RUN_SPEED);
  if (servo.read() != speed) {
    servo.write(speed);  // Skip if speed did not change (~20ms)
  }
}
