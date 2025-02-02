#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Servo.h>
#include <secrets.h>
#include <map>
#include <commandHandler.h>
#include <restCommandHandler.h>


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
String buildDebugJson(String message);
void sendHeaders();
float calculateSpeedMovingToTarget();
int getRemainingPulses();
void setMotorSpeed(float normalizedSpeed);
void setTargetPanel(int panel);
void handleAdvancePulses();
int getTargetPanel();
void emergencyStop(String message);
String stateToString(AppState state);
void setCurrentState(AppState newState);

template <typename T>
void assertThis(bool condition, T&& message) {
  if (!condition) {
    emergencyStop(std::forward<T>(message));
  }
}

#define DEBUG_ENABLED 1

// Pins configuration
#define SERVO_PIN D3
#define ENCODER_PIN_A D1       // CLK
#define ENCODER_PIN_B D2       // DT
#define OPTICAL_SENSOR_PIN D6  // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;
const int PULSES_PER_PANEL = 24; //ENCODER_RESOLUTION / ENCODER_GEAR_TEETH * ENCODER_PULSES_PER_STEP = 360 / 60 * 4 = 24
const int PULSES_COUNT = PANELS_COUNT * PULSES_PER_PANEL;  // Nombre total d'impulsions (2232 par tour de panel. Encodeur tourne 1.55x plus vite que panels, 2232/1.55 = 1440 pulses par tour d'encodeur = 360 steps)
const int ENCODER_DIRECTION_SIGN = 1;  // TODO c'est pas clair si c'est géré par l'interruption sans s'en soucier à la lecture car on en tient compte dans le getter... et on en tient compte 2x dans l'interruption ce qui semble etre faux
const int TARGET_PULSE_OFFSET = 12;    // When going to target, go to the nth pulse of the target panel
static_assert(TARGET_PULSE_OFFSET >= 0 && TARGET_PULSE_OFFSET < PULSES_PER_PANEL, "OFFSET must be in range [0-PULSES_PER_PANEL]");

// Globals
String command = "";  // Current serial command
unsigned long loopMillis = 0;
unsigned long lastLoopMillis = 0;  // Variable to store the last loop time
ESP8266WebServer server(80);
bool errorFlag = false;    // Emergency stop flag
String errorMessage = "";  // Emergency stop message
volatile AppState currentState = STOPPED;
volatile bool calibrated = false;

// Encoder
int defaultPulse = 0;
int targetPulses = 0;  // New targetPulses property
volatile int encoderInterruptCallCount = 0;
volatile uint8_t encoderState = 0;
volatile int encoderPulses = 0;
volatile int encoderPulsesRaw = 0;
const int8_t ENCODER_STATE_TABLE[16] = {0, 1, -1, -0, -1, 0, -0, 1, 1, -0, 0, -1, -0, -1, 1, 0};  // Encoder state table for natural debouncing (-0 are non valid states)

// Optical sensor
const int OPTICAL_DETECTED_EDGE = RISING;  // Capteur à 1 quand coupé / 0 quand trou / ralentit quand trou, et calibre sur rising vers coupé
const bool OPTICAL_DETECTED_STATE = LOW;             // Define the detected state for the optical sensor
volatile bool opticalState = LOW;
volatile bool opticalLastState = HIGH;        // Initialize to HIGH (not detected)
volatile int opticalDetectedPulses = 0;
volatile int opticalDetectedEdgesCount = 0;

// Servo
Servo servo;
const int STOP_SPEED = 90;  // Valeur pour arrêter le servo
const int RUN_SPEED = 140;  // Vitesse du servo pour avancer (91-180) 140 c'est bien (met 6 secondes à faire un tour)
static_assert(RUN_SPEED > 90, "RUN_SPEED must be greater than 90 or everything will break !");
const unsigned long BLOCKAGE_TIMEOUT = 500;  // Timeout in milliseconds to detect blockage

#ifdef DEBUG_ENABLED
std::map<String, String> lastDebugMessages;  // Map to store the last debug messages
#endif

void emergencyStop(String message) {
  servo.write(STOP_SPEED);  // First things first, stop the motor
  setCurrentState(EMERGENCY_STOPPED);
  errorFlag = true;  // TODO à voir si redondant ou safe ou si servo.detach() serait mieux et si ça fonctionnerait
  errorMessage = message;
  Serial.println(String(loopMillis) + " EMERGENCY STOP: " + message);
}

int getCurrentPulses() {
  return (encoderPulses * ENCODER_DIRECTION_SIGN) % PULSES_COUNT;
}

void setTargetPulses(int pulses) {
  assertThis(pulses >= 0 && pulses < PULSES_COUNT, "pulses " + String(pulses) + " out of bounds");  // TODO à voir si la création du string n'est pas appelée (appeler ici une fonction qui retourne string et print un truc dans serial)
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
  assertThis(panel < PANELS_COUNT, "panel " + String(panel) + " > " + PANELS_COUNT);
  setTargetPulses((panel * PULSES_PER_PANEL) + TARGET_PULSE_OFFSET);
}

int getDefaultPanel() {
  return defaultPulse / PULSES_PER_PANEL;
}

int getDefaultPulseOffset() {
  return defaultPulse % PULSES_PER_PANEL;
}

String buildDebugJson(String message) {
  String debugJson = "{";
  debugJson += "\"msg\":\"" + message + "\"" +
               //",\"dur\":" + String(loopMillis - lastLoopMillis) +
               ",\"currentPanel\":" + String(getCurrentPanel()) +
               ",\"currentPulses\":" + String(getCurrentPulses()) +
               ",\"targetPanel\":" + String(getTargetPanel()) +
               ",\"targetPulses\":" + String(getTargetPulses()) +
               ",\"optical\":" + String(opticalState) +
               ",\"odPulses\":" + String(opticalDetectedPulses) +
               ",\"odCount\":" + String(opticalDetectedEdgesCount) +
               ",\"encPulses\":" + String(encoderPulses) +
               ",\"encPulsesRaw\":" + String(encoderPulsesRaw) +
               ",\"encInt\":" + String(encoderInterruptCallCount) +
               ",\"calibrated\":" + String(calibrated) +
               ",\"dist\":" + String(getRemainingPulses()) +
               ",\"servo\":" + String(servo.read()) +
               ",\"defaultPulse\":" + String(defaultPulse) +
               ",\"defaultPanel\":" + String(getDefaultPanel()) +
               ",\"defaultPulseOffset\":" + String(getDefaultPulseOffset()) +
               ",\"state\":" + stateToString(currentState) +
               //",\"errorFlag\":" + String(errorFlag) +
               ",\"errorMessage\":\"" + errorMessage + "\"" +
               "}";
  return debugJson;
}

#ifdef DEBUG_ENABLED
void serialPrintThrottled(String key, String message) {
  if (lastDebugMessages[key] != message) {
    Serial.println(String(loopMillis) + " " + message);
    lastDebugMessages[key] = message;
  }
}
#endif

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

void doSetupGoToZero() {
  calibrated = false;  // TODO Voir si c'ets bien de réutiliser la même var
  defaultPulse = 0;    // Reset default pulse
  setCurrentState(SETUP_GO_TO_ZERO);
}

void doSetupNextPulse() {
  if (currentState != SETUP_WAITING_COMMAND) {
    return;
  }
  setCurrentState(SETUP_MOVE_PULSE);
  targetPulses = (getCurrentPulses() + 1) % PULSES_COUNT;
  Serial.println("Next pulse: " + String(targetPulses)); //TODO retourner message depuis les méthodes d'action
  // Will make currentPulses increment
}

void doSetupSetPanelNb(int panel) {
  if (currentState != SETUP_WAITING_COMMAND) {
    return;
  }

  setCurrentState(STOPPED);
  defaultPulse = (panel * PULSES_PER_PANEL) - getCurrentPulses();
  assertThis(defaultPulse >= 0 && defaultPulse < PULSES_COUNT, "defaultPulse " + String(defaultPulse) + " out of bounds [0-" + String(PULSES_COUNT) + "]");
  encoderPulses += defaultPulse; //TODO à virer si on gère l'offset dynamiquement ce qui serait pas mal ET SURTOUT ne pas placer cette ligne avant defaultPulse=...
  saveDefaultPulse();
}

void doSetupCancel() {
  setCurrentState(STOPPED);
  loadDefaultPulse();
}

void sendHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
}

String doGetDebug() {
  return buildDebugJson("Debug request");
}

void handleGetDebug() {
  sendHeaders();
  server.send(200, "text/plain", doGetDebug());
}

int doGetCurrentPanel() {
  return getCurrentPanel();
}

void handleGetCurrentPanel() {
  sendHeaders();
  server.send(200, "text/plain", String(doGetCurrentPanel()));
}

String doMoveToPanel(int panel) {
  panel %= PANELS_COUNT;  // Ensure target is within bounds
  setTargetPanel(panel);
  setCurrentState(MOVING_TO_TARGET);
  return "Moving to panel " + String(getTargetPanel());
}

void handleMoveToPanel() {
  sendHeaders();
  if (server.hasArg("panel")) {
    int panel = server.arg("panel").toInt();
    String message = doMoveToPanel(panel);
    server.send(200, "text/plain", buildDebugJson(message));
  } else {
    server.send(400, "text/plain", "Missing 'panel' argument");
  }
}

String doAdvancePanels(int count) {
  int currentPanel = getCurrentPanel();
  setTargetPanel((currentPanel + count) % PANELS_COUNT);
  setCurrentState(MOVING_TO_TARGET);
  return "From " + String(currentPanel) + ", Advancing " + String(count) + " panels to " + String(getTargetPanel());
}

void handleAdvancePanels() {
  sendHeaders();
  if (server.hasArg("count")) {
    int count = server.arg("count").toInt();
    String message = doAdvancePanels(count);
    server.send(200, "text/plain", buildDebugJson(message));
  } else {
    server.send(400, "text/plain", "Missing 'count' argument");
  }
}

String doAdvancePulses(int pulseCount) {
  int currentPulses = getCurrentPulses();
  setTargetPulses((currentPulses + pulseCount) % PULSES_COUNT);
  setCurrentState(MOVING_TO_TARGET);
  return "Advancing " + String(pulseCount) + " pulses to " + String(targetPulses);
}

void handleAdvancePulses() {
  sendHeaders();
  if (server.hasArg("count")) {
    int pulseCount = server.arg("count").toInt();
    String message = doAdvancePulses(pulseCount);
    server.send(200, "text/plain", buildDebugJson(message));
  } else {
    server.send(400, "text/plain", "Missing 'count' argument");
  }
}

String doCalibrate() {
  setCurrentState(CALIBRATING);  // Set motor mode for calibration
  calibrated = false;
  return "Calibration started. Rotating until optical sensor edge is detected.";
}

void handleCalibrate() {
  sendHeaders();
  String message = doCalibrate();
  server.send(200, "text/plain", buildDebugJson(message));
}

String doStop() {
  setMotorSpeed(0);          // Use the centralized function to stop the motor
  setCurrentState(STOPPED);  // Set motor mode to stopped
  setTargetPulses(getCurrentPulses());
  return "Stopped. Current panel set to " + String(getCurrentPanel());
}

void handleStop() {
  sendHeaders();
  String message = doStop();
  server.send(200, "text/plain", buildDebugJson(message));
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

void handleReset() {
  sendHeaders();
  String message = doReset();
  server.send(200, "text/plain", buildDebugJson(message));
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
        setCurrentState(STOPPED);  // TODO Go to panel 0 instead via MOVING_TO_TARGET ? (attention pas changer les valeurs de quoi que ce soit ici en principe mais à voir)
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
      // Wait for next pulse command
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
  // TODO ça fonctionne pas..... avant fonctionnait MAIS avec un délai de 500ms qui me semble très long et y aurait-il moyen d'utiliser la même variable ? problème vu qu'elle est mise à jour à chaque loop, quand le moteur tourne à la loop suivante si le délai est dépassé, ça détecte que ça n'a pas bougé. Voir si alternative possible : https://chatgpt.com/c/678e9109-34b8-8005-ac88-cb9013f09a07
  int motorSpeed = servo.read();

  // Detect blockage
  {
    static unsigned long lastBlockageCheckTime = 0;  // Time of the last encoder check for blockage
    static int lastBlockageCheckPulses = 0;

    if (motorSpeed > STOP_SPEED && loopMillis - lastBlockageCheckTime > BLOCKAGE_TIMEOUT) {
      if (getCurrentPulses() == lastBlockageCheckPulses) {
        emergencyStop("Blockage detected: Encoder value did not change for " + String(BLOCKAGE_TIMEOUT) + " ms");
      } else if (getCurrentPulses() < lastBlockageCheckPulses) {
        emergencyStop("Blockage detected: Encoder value decreased");
      }
      // When motor is running, update values only after each timed check
      lastBlockageCheckTime = loopMillis;
      lastBlockageCheckPulses = getCurrentPulses();
    } else {
      // When motor is stopped, always update values so when it starts, we wait for a full timeout before 1st check (grace period)
      lastBlockageCheckTime = loopMillis;
      lastBlockageCheckPulses = getCurrentPulses();
    }
  }

  // Detect motor speed out of bounds
  {
    if (motorSpeed < 90) {
      emergencyStop("Motor speed should not be lower than 90");
    } else if (motorSpeed > 180) {
      emergencyStop("Motor speed should not be greater than 180");
    }
  }
}

void IRAM_ATTR handleEncoderInterrupt() {
  encoderInterruptCallCount++;
  byte pinA = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> ENCODER_PIN_A) & 1;
  byte pinB = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> ENCODER_PIN_B) & 1;
  encoderState = ((encoderState << 2) | (pinA << 1) | pinB) & 15;  // Décale et masque les bits
  int8_t pulseInc = ENCODER_STATE_TABLE[encoderState];
  encoderPulses += pulseInc;
  encoderPulsesRaw += pulseInc;  // TODO pas génial, faudrait gérer l'offset dans le getter ou le setter à partir du raw systématiquement et n'incrémenter que le raw

  if (pulseInc < 1)
    return;

  // Check if the optical sensor detected an edge
  opticalState = (GPIO_REG_READ(GPIO_IN_ADDRESS) >> OPTICAL_SENSOR_PIN) & 1;
  if ((OPTICAL_DETECTED_STATE == LOW && opticalState == HIGH && opticalLastState == LOW) ||
      (OPTICAL_DETECTED_STATE == HIGH && opticalState == LOW && opticalLastState == HIGH)) {
    opticalDetectedEdgesCount++;
    opticalDetectedPulses = encoderPulses % PULSES_COUNT;
    encoderPulses = defaultPulse * ENCODER_DIRECTION_SIGN;
    encoderPulsesRaw = 0;
    calibrated = true;
  }
  opticalLastState = opticalState;

  // Check if the target is reached
  if (currentState == MOVING_TO_TARGET && targetPulses == (encoderPulses * ENCODER_DIRECTION_SIGN) % PULSES_COUNT) {
    currentState = STOPPED;
  }
}

void handleSerialCommand(String command) {
  if (command == "stop") {
    String response = doStop();
    Serial.println(response);
  } else if (command == "reset") {
    String response = doReset();
    Serial.println(response);
  } else if (command == "calibrate") {
    String response = doCalibrate();
    Serial.println(response);
  } else if (command.startsWith("moveToPanel")) {
    int panel = command.substring(command.indexOf(' ') + 1).toInt();
    String response = doMoveToPanel(panel);
    Serial.println(response);
  } else if (command.startsWith("advancePanels")) {
    int count = command.substring(command.indexOf(' ') + 1).toInt();
    String response = doAdvancePanels(count);
    Serial.println(response);
  } else if (command.startsWith("advancePulses")) {
    int pulseCount = command.substring(command.indexOf(' ') + 1).toInt();
    String response = doAdvancePulses(pulseCount);
    Serial.println(response);
  } else if (command == "debug") {
    String response = doGetDebug();
    Serial.println(response);
  } else if (command == "currentPanel") {
    int panel = doGetCurrentPanel();
    Serial.println("Current panel: " + String(panel));
  } else if (command == "setupGoToZero") {
    doSetupGoToZero();
    Serial.println("Setup: Go to zero");
  } else if (command == "setupNextPulse") {
    doSetupNextPulse();
    Serial.println("Setup: Next pulse");
  } else if (command.startsWith("setupSetPanelNb")) {
    int panel = command.substring(command.indexOf(' ') + 1).toInt();
    doSetupSetPanelNb(panel);
    Serial.println("Setup: Set panel number " + String(panel));
  } else if (command == "setupCancel") {
    doSetupCancel();
    Serial.println("Setup: Cancel");
  } else {
    Serial.println("Unknown command: -" + command + "-");
  }
}

void readSerial() {
  if (Serial.available() > 0) {
    String incomingChar = Serial.readString();  // Lire un caractère
    if (incomingChar.endsWith("\n")) {
      handleSerialCommand(command);  // Traiter la commande complète
      command = "";                  // Réinitialiser pour la prochaine commande
    } else {
      command += incomingChar;  // Ajouter le caractère à la commande
    }
  }
}

// Instance globale pour gérer les commandes
CommandHandler commandHandler;
RestCommandHandler restCommandHandler(server);

void setup() {
  assert(!WiFi.getPersistent());
  Serial.begin(115200);
  Serial.setTimeout(10);

  commandHandler.registerCommand<int>("move", doMoveToPanel);
    restCommandHandler.registerCommand<int>("moveToPanel", HTTP_POST, {"panelNb"}, doMoveToPanel);


  // REST API routes
  server.on("/panel", HTTP_GET, handleGetCurrentPanel);  // TODO renommer en virant les gets et les mots panel des actions
  server.on("/debug", HTTP_GET, handleGetDebug);
  server.on("/moveToPanel", HTTP_POST, handleMoveToPanel);
  server.on("/advancePanels", HTTP_POST, handleAdvancePanels);
  server.on("/advancePulses", HTTP_POST, handleAdvancePulses);
  server.on("/calibrate", HTTP_GET, handleCalibrate);  // TODO POST pour les actions memes simpes ?
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/reset", HTTP_GET, handleReset);
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
  // attachInterrupt(digitalPinToInterrupt(OPTICAL_SENSOR_PIN), handleOpticalSensorInterrupt, OPTICAL_DETECTED_EDGE);  // Attach interrupt

  // Encoder setup
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderInterrupt, CHANGE);
}

void loop() {
      commandHandler.handleSerialCommands();

  server.handleClient();  // Handle incoming HTTP requests
  connectToWiFi();        // Keep it alive
  loopMillis = millis();
  readSensors();  // Read sensors and handle edge detection
  evaluateStateTransitions();
  processStateActions();
  checkForRunningErrors();  // Check for blockages anc co
  //readSerial();             // Read and handle serial commands

#ifdef DEBUG_ENABLED
  serialPrintThrottled("ALL", buildDebugJson(""));
#endif
  lastLoopMillis = loopMillis;  // Update last loop time
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
  servo.write(speed);
}
