#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Encoder.h>
#include <Servo.h>
#include <map>

// Prototypes declaration
int getCurrentPulses();
int getCurrentPanel();
void setCurrentPulses(int pulses);
void setCurrentPanel(int panel);
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

template <typename T>
void assertThis(bool condition, T&& message) {
  if (!condition) {
    emergencyStop(std::forward<T>(message));
  }
}

enum AppState {
  STOPPED,
  AUTO_CALIBRATING,
  MOVING_TO_TARGET,
  CALIBRATING
};

// TODO MECANIQUE :
//- Faciliter l'ajustement de la roue optique
//   - fixation par le haut ?
//   - fentes pour voir le trou ?
//   - ligne visuelle pour guider sur la roue (OUI FACILE)
//- Aligner fente avec dent car ça coince mieux sur cette position
//- Permettre de découpler les roues ? dur 
//  - Roue libre à insérer une fois que tout est en place pour permettre de libérer le mécanisme en manuel) ?
//- Trous de fixation corrects ? l'un était trop étroit
//- Trou axe roue optique trop petit il semble
//- Trous des vis encodeur pas assez profonds
//- Voir comment fixer axe roue optique + rondelles
//- Voir comment fixer roue encodeur + rondelles
//- Prévoir plus de place pour pins capteur optique

// TODO SOFTWARE :
// TODO Sécurités si valeur hors norme, stoppe servo
// TODO ? stocker la valeur de l'encodeur dans variable et la remettre à zéro avec l'optique, mais laisser la valeur de l'encodeur originale.
// - ou plutôt afficher la valeur de l'encodeur brute lors du passage de l'optique
// - et faire une variable previousPulses pour voir si une boucle ne loupe pas un step
// - vérifier qu'on a bien atteint le nombre de steps max au moment du passage et afficher un warning sinon
// TODO s'assurer que tous les "getters" utilisés notamment dans les debug ne change pas les états ou valeurs :-)
// TODO état ERROR ? en même temps le flag reste nécessaire car il garantit qu'on peut pas setter à nouveau autre chose via transition ou autre... mais la boucle continue de tourner et calculer des trucs pour rien...
// TODO Vérifier les valeurs passées depuis l'api un peu mieux en terme de limites et type
// TODO go to rising/falling edges via API ?
// TODO détecter si blocage (en fonction de l'encodeur et du temps écoulé) et emergencystop
// TODO garder les N derniers messages (erreur ou warning...)
// TODO API avec commandes plus poussées sur même endpoint ? par ex pour aider à définir les offsets
// - positionner par incréments de steps le panneau 0 à son tout début
// - lancer moteur jusqu'à l'optique et récupérer la valeur de l'encodeur à ce moment
// - calculer la pulse courante (attention au sens) et les constantes DEFAULT...
// - en tenant compte de l'offset et en remettant l'encodeur à 0 on obtient le panel courant et on peut vérifier si c'est OK

#define DEBUG_ENABLED 1

// Pins configuration
#define SERVO_PIN D3
#define ENCODER_PIN_A D1       // CLK
#define ENCODER_PIN_B D2       // DT
#define OPTICAL_SENSOR_PIN D6  // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;                               // Nombre total de panneaux
const int PULSES_PER_PANEL = 2;                            // Ajuste pour 4 impulsions par panneau
const int PULSES_COUNT = PANELS_COUNT * PULSES_PER_PANEL;  // Nombre total d'impulsions
const int DEFAULT_PANEL = 40;                              // Panel at optical sensor position
const int DEFAULT_PANEL_PULSE_OFFSET = 1;                  // Optical sensor is detected at nth pulse of the default panel //TODO Directement calculer un DEFAULT_PULSE via les deux variables et faire en sorte que ce soit dans les limites [0-PULSES_TOTAL]
const int DEFAULT_PULSE = (DEFAULT_PANEL * PULSES_PER_PANEL) + DEFAULT_PANEL_PULSE_OFFSET;
const int ENCODER_DIRECTION_SIGN = -1;
const int TARGET_PULSE_OFFSET = 1;  // When going to target, go to the nth pulse of the target panel //TODO chuis tjrs pas sur que ce soit utile... au moment du passage optique, suffit de lui faire croire qu'il est en avant ou en arrière et ça devrait faire le job pareil
static_assert(TARGET_PULSE_OFFSET >= 0 && TARGET_PULSE_OFFSET < PULSES_PER_PANEL, "OFFSET must be in range [0-PULSES_PER_PANEL]");
static_assert(DEFAULT_PANEL_PULSE_OFFSET >= 0 && DEFAULT_PANEL_PULSE_OFFSET < PULSES_PER_PANEL, "OFFSET must be in range [0-PULSES_PER_PANEL]");
const bool DETECTED_STATE = LOW;  // Define the detected state for the optical sensor

// Servo configuration
Servo servo;
const int STOP_SPEED = 90;  // Valeur pour arrêter le servo
const int RUN_SPEED = 140;  // Vitesse du servo pour avancer (91-180) 140 c'est bien (met 6 secondes à faire un tour)
static_assert(RUN_SPEED > 90, "RUN_SPEED must be greater than 90 or everything will break !");

// Globals
ESP8266WebServer server(80);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
int targetPanel = 0;                 // Panneau cible //TODO utiliser un targetPulses et virer quasi tous les appels get/setPanel sauf depuis l'api ou les debug
bool motorEnabled = false;           // Boolean to enable/disable motor, starts disabled
int encoderValue = 0;                // Variable to store the encoder value
bool isOpticalEdgeDetected = false;  // Variable to store if the optical edge is detected during the current loop
int sensorState = LOW;
int lastSensorState = HIGH;  // Initialize to HIGH (not detected)
bool errorFlag = false;      // Emergency stop flag
String errorMessage = "";    // Emergency stop message
AppState currentState = STOPPED;
int targetPulses = 0;  // New targetPulses property
bool calibrated = false;

#ifdef DEBUG_ENABLED
std::map<String, String> lastDebugMessages;  // Map to store the last debug messages
#endif

// Setup Wi-Fi
const char* ssid = "ap8F2EOjLm";
const char* password = "gsecumonwifiii123";

void emergencyStop(String message) {
  errorFlag = true;
  errorMessage = message;
  servo.write(STOP_SPEED);  // Stop the motor immediately
  Serial.println("EMERGENCY STOP: " + message);
}

int getCurrentPanel() {
  return getCurrentPulses() / PULSES_PER_PANEL;
}

int getCurrentPulses() {
  return (encoderValue * ENCODER_DIRECTION_SIGN) % PULSES_COUNT;  // TODO LOW n'est pas censé avoir besoin de modulo si l'encodeur ne génère pas de steps en trop depuis le denier optique
}

void setCurrentPulses(int pulses) {
  encoder.write((pulses % PULSES_COUNT) * ENCODER_DIRECTION_SIGN);
}

int getTargetPanel() {
  return targetPulses / PULSES_PER_PANEL;
}

void setTargetPanel(int panel) {
  assertThis(panel < PANELS_COUNT, "panel " + String(panel) + " > " + PANELS_COUNT);
  setTargetPulses((panel * PULSES_PER_PANEL) + TARGET_PULSE_OFFSET);
}

void setTargetPulses(int pulses) {
  assertThis(pulses < PULSES_COUNT, "pulses " + String(pulses) + " > " + PULSES_COUNT);  // TODO à voir si la création du string n'est pas appelée (appeler ici une fonction qui retourne string et print un truc dans serial)
  targetPulses = pulses;
}

int getTargetPulses() {
  return targetPulses;
}

String buildDebugJson(String message) {
  String debugJson = "{";
  debugJson += "\"msg\":\"" + message + "\"" +
               ",\"state\":" + String(currentState) +
               ",\"currentPanel\":" + String(getCurrentPanel()) +
               ",\"currentPulses\":" + String(getCurrentPulses()) +
               ",\"targetPanel\":" + String(getTargetPanel()) +
               ",\"targetPulses\":" + String(getTargetPulses()) +
               ",\"rawPulses\":" + String(encoderValue) +
               ",\"optical\":" + String(sensorState) +
               ",\"edge\":" + String(isOpticalEdgeDetected) +
               ",\"calibrated\":" + String(calibrated) +
               ",\"dist\":" + String(getRemainingPulses()) +
               ",\"speed\":" + String(calculateSpeedMovingToTarget()) +
               ",\"errorFlag\":" + String(errorFlag) +
               ",\"errorMessage\":\"" + errorMessage + "\"";
  debugJson += "}";
  return debugJson;
}

#ifdef DEBUG_ENABLED
void serialPrintThrottled(String key, String message) {
  if (lastDebugMessages[key] != message) {
    Serial.println(message);
    lastDebugMessages[key] = message;
  }
}
#endif

void setCurrentState(AppState newState) {
  serialPrintThrottled("STATE", "state:" + String(currentState) + " newState:" + String(newState));
  currentState = newState;
}

// Functions

void sendHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
}

void handleGetDebug() {
  sendHeaders();
  server.send(200, "text/plain", buildDebugJson("Debug request"));
}

void handleGetCurrentPanel() {
  sendHeaders();
  server.send(200, "text/plain", String(getCurrentPanel()));
}

void handleMoveToPanel() {
  sendHeaders();
  if (server.hasArg("panel")) {
    int panel = server.arg("panel").toInt();
    panel %= PANELS_COUNT;  // Ensure target is within bounds
    setTargetPanel(panel);
    setCurrentState(MOVING_TO_TARGET); 
    server.send(200, "text/plain", buildDebugJson("Moving to panel " + String(getTargetPanel())));
  } else {
    server.send(400, "text/plain", "Missing 'panel' argument");
  }
}

void handleAdvancePanels() {
  sendHeaders();
  if (server.hasArg("count")) {
    int advanceCount = server.arg("count").toInt();
    int currentPanel = getCurrentPanel();
    setTargetPanel((currentPanel + advanceCount) % PANELS_COUNT);  
    setCurrentState(MOVING_TO_TARGET);                             
    server.send(200, "text/plain", buildDebugJson("From " + String(currentPanel) + ", Advancing " + String(advanceCount) + " panels to " + String(getTargetPanel())));
  } else {
    server.send(400, "text/plain", "Missing 'count' argument");
  }
}

void handleAdvancePulses() {
  sendHeaders();
  if (server.hasArg("count")) {
    int pulseCount = server.arg("count").toInt();
    int currentPulses = getCurrentPulses();
    setTargetPulses((currentPulses + pulseCount) % PULSES_COUNT);  
    setCurrentState(MOVING_TO_TARGET);                             
    server.send(200, "text/plain", buildDebugJson("Advancing " + String(pulseCount) + " pulses to " + String(targetPulses)));
  } else {
    server.send(400, "text/plain", "Missing 'count' argument");
  }
}

void handleCalibrate() {
  sendHeaders();
  setCurrentState(CALIBRATING);  // Set motor mode for calibration
  server.send(200, "text/plain", buildDebugJson("Calibration started. Rotating until optical sensor edge is detected."));
}

void handleStop() {
  sendHeaders();
  setMotorSpeed(0);          // Use the centralized function to stop the motor
  setCurrentState(STOPPED);  // Set motor mode to stopped
  int currentPanel = getCurrentPanel();
  setTargetPanel(currentPanel);
  server.send(200, "text/plain", buildDebugJson("Stopped. Current panel set to " + String(currentPanel)));
}

int getRemainingPulses() {
  int currentPulses = getCurrentPulses();
  return (targetPulses - currentPulses + PULSES_COUNT) % PULSES_COUNT;  // Calcule la distance en avance (horaire)
}

bool isTargetPanelReached() {
  return getRemainingPulses() == 0;
}

float calculateSpeedCalibration() {
  // Half speed when we are about to detect the 2nd edge  // TODO problème si on ralentit à la calibration initiale, au prochain tour à plein régime on va pas avoir exactement la même position ? Après essais pratiques, semblerait que ça fasse aucune différence car le tout est assez réactif même à pleine vitesse
  if (DETECTED_STATE == LOW) {
    return sensorState == LOW ? 0.5f : 1.0f;
  } else {
    return sensorState == HIGH ? 0.5f : 1.0f;
  }
}

float calculateSpeedMovingToTarget() {
  int remainingPulses = getRemainingPulses();
  if (remainingPulses == 0) {
    return 0.f;
  } else if (remainingPulses <= PULSES_PER_PANEL) {
    return 0.3f;
  } else if (remainingPulses <= PULSES_PER_PANEL * 3) {
    return 0.5f;
  } else if (remainingPulses <= PULSES_PER_PANEL * 5) {
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
      if (isTargetPanelReached()) {
        setCurrentState(STOPPED);
      }
      break;
    }
    case CALIBRATING: {
      if (isOpticalEdgeDetected) {
        setCurrentState(STOPPED);
      }
      break;
    }
  }
}

void processStateActions() {
  switch (currentState) {
    case STOPPED:
      setMotorSpeed(0);
      break;
    case AUTO_CALIBRATING: {
      float speed = calculateSpeedCalibration();
      setMotorSpeed(speed);
    }
    case MOVING_TO_TARGET: {
      float speed = calculateSpeedMovingToTarget();
      setMotorSpeed(speed);
    } break;
    case CALIBRATING: {
      float speed = calculateSpeedCalibration();
      setMotorSpeed(speed);
    } break;
  }
}

void readSensors() {
  // Read optical sensor state and do edge detection
  sensorState = digitalRead(OPTICAL_SENSOR_PIN);
  isOpticalEdgeDetected = (DETECTED_STATE == LOW && sensorState == HIGH && lastSensorState == LOW) ||
                          (DETECTED_STATE == HIGH && sensorState == LOW && lastSensorState == HIGH);
  lastSensorState = sensorState;

  // Read encoder value, reset if edge was detected
  if (isOpticalEdgeDetected) {
    calibrated = true;
    serialPrintThrottled("OPTICALSTATE", "Optical edge detected");
    setCurrentPulses(DEFAULT_PULSE);
  }
  encoderValue = encoder.read();
}

void connectToWiFi() {
  assert(!WiFi.getPersistent());
  if (WiFi.status() != WL_CONNECTED) {
    setMotorSpeed(0);  // Stop the motor if we are offline
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi " + String(ssid) + "...");
    }
    Serial.println("Connected to WiFi " + String(ssid) + "!");
    Serial.println("IP Address: " + WiFi.localIP().toString() + " RSSI: " + WiFi.RSSI());
  }
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();

  // REST API routes
  server.on("/panel", HTTP_GET, handleGetCurrentPanel);  // TODO renommer en virant les gets et les mots panel des actions
  server.on("/debug", HTTP_GET, handleGetDebug);
  server.on("/moveToPanel", HTTP_POST, handleMoveToPanel);
  server.on("/advancePanels", HTTP_POST, handleAdvancePanels);
  server.on("/advancePulses", HTTP_POST, handleAdvancePulses);
  server.on("/calibrate", HTTP_GET, handleCalibrate); //TODO POST pour les actions memes simpes ?
  server.on("/stop", HTTP_GET, handleStop);
  server.begin();
  Serial.println("HTTP server started");

  // Servo setup
  servo.attach(SERVO_PIN);
  setMotorSpeed(0);  // Ensure the servo starts stopped using the centralized function

  // Optical sensor setup
  pinMode(OPTICAL_SENSOR_PIN, INPUT);                 // Configure le capteur optique en entrée
  lastSensorState = digitalRead(OPTICAL_SENSOR_PIN);  // Initialize lastSensorState
}

void loop() {
  connectToWiFi(); // Keep it alive
  readSensors();
  evaluateStateTransitions();
  processStateActions();
#ifdef DEBUG_ENABLED
  serialPrintThrottled("ALL", buildDebugJson(""));
#endif
  server.handleClient();  // Handle incoming HTTP requests
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
  if (speed < 90) {
    emergencyStop("Backwards speed is not allowed");
    return;
  }
  servo.write(speed);
}
