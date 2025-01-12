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
String buildDebugString();
String buildDebugJson(String message);
void sendHeaders();
float calculateEaseOutSpeed();
int getRemainingPulses();
void setMotorSpeed(float normalizedSpeed);
// TODO MECANIQUE :
//- Faciliter l'ajustement de la roue optique
//   - en roue libre ?
//   - fixation par le haut ?
//   - fentes pour voir le trou ?
//   - ligne visuelle pour guider sur la roue ?
//- Aligner fente avec dent car ça coince mieux sur cette position
//- Revoir alignement dents et trou de l'encodeur ? car 1 dent = 1 panneau doit tomber pile dans une step pleine de l'encodeur
//- Permettre de découpler les roues ? dur
//- Trous de fixation corrects ? l'un était trop étroit
//- Trou de roue optique trop petit il semble
//- Voir comment fixer axe roue optique + rondelles
//- Voir comment fixer roue encodeur + rondelles
//- Prévoir plus de place pour pins capteur optique
//- Roue libre à insérer une fois que tout est en place pour permettre de libérer le mécanisme en manuel ?

// TODO SOFTWARE :
//- Sécurités si valeur hors norme, stoppe servo
//- Remplacer service setCurrentPanel par un service calibration qui met l'état indéfini sur le panel courant (+ gérer panel courant indéfini tourne un tour jusqu'à déclencher l'optique)
//- Du coup démarrer en "indéfini" mais ne pas bouger, et à la première commande, aller jusqu'à l'optique au minimum
// TODO setTargetPanel ?
// TODO !!!!!!!!!!!! FAIRE UN MOVEUNTILOPTICALEDGE service pour avancer jusqu'à l'optique ? afficher le currentPulses et cie quand optical est détecté dans debug ?
// TODO ? stocker la valeur de l'encodeur dans variable et la remettre à zéro avec l'optique, mais laisser la valeur de l'encodeur originale. Permettrait de voir s'il y a un décalage au début et s'il y a un décalage progressif du à ratés en raison d'une boucle trop lente
// - ou plutôt afficher la valeur de l'encodeur brute lors du passage de l'optique
// - et faire une variable previousPulses pour voir si une boucle ne loupe pas un step
// TODO s'assurer que tous les "getters" utilisés notamment dans les debug ne change pas les états ou valeurs :-)
// TODO à voir la fonction de calibration ça me plait moyen : on doit gérer l'optique à plusieurs endroits (soit abstraire la notion rising/falling/low/high) (soit voir si pas mieux où on donne simplement le panel à atteindre et on laisse calibrer automatiquement, ce qui évite d'avoir une vitesse de calibration différente de la vitesse en opération)

#define DEBUG_ENABLED 1

// Pins configuration
#define SERVO_PIN D3
#define ENCODER_PIN_A D1       // CLK
#define ENCODER_PIN_B D2       // DT
#define OPTICAL_SENSOR_PIN D6  // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;     // Nombre total de panneaux
const int PULSES_PER_PANEL = 2;  // Ajuste pour 4 impulsions par panneau
const int DEFAULT_PANEL = 40;    // Panel at optical sensor position (si on calibre, il s'arrête sur le panel précédent l'optique, donc s'il s'arrête au 3, l'optique est au 4)
const int ENCODER_DIRECTION_SIGN = -1;
const int OFFSET = 0;  // Offset between pulses and panel, must not exceed PULSES_PER_PANEL
static_assert(OFFSET < PULSES_PER_PANEL, "OFFSET must be less than PULSES_PER_PANEL");
static_assert(OFFSET >= 0, "OFFSET must be non-negative");

#define DETECTED_STATE LOW  // Define the detected state for the optical sensor

// Servo configuration
Servo servo;
const int STOP_SPEED = 90;  // Valeur pour arrêter le servo
const int RUN_SPEED = 140;  // Vitesse du servo pour avancer (91-180) 140 c'est bien
static_assert(RUN_SPEED > 90, "RUN_SPEED must be greater than 90 or everything will break !");

// Globals
ESP8266WebServer server(80);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
int targetPanel = 0;        // Panneau cible //TODO utiliser un targetPulses et virer quasi tous les appels get/setPanel sauf depuis l'api ou les debug
bool motorEnabled = false;  // Boolean to enable/disable motor, starts disabled
int encoderValue = 0;       // Variable to store the encoder value
int sensorState = LOW;
int lastSensorState = HIGH;  // Initialize to HIGH (not detected)
bool errorFlag = false;      // Emergency stop flag
String errorMessage = "";    // Emergency stop message

enum MotorMode {
  STOPPED,
  GO_TO_TARGET,
  GO_TO_CALIBRATION
};

MotorMode motorMode = STOPPED;

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

int getCurrentPulses() {
  int pulses = encoderValue * ENCODER_DIRECTION_SIGN;
  pulses = pulses % (PANELS_COUNT * PULSES_PER_PANEL);
  return pulses;
}

int getCurrentPanel() {
  return (getCurrentPulses() - OFFSET) / PULSES_PER_PANEL;
}

void setCurrentPulses(int pulses) {
  pulses = pulses % (PANELS_COUNT * PULSES_PER_PANEL);
  encoder.write(pulses * ENCODER_DIRECTION_SIGN);
}

void setCurrentPanel(int panel) {
  int pulses = (panel * PULSES_PER_PANEL + OFFSET);
  setCurrentPulses(pulses);
}

int getTargetPulses() {
  return (targetPanel * PULSES_PER_PANEL + OFFSET) /*% (PANELS_COUNT * PULSES_PER_PANEL) TODO a voir si modulo nécessaire mais en principe pas*/;
}

String buildDebugJson(String message) {
  String debugJson = "{";
  debugJson += "\"message\":\"" + message + "\"," +
               "\"currentPanel\":" + String(getCurrentPanel()) + "," +
               "\"targetPanel\":" + String(targetPanel) + "," +
               "\"currentPulses\":" + String(getCurrentPulses()) + "," +
               "\"targetPulses\":" + String(getTargetPulses()) + "," +
               "\"rawPulses\":" + String(encoderValue) + "," +
               "\"optical\":" + String(digitalRead(OPTICAL_SENSOR_PIN)) + "," +
               "\"errorFlag\":" + String(errorFlag) + "," +
               "\"errorMessage\":\"" + errorMessage + "\"";
  debugJson += "}";
  return debugJson;
}

#ifdef DEBUG_ENABLED
String buildDebugString() {
  String debugString = "currentPanel: " + String(getCurrentPanel()) +
                       " targetPanel:" + String(targetPanel) +
                       " currentPulses:" + String(getCurrentPulses()) +
                       " targetPulses:" + String(getTargetPulses()) +
                       " rawPulses:" + String(encoderValue) +
                       " optical:" + String(digitalRead(OPTICAL_SENSOR_PIN)) +
                       " distance: " + String(getRemainingPulses()) +
                       " speed: " + String(calculateEaseOutSpeed()) +
                       " errorFlag: " + String(errorFlag) +
                       " errorMessage: " + errorMessage;

  return debugString;
}

void serialPrintThrottled(String key, String message) {
  if (lastDebugMessages[key] != message) {
    Serial.println(message);
    lastDebugMessages[key] = message;
  }
}
#endif

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
    targetPanel = server.arg("panel").toInt();
    targetPanel %= PANELS_COUNT;  // Assure que la cible est dans les limites
    motorMode = GO_TO_TARGET;     // Set motor mode to go to target
    server.send(200, "text/plain", buildDebugJson("Moving to panel " + String(targetPanel)));
  } else {
    server.send(400, "text/plain", "Missing 'panel' argument");
  }
}

void handleAdvancePanels() {
  sendHeaders();
  if (server.hasArg("count")) {
    int advanceCount = server.arg("count").toInt();
    int currentPanel = getCurrentPanel();
    targetPanel = (currentPanel + advanceCount) % PANELS_COUNT;
    motorMode = GO_TO_TARGET;  // Set motor mode to go to target
    server.send(200, "text/plain", buildDebugJson("From " + String(currentPanel) + ", Advancing " + String(advanceCount) + " panels to " + String(targetPanel)));
  } else {
    server.send(400, "text/plain", "Missing 'count' argument");
  }
}

void handleCalibrate() {
  sendHeaders();
  motorMode = GO_TO_CALIBRATION;  // Set motor mode for calibration
  server.send(200, "text/plain", buildDebugJson("Calibration started. Rotating until optical sensor edge is detected."));
}

void handleStop() {
  sendHeaders();
  setMotorSpeed(0);     // Use the centralized function to stop the motor
  motorMode = STOPPED;  // Set motor mode to stopped
  int currentPanel = getCurrentPanel();
  targetPanel = currentPanel;
  server.send(200, "text/plain", buildDebugJson("Stopped. Current panel set to " + String(currentPanel)));
}

int getRemainingPulses() {
  int currentPulses = getCurrentPulses();
  int targetPulses = getTargetPulses();
  int totalPulses = PANELS_COUNT * PULSES_PER_PANEL;
  int distance = (targetPulses - currentPulses + totalPulses) % totalPulses;  // Calcule la distance en avance (horaire)
  return distance;
}

bool isOpticalSensorEdgeDetected() {
  return (DETECTED_STATE == LOW && sensorState == HIGH && lastSensorState == LOW) ||
         (DETECTED_STATE == HIGH && sensorState == LOW && lastSensorState == HIGH);
         //TODO à voir car plus court mais moins explicite : return (sensorState != lastSensorState && sensorState == DETECTED_STATE);
}

float calculateCalibrationSpeed() {
  // Half speed when we are about to detect the 2nd edge  // TODO problème si on ralentit à la calibration initiale, au prochain tour à plein régime on va pas avoir exactement la même position ? Après essais pratiques, semblerait que ça fasse aucune différence car le tout est assez réactif même à pleine vitesse
  if (DETECTED_STATE == LOW) {
    return sensorState == LOW ? 0.5f : 1.0f;
  } else {
    return sensorState == HIGH ? 0.5f : 1.0f;
  }
}

bool isTargetPanelReached() {
  return getRemainingPulses() == 0;
}

float calculateEaseOutSpeed() {
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
    return 1.0f;
  }
}

void processStateMachine() {
  serialPrintThrottled("STATE", "motorMode:" + String(motorMode));
  switch (motorMode) {
    case STOPPED:
      setMotorSpeed(0);
      break;
    case GO_TO_TARGET: {
      if (isTargetPanelReached()) {
        motorMode = STOPPED;
        setMotorSpeed(0);
      } else {
        float speed = calculateEaseOutSpeed();
        setMotorSpeed(speed);
      }
      break;
    }
    case GO_TO_CALIBRATION: {
      if (isOpticalSensorEdgeDetected()) {
        motorMode = STOPPED;
        setMotorSpeed(0); //TODO ça m'embete un peu d'avoir setMotorSpeed ici car c'est déjà dans l'état STOPPED.... est-ce un problème ? d'un coté je préfère le faire direct plutôt que d'attendre la loop suivante
        //TODO Splitter la machine d'état en deux ? une fonction pour les transitions, et une pour les actions
      } else {
        float speed = calculateCalibrationSpeed();
        setMotorSpeed(speed);
      }
      break;
    }
  }
}

void readSensors() {
  sensorState = digitalRead(OPTICAL_SENSOR_PIN);
  if (isOpticalSensorEdgeDetected()) { //TODO remplacer par une variable isEdgeDetected comme ça je peux direct mettre lastState à jour et pas le gérer dehors de cette fonction. Faut le rendre inaccessible depuis dehors aussi
    serialPrintThrottled("OPTICALSTATE", "sensorState:" + String(sensorState) + " encoderValue:" + String(encoderValue));
    setCurrentPanel(DEFAULT_PANEL);  // Set the current panel only on the specified edge
  }
  encoderValue = encoder.read();  // Update encoder value at the beginning of each loop
}

void setup() {
  Serial.begin(115200);

  // Wi-Fi setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected to WiFi! ");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // REST API routes
  server.on("/getCurrentPanel", HTTP_GET, handleGetCurrentPanel);  // TODO renommer en virant les gets et les mots panel des actions
  server.on("/getDebug", HTTP_GET, handleGetDebug);
  server.on("/moveToPanel", HTTP_POST, handleMoveToPanel);
  server.on("/advancePanels", HTTP_POST, handleAdvancePanels);
  server.on("/calibrate", HTTP_GET, handleCalibrate);
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
  readSensors();
  processStateMachine();

  // Update sensors last state
  lastSensorState = sensorState;
#ifdef DEBUG_ENABLED
  serialPrintThrottled("ALL", buildDebugString());
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
