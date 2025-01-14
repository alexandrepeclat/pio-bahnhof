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

enum AppState {
  STOPPED,
  MOVING_TO_TARGET,
  CALIBRATING
};

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
//- Trous des vis encodeur pas assez profonds
//- Voir comment fixer axe roue optique + rondelles
//- Voir comment fixer roue encodeur + rondelles
//- Prévoir plus de place pour pins capteur optique
//- Roue libre à insérer une fois que tout est en place pour permettre de libérer le mécanisme en manuel ?

// TODO SOFTWARE :
//- Sécurités si valeur hors norme, stoppe servo
//- Remplacer service setCurrentPanel par un service calibration qui met l'état indéfini sur le panel courant (+ gérer panel courant indéfini tourne un tour jusqu'à déclencher l'optique)
//- Du coup démarrer en "indéfini" mais ne pas bouger, et à la première commande, aller jusqu'à l'optique au minimum
// TODO setTargetPanel ?
// TODO ? stocker la valeur de l'encodeur dans variable et la remettre à zéro avec l'optique, mais laisser la valeur de l'encodeur originale. Permettrait de voir s'il y a un décalage au début et s'il y a un décalage progressif du à ratés en raison d'une boucle trop lente
// - ou plutôt afficher la valeur de l'encodeur brute lors du passage de l'optique
// - et faire une variable previousPulses pour voir si une boucle ne loupe pas un step
// - vérifier qu'on a bien atteint le nombre de steps max au moment du passage et afficher un warning sinon
// TODO s'assurer que tous les "getters" utilisés notamment dans les debug ne change pas les états ou valeurs :-)
// TODO à voir la fonction de calibration ça me plait moyen : on doit gérer l'optique à plusieurs endroits (soit abstraire la notion rising/falling/low/high) (soit voir si pas mieux où on donne simplement le panel à atteindre et on laisse calibrer automatiquement, ce qui évite d'avoir une vitesse de calibration différente de la vitesse en opération)
// TODO état ERROR ? en même temps le flag reste nécessaire car il garantit qu'on peut pas setter à nouveau autre chose via transition ou autre... mais la boucle continue de tourner et calculer des trucs pour rien...
// TODO stocker des trucs en EEPROM ? genre le panel courant pour redémarrer dessus ?
// TODO stocker des options en EEPROM ? genre s'il faut auto-calibrer au démarrage ou pas ? la vitesse de rotation ? le panel par défaut si démontage + remontage sans devoir reflasher ?
// TODO Vérifier les valeurs passées depuis l'api un peu mieux en terme de limites et type
// TODO reconnexion wifi si coupure
// TODO go to rising/falling edges via API ?
//TODO détecter si blocage (en fonction de l'encodeur et du temps écoulé) et emergencystop
//TODO garder les N derniers messages (erreur ou warning...)

/*
TODO à voir car j'ai un décalage d'une pulse. je sais pas si c'est la position du front montant de l'optique ou autre chose... pose surtout problème pour le panneau 30 (praha)
mais il y a toujours ce décalage...
- pulse = 0, panel = 0, panel réel = 0
- pulse = 1, panel = 0, panel réel = 1
- pulse = 2, panel = 1, panel réel = 1
- pulse = 3, panel = 1, panel réel = 2
- pulse = 4, panel = 2, panel réel = 2
- pulse = 5, panel = 2, panel réel = 3
- pulse = 6, panel = 3, panel réel = 3
...
pas sûr que l'offset au moment du passage optique soit une bonne solution... est-ce que ça a un effet quand on sette un target ? à réfléchir. Aussi si on met un offset de 1 au départ, la pulse 0 n'existe pas, du coup vu qu'on fait modulo pulses totales on devrait faire + offset aussi non ?
j'ai aussi un décalage de 1 panel au moment de la calibration / passage optique. on dit que le panel par défaut est le 40, mais en fait on est au 41. Pourtant la réalité correspond
idée: considérer de trigger la mise à 0 lors du prochain changement d'encodeur après un front montant de l'optique ? comme ça c'est calqué sur les steps de l'encodeur. Mais en même temps, on remet à 0 l'encodeur lui-même donc le step 1 sera de toute manière calqué sur l'encodeur....:-P
 
 
 MEILLEURE IDEE : voir description des valeurs d'offset. avancer jusqu'à l'optique rising edge, voir si on est sur un step pair/impair et ajuster l'offset optique.
 MEILLEURE IDEE 2 : un seul offset est nécessaire au moment du passage de l'optique. Pour le définir, il suffit de tester successivement 0,1,2,3... pour voir si ça atterit sur un panel de manière correcte (centré) on peut même corriger le no de panel dans un second temps
 
 */

#define DEBUG_ENABLED 1

// Pins configuration
#define SERVO_PIN D3
#define ENCODER_PIN_A D1       // CLK
#define ENCODER_PIN_B D2       // DT
#define OPTICAL_SENSOR_PIN D6  // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;               // Nombre total de panneaux
const int PULSES_PER_PANEL = 2;            // Ajuste pour 4 impulsions par panneau
const int DEFAULT_PANEL = 40;              // Panel at optical sensor position
const int DEFAULT_PANEL_PULSE_OFFSET = 1;  // Optical sensor is detected at nth pulse of the default panel //TODO Directement calculer un DEFAULT_PULSE via les deux variables et faire en sorte que ce soit dans les limites [0-PULSES_TOTAL]
const int ENCODER_DIRECTION_SIGN = -1;
const int TARGET_PULSE_OFFSET = 1;  // When going to target, go to the nth pulse of the target panel //TODO chuis tjrs pas sur que ce soit utile... au moment du passage optique, suffit de lui faire croire qu'il est en avant ou en arrière et ça devrait faire le job pareil
static_assert(TARGET_PULSE_OFFSET < PULSES_PER_PANEL, "OFFSET must be less than PULSES_PER_PANEL");
static_assert(TARGET_PULSE_OFFSET >= 0, "OFFSET must be non-negative");

#define DETECTED_STATE LOW  // Define the detected state for the optical sensor

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
  return getCurrentPulses() / PULSES_PER_PANEL;
}

void setCurrentPulses(int pulses) {
  pulses = pulses % (PANELS_COUNT * PULSES_PER_PANEL);
  encoder.write(pulses * ENCODER_DIRECTION_SIGN);
}

void setTargetPanel(int panel) {
  targetPulses = (panel * PULSES_PER_PANEL) + TARGET_PULSE_OFFSET;
}

int getTargetPulses() {
  return targetPulses;
}

int getTargetPanel() {
  return targetPulses / PULSES_PER_PANEL;
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
    panel %= PANELS_COUNT;  // Ensure the target is within bounds
    setTargetPanel(panel);
    setCurrentState(MOVING_TO_TARGET);  // Set motor mode to go to target
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
    setTargetPanel((currentPanel + advanceCount) % PANELS_COUNT);  // TODO faire les modulos dans les setters (y compris setTargetPulses ce qui serait sa raison d'être)
    setCurrentState(MOVING_TO_TARGET);                             // Set motor mode to go to target
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
    targetPulses = (currentPulses + pulseCount) % (PANELS_COUNT * PULSES_PER_PANEL);  // TODO faire les modulos dans les setters (y compris setTargetPulses ce qui serait sa raison d'être)
    setCurrentState(MOVING_TO_TARGET);                                                // Set motor mode to go to target
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
  int totalPulses = PANELS_COUNT * PULSES_PER_PANEL;
  int distance = (targetPulses - currentPulses + totalPulses) % totalPulses;  // Calcule la distance en avance (horaire)
  return distance;
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

void evaluateStateTransitions() {
  switch (currentState) {
    case STOPPED:
      // Nothing to do, wait for command
      break;
    case MOVING_TO_TARGET: {
      if (isTargetPanelReached()) {
        currentState = STOPPED;
      }
      break;
    }
    case CALIBRATING: {
      if (isOpticalEdgeDetected) {
        currentState = STOPPED;
      }
      break;
    }
  }
}

void processStateActions() {
  switch (currentState) {
    case STOPPED:
      setMotorSpeed(0);
      // setTargetPulses(getCurrentPulses()); //TODO Nécessaire pour pas que ça reparte après calibration ?
      break;
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
  lastSensorState = sensorState;  // TODO rendre inaccessible en dehors ?

  // Read encoder value, reset if edge was detected
  if (isOpticalEdgeDetected) {
    serialPrintThrottled("OPTICALSTATE", "Optical edge detected");
    setCurrentPulses(DEFAULT_PANEL * PULSES_PER_PANEL + DEFAULT_PANEL_PULSE_OFFSET);
  }
  encoderValue = encoder.read();
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
  server.on("/advancePulses", HTTP_POST, handleAdvancePulses);
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
