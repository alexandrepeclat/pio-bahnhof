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

// TODO SOFTWARE :
//- Sécurités si valeur hors norme, stoppe servo
//- Remplacer service setCurrentPanel par un service calibration qui met l'état indéfini sur le panel courant (+ gérer panel courant indéfini tourne un tour jusqu'à déclencher l'optique)
//- Du coup démarrer en "indéfini" mais ne pas bouger, et à la première commande, aller jusqu'à l'optique au minimum
// TODO setTargetPanel ?

#define DEBUG_ENABLED 1

// Pins configuration
#define SERVO_PIN D3
#define ENCODER_PIN_A D1       // CLK
#define ENCODER_PIN_B D2       // DT
#define OPTICAL_SENSOR_PIN D6  // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;              // Nombre total de panneaux
const int PULSES_PER_PANEL = 2;           // Ajuste pour 4 impulsions par panneau
const bool OPTICAL_DETECTED_STATE = LOW;  // Constante qui définit l'état du capteur optique (LOW ou HIGH) lorsqu'il est au panneau 0
const int DEFAULT_PANEL = 3;              // Panel at optical sensor position
const int ENCODER_DIRECTION_SIGN = -1;

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

#ifdef DEBUG_ENABLED
std::map<String, String> lastDebugMessages;  // Map to store the last debug messages
#endif

// Setup Wi-Fi
const char* ssid = "ap8F2EOjLm";
const char* password = "gsecumonwifiii123";

int getCurrentPulses() {
  int pulses = encoder.read() * ENCODER_DIRECTION_SIGN;
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

void setCurrentPanel(int panel) {
  setCurrentPulses(panel * PULSES_PER_PANEL);
}

int getTargetPulses() {
  return targetPanel * PULSES_PER_PANEL;
}

String buildDebugJson(String message) {
  String debugJson = "{";
  debugJson += "\"message\":\"" + message + "\",";
  debugJson += "\"currentPanel\":" + String(getCurrentPanel()) + ",";
  debugJson += "\"targetPanel\":" + String(targetPanel) + ",";
  debugJson += "\"currentPulses\":" + String(getCurrentPulses()) + ",";
  debugJson += "\"targetPulses\":" + String(getTargetPulses()) + ",";
  debugJson += "\"rawPulses\":" + String(encoder.read()) + ",";
  debugJson += "\"optical\":" + String(digitalRead(OPTICAL_SENSOR_PIN));
  debugJson += "}";
  return debugJson;
}

#ifdef DEBUG_ENABLED
String buildDebugString() {
  String debugString = "currentPanel: " + String(getCurrentPanel()) +
                       " targetPanel:" + String(targetPanel) +
                       " currentPulses:" + String(getCurrentPulses()) +
                       " targetPulses:" + String(getTargetPulses()) +
                       " rawPulses:" + String(encoder.read()) +
                       " optical:" + String(digitalRead(OPTICAL_SENSOR_PIN)) +
                       " distance: " + String(getRemainingPulses()) +
                       " speed: " + String(calculateEaseOutSpeed());

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
    motorEnabled = true;          // Enable motor on move command
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
    motorEnabled = true;  // Enable motor on advance command
    server.send(200, "text/plain", buildDebugJson("From " + String(currentPanel) + ", Advancing " + String(advanceCount) + " panels to " + String(targetPanel)));
  } else {
    server.send(400, "text/plain", "Missing 'count' argument");
  }
}

void handleSetCurrentPanel() {
  sendHeaders();
  if (server.hasArg("panel")) {
    int value = server.arg("panel").toInt();
    setCurrentPanel(value);  // Will set encoder accordingly and keep value within limits
    int currentPanel = getCurrentPanel();
    targetPanel = currentPanel;
    server.send(200, "text/plain", buildDebugJson("Current panel set to " + String(currentPanel)));
  } else {
    server.send(400, "text/plain", "Missing 'panel' argument");
  }
}

void handleStop() {
  sendHeaders();
  setMotorSpeed(0);      // Use the centralized function to stop the motor
  motorEnabled = false;  // Disable motor on stop command
  int currentPanel = getCurrentPanel();
  targetPanel = currentPanel;
  server.send(200, "text/plain", buildDebugJson("Stopped. Current panel set to " + String(currentPanel)));
}

int getRemainingPulses() {
  int currentPulses = getCurrentPulses();
  int targetPulses = getTargetPulses();
  int totalPulses = PANELS_COUNT * PULSES_PER_PANEL;
  int distance = (targetPulses - currentPulses + totalPulses) % totalPulses;  // Calcule la distance en avance (horaire) //TODO on doit tjrs avoir du positif ici asserter ça
  return distance;
}

float easeOutExpo(float x) {
  return (x == 1) ? 1 : 1 - pow(2, -10 * x);  // Calcul de l'expo //TODO https://easings.net/
}

float calculateEaseOutSpeed() {
  int remainingPulses = getRemainingPulses();

  if (remainingPulses < PULSES_PER_PANEL) {  // TODO parfois j'ai un décalage de 1... peut être même problème que dans l'ancien code commenté...
    return 0.f;                              // Normalized speed for stop
  } else if (remainingPulses == PULSES_PER_PANEL) {
    return 0.7f;
  } else if (remainingPulses == PULSES_PER_PANEL * 2) {
    return 0.8f;
  } else if (remainingPulses == PULSES_PER_PANEL * 3) {
    return 0.9f;
  } else {
    return 1.0f;
  }
}

// Non-blocking move logic
void updateServoMovement() {
  if (!motorEnabled) {
    setMotorSpeed(0);
    return;
  }
  float speed = calculateEaseOutSpeed();
  setMotorSpeed(speed);  // Set the motor speed using the centralized function
}

// Fonction qui vérifie l'état du capteur optique
void checkOpticalSensor() {
  int sensorState = digitalRead(OPTICAL_SENSOR_PIN);
  if (sensorState == OPTICAL_DETECTED_STATE) {
    setCurrentPanel(DEFAULT_PANEL);  // Réinitialiser le panneau actuel à 0 lorsque le capteur détecte le panneau 0 //TODO à voir mais c'est pas nécessaire de setter ce compteur si on fait toujours le calcul dans la boucle du servo

    // TODO au cas où l'optique détecte le panneau sur plusieurs steps, ne pas setter le current panel à 0 tant qu'on est pas au moins au 2ème panel
    //- Sinon ça détecte l'optique au step 0
    //- ça met l'encodeur à 0
    //- ça détecte au step 1
    //- ça remet l'encodeur à 0
    //- ça détecte pas au step 1 suivant (qui serait le 2) et on a un décalage
    //- à voir aussi si nécessite pas un ajustement mécanique

    serialPrintThrottled("OPTICAL", "Panneau 0 détecté, currentPanel réinitialisé à 0");
  }
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
  server.on("/getCurrentPanel", HTTP_GET, handleGetCurrentPanel);
  server.on("/getDebug", HTTP_GET, handleGetDebug);
  server.on("/moveToPanel", HTTP_POST, handleMoveToPanel);
  server.on("/advancePanels", HTTP_POST, handleAdvancePanels);
  server.on("/setCurrentPanel", HTTP_POST, handleSetCurrentPanel);
  server.on("/stop", HTTP_GET, handleStop);
  server.begin();
  Serial.println("HTTP server started");

  // Servo setup
  servo.attach(SERVO_PIN);
  setMotorSpeed(0);  // Ensure the servo starts stopped using the centralized function

  // Optical sensor setup
  pinMode(OPTICAL_SENSOR_PIN, INPUT);  // Configure le capteur optique en entrée
}

void loop() {
  server.handleClient();  // Handle incoming HTTP requests
  checkOpticalSensor();   // Vérifie l'état du capteur optique
  updateServoMovement();  // Manage the servo's movement
#ifdef DEBUG_ENABLED
  serialPrintThrottled("ALL", buildDebugString());
#endif
}

void setMotorSpeed(float normalizedSpeed) {
  normalizedSpeed = constrain(normalizedSpeed, 0.f, 1.f);
  int speed = map(normalizedSpeed * 100, 0, 100, STOP_SPEED, RUN_SPEED);
  servo.write(speed);
}
