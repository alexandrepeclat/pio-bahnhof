#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <Encoder.h>

//TODO MECANIQUE : 
//- Faciliter l'ajustement de la roue optique
//  - en roue libre ?
//  - fixation par le haut ?
//  - fentes pour voir le trou ?
//  - ligne visuelle pour guider sur la roue ?
//- Aligner fente avec dent car ça coince mieux sur cette position
//- Revoir alignement dents et trou de l'encodeur ? car 1 dent = 1 panneau doit tomber pile dans une step pleine de l'encodeur
//- Permettre de découpler les roues ? dur
//- Trous de fixation corrects ? l'un était trop étroit
//- Trou de roue optique trop petit il semble
//- Voir comment fixer axe roue optique + rondelles
//- Voir comment fixer roue encodeur + rondelles
//- Prévoir plus de place pour pins capteur optique

//TODO SOFTWARE :
//- Sécurités si valeur hors norme, stoppe servo

// Pins configuration
#define SERVO_PIN D3
#define ENCODER_PIN_A D1      // CLK
#define ENCODER_PIN_B D2      // DT
#define OPTICAL_SENSOR_PIN D6 // Pin pour le capteur optique

// Constants
const int PANELS_COUNT = 62;    // Nombre total de panneaux
const int PULSES_PER_PANEL = 2; // Ajuste pour 4 impulsions par panneau
const bool SENSOR_STATE = LOW;  // Constante qui définit l'état du capteur optique (LOW ou HIGH) lorsqu'il est au panneau 0
const int DEFAULT_PANEL = 3;    // Panneau situé en position 0 selon capteur optique
// Servo configuration
Servo servo;
const int STOP_SPEED = 90; // Valeur pour arrêter le servo
const int RUN_SPEED = 120;  // Vitesse du servo pour avancer (91-180) 140 c'est bien 

// Globals
ESP8266WebServer server(80);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
int currentPanel = 0; // Panel actuel
int targetPanel = 0;  // Panneau cible

// Setup Wi-Fi
const char *ssid = "ap8F2EOjLm";
const char *password = "gsecumonwifiii123";

// Functions

  unsigned long lastPrintTime = 0;
void serialPrintThrottled(String message)
{
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime > 100)
  {
    Serial.println(message);
    lastPrintTime = currentTime;
  }
}

void sendHeaders()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
}

void handleGetDebug()
{
  sendHeaders();
  int currentPulses = abs(encoder.read());                                 // Compte total des impulsions
  currentPulses = currentPulses % (PANELS_COUNT * PULSES_PER_PANEL); // Convertit les impulsions en panneaux
  int targetPulses = targetPanel * PULSES_PER_PANEL;                  // Pulses correspondant au panneau cible
  currentPanel = currentPulses / PULSES_PER_PANEL;
  server.send(200, "text/plain", "Current panel: " + String(currentPanel) + ", Target panel: " + String(targetPanel) + ", Current pulses: " 
  + String(currentPulses) + ", Target pulses: " + String(targetPulses) + ", Optical: " + String(digitalRead(OPTICAL_SENSOR_PIN)));
}

void handleGetCurrentPanel()
{
  sendHeaders();
  server.send(200, "text/plain", String(currentPanel));
}

void handleMoveToPanel()
{
  sendHeaders();
  if (server.hasArg("panel"))
  {
    targetPanel = server.arg("panel").toInt();
    targetPanel %= PANELS_COUNT; // Assure que la cible est dans les limites
    server.send(200, "text/plain", "Moving to panel " + String(targetPanel));
  }
  else
  {
    server.send(400, "text/plain", "Missing 'panel' argument");
  }
}

void handleAdvancePanels()
{
  sendHeaders();
  if (server.hasArg("count"))
  {
    int advanceCount = server.arg("count").toInt();
    targetPanel = (currentPanel + advanceCount) % PANELS_COUNT;
    server.send(200, "text/plain", "From " + String(currentPanel) + ", Advancing " + String(advanceCount) + " panels to " + String(targetPanel));
  }
  else
  {
    server.send(400, "text/plain", "Missing 'count' argument");
  }
}

void handleSetCurrentPanel()
{
  sendHeaders();
  if (server.hasArg("panel"))
  {
    currentPanel = server.arg("panel").toInt();
    currentPanel %= PANELS_COUNT; // Assure que la valeur est dans les limites
    targetPanel = currentPanel;
    encoder.write(abs(currentPanel) * PULSES_PER_PANEL); //TODO Centraliser gestion direction de l'encodeur (les moins partout pas cool)
    server.send(200, "text/plain", "Current panel set to " + String(currentPanel));
  }
  else
  {
    server.send(400, "text/plain", "Missing 'panel' argument");
  }
}

void handleStop()
{
  sendHeaders();
  servo.write(STOP_SPEED);
  targetPanel = currentPanel;
  server.send(200, "text/plain", "Stopped. Current panel set to " + String(currentPanel));
}

// Fonction qui vérifie l'état du capteur optique
void checkOpticalSensor()
{
  int sensorState = digitalRead(OPTICAL_SENSOR_PIN);
  //Serial.println("Optical state: " + String(sensorState) + " currentPanel: " + String(currentPanel));
  digitalWrite(LED_BUILTIN, sensorState);

  if (sensorState == 0)
  {
    currentPanel = DEFAULT_PANEL; // Réinitialiser le panneau actuel à 0 lorsque le capteur détecte le panneau 0 //TODO à voir mais c'est pas nécessaire de setter ce compteur si on fait toujours le calcul dans la boucle du servo
    encoder.write(DEFAULT_PANEL * PULSES_PER_PANEL); // Réinitialise l'encodeur à 0


  //TODO VUE QUE MON ENCODEUR TOURNE A L'ENVERS, ça joue pas quand on sette le compteur à 3 positif 


    serialPrintThrottled("Panneau 0 détecté, currentPanel réinitialisé à 0");
  }
}



// Non-blocking move logic
void updateServoMovement()
{
   /*if (currentPanel == targetPanel)
   {
     return;
   }*/

  int currentPulses = abs(encoder.read());                                 // Compte total des impulsions
  currentPulses = currentPulses % (PANELS_COUNT * PULSES_PER_PANEL); // Convertit les impulsions en panneaux
  int targetPulses = targetPanel * PULSES_PER_PANEL;                  // Pulses correspondant au panneau cible
  currentPanel = currentPulses / PULSES_PER_PANEL; //TODO séparer boucle servo et encodeur ? 

  serialPrintThrottled("Current panel: " + String(currentPanel) + ", Target panel: " + String(targetPanel) + ", Current pulses: " 
  + String(currentPulses) + ", Target pulses: " + String(targetPulses) + ", Optical: " + String(digitalRead(OPTICAL_SENSOR_PIN)));
  // Vérifie si la position cible est atteinte
  if (currentPulses == targetPulses)
  {
    servo.write(STOP_SPEED); // Arrête le servo si la cible est atteinte
  }
  else
  {
    servo.write(RUN_SPEED); // Continue à faire tourner le servo
  }
}

void setup()
{
  Serial.begin(115200);

  // Wi-Fi setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected to WiFi! ");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Pin setup
  pinMode(OPTICAL_SENSOR_PIN, INPUT); // Configure le capteur optique en entrée

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
  servo.write(STOP_SPEED); // Ensure the servo starts stopped
}

void loop()
{
  server.handleClient(); // Handle incoming HTTP requests
  checkOpticalSensor();  // Vérifie l'état du capteur optique
  updateServoMovement(); // Manage the servo's movement
}
