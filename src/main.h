#include <ESP8266WebServer.h>
#include <vector>

enum AppState {
  EMERGENCY_STOPPED,
  STOPPED,
  AUTO_CALIBRATING,
  MOVING_TO_TARGET,
  CALIBRATING
};

extern ESP8266WebServer server;
extern volatile AppState currentState;
extern volatile bool calibrated;
extern int targetPulses;
extern const int PULSES_COUNT;
extern const int PANELS_COUNT;
extern const int PULSES_PER_PANEL;

void sendHeaders();
void setMotorSpeed(float normalizedSpeed);
String buildDebugJson(String message);
int getCurrentPulses();
int getCurrentPanel();
void setTargetPulses(int pulses);
int getTargetPulses();
int getTargetPanel();
void setTargetPanel(int panel);
void setCurrentState(AppState newState);

class Command {
 public:
  virtual String getName() = 0;
  virtual std::vector<AppState> getAppStates() = 0;
  virtual void handleRest() = 0;
  virtual void handleSerial(String params) = 0;
  virtual String execute() = 0;
};
