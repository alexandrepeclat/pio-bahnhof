#include <ESP8266WebServer.h>
#include <vector>
#include "main.h"  // Include the main header for shared declarations

class StopCommand : public Command {
 public:
  String getName() override {
    return "stop";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    String message = execute();
    server.send(200, "text/plain", buildDebugJson(message));
  }

  void handleSerial(String params) override {
    String response = execute();
    Serial.println(response);
  }

  String execute() override {
    setMotorSpeed(0);
    setCurrentState(STOPPED);
    int currentPanel = getCurrentPanel();
    setTargetPanel(currentPanel);
    return "Stopped. Current panel set to " + String(currentPanel);
  }
};

class PanelCommand : public Command {
 public:
  String getName() override {
    return "panel";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    server.send(200, "text/plain", String(execute()));
  }

  void handleSerial(String params) override {
    Serial.println("Current panel: " + String(execute()));
  }

  String execute() override {
    return String(getCurrentPanel());
  }
};

class DebugCommand : public Command {
 public:
  String getName() override {
    return "debug";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    server.send(200, "text/plain", execute());
  }

  void handleSerial(String params) override {
    Serial.println(execute());
  }

  String execute() override {
    return buildDebugJson("Debug request");
  }
};

class MoveToPanelCommand : public Command {
 public:
  String getName() override {
    return "moveToPanel";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    if (server.hasArg("panel")) {
      int panel = server.arg("panel").toInt();
      String message = execute(panel);
      server.send(200, "text/plain", buildDebugJson(message));
    } else {
      server.send(400, "text/plain", "Missing 'panel' argument");
    }
  }

  void handleSerial(String params) override {
    int panel = params.toInt();
    String response = execute(panel);
    Serial.println(response);
  }

  String execute(int panel) {
    panel %= PANELS_COUNT;
    setTargetPanel(panel);
    setCurrentState(MOVING_TO_TARGET);
    return "Moving to panel " + String(getTargetPanel());
  }
};

class AdvancePanelsCommand : public Command {
 public:
  String getName() override {
    return "advancePanels";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    if (server.hasArg("count")) {
      int count = server.arg("count").toInt();
      String message = execute(count);
      server.send(200, "text/plain", buildDebugJson(message));
    } else {
      server.send(400, "text/plain", "Missing 'count' argument");
    }
  }

  void handleSerial(String params) override {
    int count = params.toInt();
    String response = execute(count);
    Serial.println(response);
  }

  String execute(int count) {
    int currentPanel = getCurrentPanel();
    setTargetPanel((currentPanel + count) % PANELS_COUNT);
    setCurrentState(MOVING_TO_TARGET);
    return "From " + String(currentPanel) + ", Advancing " + String(count) + " panels to " + String(getTargetPanel());
  }
};

class AdvancePulsesCommand : public Command {
 public:
  String getName() override {
    return "advancePulses";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    if (server.hasArg("count")) {
      int pulseCount = server.arg("count").toInt();
      String message = execute(pulseCount);
      server.send(200, "text/plain", buildDebugJson(message));
    } else {
      server.send(400, "text/plain", "Missing 'count' argument");
    }
  }

  void handleSerial(String params) override {
    int pulseCount = params.toInt();
    String response = execute(pulseCount);
    Serial.println(response);
  }

  String execute(int pulseCount) {
    int currentPulses = getCurrentPulses();
    setTargetPulses((currentPulses + pulseCount) % PULSES_COUNT);
    setCurrentState(MOVING_TO_TARGET);
    return "Advancing " + String(pulseCount) + " pulses to " + String(targetPulses);
  }
};

class CalibrateCommand : public Command {
 public:
  String getName() override {
    return "calibrate";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    String message = execute();
    server.send(200, "text/plain", buildDebugJson(message));
  }

  void handleSerial(String params) override {
    String response = execute();
    Serial.println(response);
  }

  String execute() override {
    setCurrentState(CALIBRATING);
    calibrated = false;
    return "Calibration started. Rotating until optical sensor edge is detected.";
  }
};

class ResetCommand : public Command {
 public:
  String getName() override {
    return "reset";
  }

  std::vector<AppState> getAppStates() override {
    return {STOPPED, MOVING_TO_TARGET, CALIBRATING, AUTO_CALIBRATING};
  }

  void handleRest() override {
    sendHeaders();
    String message = execute();
    server.send(200, "text/plain", buildDebugJson(message));
  }

  void handleSerial(String params) override {
    String response = execute();
    Serial.println(response);
  }

  String execute() override {
    setMotorSpeed(0);
    calibrated = false;
    errorFlag = false;
    errorMessage = "";
    setCurrentState(STOPPED);
    return "Reset";
  }
};
