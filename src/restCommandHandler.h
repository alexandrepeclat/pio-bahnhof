#include <ESP8266WebServer.h>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

class RestCommandHandler {
 public:
  // Constructeur qui prend une référence à l'ESP8266WebServer
  RestCommandHandler(ESP8266WebServer& server)
      : _server(server) {}

  // Enregistrer une commande REST sans argument
  void registerCommand(const String& endpoint, HTTPMethod method, std::function<String()> callback) {
    _server.on("/" + endpoint, method, [this, callback, method]() {
      sendHeaders();
      // Vérifier si la méthode correspond
      if (_server.method() != method) {
        _server.send(405, "text/plain", "Method Not Allowed");
        return;
      }

      // Vérifier qu'il n'y a pas d'arguments
      if (_server.args() > 0) {
        _server.send(400, "text/plain", "Bad Request: too many arguments");
        return;
      }

      // Appel du callback
      String response = callback();
      _server.send(200, "text/plain", response);
    });
  }

  // Enregistrer une commande REST avec 1 argument
  template <typename Arg1>
  void registerCommand(const String& endpoint, HTTPMethod method, const std::vector<String>& args, std::function<String(Arg1)> callback) {
    _server.on("/" + endpoint, method, [this, callback, args, method]() {
      sendHeaders();
      // Vérifier si la méthode correspond
      if (_server.method() != method) {
        _server.send(405, "text/plain", "Method Not Allowed");
        return;
      }

      // Vérification des arguments
      if (args.size() != 1) {
        _server.send(400, "text/plain", "Bad Request: incorrect number of arguments");
        return;
      }
      String param = _server.arg(args[0]);
      if (param.length() == 0) {
        _server.send(400, "text/plain", "Bad Request: missing argument");
        return;
      }

      // Conversion et appel du callback
      Arg1 arg1 = convertArgument<Arg1>(param.c_str());
      String response = callback(arg1);
      _server.send(200, "text/plain", response);
    });
  }

  // Enregistrer une commande REST avec 2 arguments
  template <typename Arg1, typename Arg2>
  void registerCommand(const String& endpoint, HTTPMethod method, const std::vector<String>& args, std::function<String(Arg1, Arg2)> callback) {
    _server.on("/" + endpoint, method, [this, callback, args, method]() {
      sendHeaders();
      if (_server.method() != method) {
        _server.send(405, "text/plain", "Method Not Allowed");
        return;
      }

      if (args.size() != 2) {
        _server.send(400, "text/plain", "Bad Request: incorrect number of arguments");
        return;
      }

      String param1 = _server.arg(args[0]);
      String param2 = _server.arg(args[1]);
      if (param1.length() == 0 || param2.length() == 0) {
        _server.send(400, "text/plain", "Bad Request: missing argument");
        return;
      }

      // Conversion et appel du callback
      Arg1 arg1 = convertArgument<Arg1>(param1.c_str());
      Arg2 arg2 = convertArgument<Arg2>(param2.c_str());
      String response = callback(arg1, arg2);
      _server.send(200, "text/plain", response);
    });
  }

  // Enregistrer une commande REST avec 3 arguments
  template <typename Arg1, typename Arg2, typename Arg3>
  void registerCommand(const String& endpoint, HTTPMethod method, const std::vector<String>& args, std::function<String(Arg1, Arg2, Arg3)> callback) {
    _server.on("/" + endpoint, method, [this, callback, args, method]() {
      sendHeaders();
      if (_server.method() != method) {
        _server.send(405, "text/plain", "Method Not Allowed");
        return;
      }

      if (args.size() != 3) {
        _server.send(400, "text/plain", "Bad Request: incorrect number of arguments");
        return;
      }

      String param1 = _server.arg(args[0]);
      String param2 = _server.arg(args[1]);
      String param3 = _server.arg(args[2]);
      if (param1.length() == 0 || param2.length() == 0 || param3.length() == 0) {
        _server.send(400, "text/plain", "Bad Request: missing argument");
        return;
      }

      // Conversion et appel du callback
      Arg1 arg1 = convertArgument<Arg1>(param1.c_str());
      Arg2 arg2 = convertArgument<Arg2>(param2.c_str());
      Arg3 arg3 = convertArgument<Arg3>(param3.c_str());
      String response = callback(arg1, arg2, arg3);
      _server.send(200, "text/plain", response);
    });
  }

  void handleClient() {
    _server.handleClient();  // Handle incoming HTTP requests
  }

 private:
  ESP8266WebServer& _server;  // Référence au serveur pour l'enregistrement des routes

  void sendHeaders() {
    _server.sendHeader("Access-Control-Allow-Origin", "*");
  }

  // Conversion des arguments en fonction de leur type
  template <typename T>
  T convertArgument(const char* arg) {
    if constexpr (std::is_same_v<T, int>) {
      return atoi(arg);
    } else if constexpr (std::is_same_v<T, float>) {
      return atof(arg);
    } else {
      return String(arg).c_str();  // pour le cas des strings
    }
  }
};
