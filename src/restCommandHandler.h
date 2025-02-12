#include <ESPAsyncWebServer.h>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <type_traits> //TODO passer en revue les include inutiles
#include <vector>

class RestCommandHandler {
 public:
  // Constructeur qui prend une référence à l'AsyncWebServer
  RestCommandHandler(AsyncWebServer& server)
      : _server(server) {}

  // Enregistrer une commande REST sans argument
  void registerCommand(const String& endpoint, WebRequestMethod method, std::function<String()> callback) {
    _server.on(("/" + endpoint).c_str(), method, [this, callback](AsyncWebServerRequest *request) {
      // Appel du callback
      String response = callback();
      request->send(200, "text/plain", response);
    });
    _routes.push_back({endpoint, method, {}});
  }

  // Enregistrer une commande REST avec 1 argument
  template <typename Arg1>
  void registerCommand(const String& endpoint, WebRequestMethod method, const std::vector<String>& paramNames, std::function<String(Arg1)> callback) {
    assert(paramNames.size() == 1);

    _server.on(("/" + endpoint).c_str(), method, [this, callback, paramNames, method](AsyncWebServerRequest *request) {
      // Vérification des arguments
      bool isPost = (method == HTTP_POST);
      if (!request->hasParam(paramNames[0], isPost)) {
        request->send(400, "text/plain", "Bad Request: missing argument");
        return;
      }
      String param = request->getParam(paramNames[0], isPost)->value();
      if (param.length() == 0) {
        request->send(400, "text/plain", "Bad Request: missing argument");
        return;
      }

      // Conversion et appel du callback
      Arg1 arg1;
      if (!convertArgument<Arg1>(param, arg1)) {
        request->send(400, "text/plain", "Erreur de conversion pour l'argument " + paramNames[0] + " : " + param);
        return;
      }
      String response = callback(arg1);
      request->send(200, "text/plain", response);
    });
    _routes.push_back({endpoint, method, paramNames});
  }

  // Enregistrer une commande REST avec 2 arguments
  template <typename Arg1, typename Arg2>
  void registerCommand(const String& endpoint, WebRequestMethod method, const std::vector<String>& paramNames, std::function<String(Arg1, Arg2)> callback) {
    assert(paramNames.size() == 2);

    _server.on(("/" + endpoint).c_str(), method, [this, callback, paramNames, method](AsyncWebServerRequest *request) {
      // Vérification des arguments
      bool isPost = (method == HTTP_POST);
      if (!request->hasParam(paramNames[0], isPost) || !request->hasParam(paramNames[1], isPost)) {
        request->send(400, "text/plain", "Bad Request: missing argument");
        return;
      }
      String param1 = request->getParam(paramNames[0], isPost)->value();
      String param2 = request->getParam(paramNames[1], isPost)->value();
      if (param1.length() == 0 || param2.length() == 0) {
        request->send(400, "text/plain", "Bad Request: missing argument");
        return;
      }

      // Conversion et appel du callback
      Arg1 arg1;
      Arg2 arg2;
      if (!convertArgument<Arg1>(param1, arg1)) {
        request->send(400, "text/plain", "Erreur de conversion pour l'argument " + paramNames[0] + " : " + param1);
        return;
      }
      if (!convertArgument<Arg2>(param2, arg2)) {
        request->send(400, "text/plain", "Erreur de conversion pour l'argument " + paramNames[1] + " : " + param2);
        return;
      }
      String response = callback(arg1, arg2);
      request->send(200, "text/plain", response);
    });
    _routes.push_back({endpoint, method, paramNames});
  }

  // Enregistrer une commande REST avec 3 arguments
  template <typename Arg1, typename Arg2, typename Arg3>
  void registerCommand(const String& endpoint, WebRequestMethod method, const std::vector<String>& paramNames, std::function<String(Arg1, Arg2, Arg3)> callback) {
    assert(paramNames.size() == 3);

    _server.on(("/" + endpoint).c_str(), method, [this, callback, paramNames, method](AsyncWebServerRequest *request) {
      // Vérification des arguments
      bool isPost = (method == HTTP_POST);
      if (!request->hasParam(paramNames[0], isPost) || !request->hasParam(paramNames[1], isPost) || !request->hasParam(paramNames[2], isPost)) {
        request->send(400, "text/plain", "Bad Request: missing argument");
        return;
      }
      String param1 = request->getParam(paramNames[0], isPost)->value();
      String param2 = request->getParam(paramNames[1], isPost)->value();
      String param3 = request->getParam(paramNames[2], isPost)->value();
      if (param1.length() == 0 || param2.length() == 0 || param3.length() == 0) {
        request->send(400, "text/plain", "Bad Request: missing argument");
        return;
      }

      // Conversion et appel du callback
      Arg1 arg1;
      Arg2 arg2;
      Arg3 arg3;
      if (!convertArgument<Arg1>(param1, arg1)) {
        request->send(400, "text/plain", "Erreur de conversion pour l'argument " + paramNames[0] + " : " + param1);
        return;
      }
      if (!convertArgument<Arg2>(param2, arg2)) {
        request->send(400, "text/plain", "Erreur de conversion pour l'argument " + paramNames[1] + " : " + param2);
        return;
      }
      if (!convertArgument<Arg3>(param3, arg3)) {
        request->send(400, "text/plain", "Erreur de conversion pour l'argument " + paramNames[2] + " : " + param3);
        return;
      }
      String response = callback(arg1, arg2, arg3);
      request->send(200, "text/plain", response);
    });
    _routes.push_back({endpoint, method, paramNames});
  }

  void handleClient() {
    // No need to handle client in AsyncWebServer
  }

  // Retrieve the list of API routes with method and parameters
  String getRoutesList() {
    String list = "[";
    for (const auto& route : _routes) {
      list += "{";
      list += "\"endpoint\":\"/" + route.endpoint + "\",";
      list += "\"method\":\"" + methodToString(route.method) + "\",";
      list += "\"params\":[";
      for (const auto& param : route.params) {
        list += "\"" + param + "\",";
      }
      if (!route.params.empty()) {
        list.remove(list.length() - 1);  // Remove trailing comma
      }
      list += "]";
      list += "},";
    }
    if (!_routes.empty()) {
      list.remove(list.length() - 1);  // Remove trailing comma
    }
    list += "]";
    return list;
  }

 private:
  AsyncWebServer& _server;  // Référence au serveur pour l'enregistrement des routes

  struct Route {
    String endpoint;
    WebRequestMethod method;
    std::vector<String> params;
  };

  std::vector<Route> _routes;  // List of registered routes

  // Conversion des arguments en fonction de leur type
  template <typename T>
  bool convertArgument(const String& arg, T& outValue) {
    std::istringstream iss(arg.c_str());

    if constexpr (std::is_same_v<T, int>) {
      int temp;
      if (!(iss >> temp))
        return false;  // Vérifie la conversion
      if (!iss.eof())
        return false;  // Vérifie qu'il n'y a pas de caractères restants
      outValue = temp;
    } else if constexpr (std::is_same_v<T, float>) {
      float temp;
      if (!(iss >> temp))
        return false;  // Vérifie la conversion
      if (!iss.eof())
        return false;  // Vérifie qu'il n'y a pas de caractères restants
      outValue = temp;
    } else {
      outValue = arg;  // Pour les strings, pas besoin de conversion
    }

    return true;  // Conversion réussie
  }

  // Convert WebRequestMethod to String
  String methodToString(WebRequestMethod method) {
    switch (method) {
      case HTTP_GET:
        return "GET";
      case HTTP_POST:
        return "POST";
      case HTTP_PUT:
        return "PUT";
      case HTTP_PATCH:
        return "PATCH";
      case HTTP_DELETE:
        return "DELETE";
      case HTTP_OPTIONS:
        return "OPTIONS";
      default:
        return "UNKNOWN";
    }
  }
};
