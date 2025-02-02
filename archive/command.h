/*
Tentative de classe générique pour gérer les commandes depuis le Serial et le REST API.
- Trop tordu et complexe, trop overkill pour le besoin
- Validation des paramètres trop générique
- Difficile à suivre en lecture de code statique
*/


#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <unordered_map>
#include <functional>
#include <tuple>
#include <variant>
#include <string>

// Alias pour une commande avec plusieurs arguments
using CommandVariant = std::variant<
    std::function<void()>,
    std::function<void(int)>,
    std::function<void(String, int)>
>;

// Structure de commande
struct CommandInfo {
    std::vector<String> params;
    CommandVariant action;
};

// Map des commandes
std::unordered_map<std::string, CommandInfo> commands;

// Fonction pour appeler dynamiquement une commande avec plusieurs arguments
template <typename Func, typename... Args, size_t... I>
void invokeWithArgs(Func func, std::tuple<Args...> args, std::index_sequence<I...>) {
    func(std::get<I>(args)...);
}

// Enregistre une commande REST + Serial
template <typename Func>
void registerCommand(const ESP8266WebServer& server, const String& name, HTTPMethod method, Func func, std::vector<String> params = {}) {
    commands[std::string(name.c_str())] = {params, func};

    server.on(("/" + name), method, [name, func, params, &server]() {
        if constexpr (std::is_same_v<Func, std::function<void()>>) {
            func();
        } else {
            std::tuple<String, int> parsedArgs;  // Stocke les arguments
            bool valid = true;

            if (params.size() > 0) std::get<0>(parsedArgs) = server.arg(params[0]);
            if (params.size() > 1) {
                String arg1 = server.arg(params[1]);
                if (arg1.length() > 0) std::get<1>(parsedArgs) = arg1.toInt();
                else valid = false;
            }

            if (valid) {
                invokeWithArgs(func, parsedArgs, std::index_sequence_for<String, int>{});
            } else {
                server.send(400, "text/plain", "Bad Request: Invalid parameters");
            }
        }
    });
}

// Décodage des commandes Serial
void readSerialCommand() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        int space1 = input.indexOf(' ');
        int space2 = input.indexOf(' ', space1 + 1);

        String name = (space1 == -1) ? input : input.substring(0, space1);
        String arg1 = (space1 == -1) ? "" : input.substring(space1 + 1, space2);
        String arg2 = (space2 == -1) ? "" : input.substring(space2 + 1);

        auto it = commands.find(std::string(name.c_str()));
        if (it != commands.end()) {
            if (auto* noArgFunc = std::get_if<std::function<void()>>(&it->second.action)) {
                (*noArgFunc)();
            } else if (auto* intFunc = std::get_if<std::function<void(int)>>(&it->second.action)) {
                (*intFunc)(arg1.toInt());
            } else if (auto* stringIntFunc = std::get_if<std::function<void(String, int)>>(&it->second.action)) {
                (*stringIntFunc)(arg1, arg2.toInt());
            }
        } else {
            Serial.println("Erreur Serial: commande inconnue");
        }
    }
}