#include <Arduino.h>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

// TODO factoriser tout ce code ?
// TODO Mettre en commun les conversions d'argument et les appels au callback définitif ? 
// TODO Reprendre les conversion sur restHandler
// TODO utiliser les string arduino ?

// Classe pour gérer l'enregistrement et l'appel des commandes
class SerialCommandHandler {
 public:
  // Enregistrer une commande sans argument
  void registerCommand(const std::string& command, std::function<String()> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<std::string>& args) {
      Serial.println("args size: " + String(args.size()));
      if (!args.empty()) {
        Serial.println("Erreur: trop d'arguments.");
        return;
      }
      String response = callback();
      Serial.println(response);
    };
  }

  // Enregistrer une commande avec 1 argument
  template <typename Arg1>
  void registerCommand(const std::string& command, std::function<String(Arg1)> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<std::string>& args) {
      if (args.size() != 1) {
        Serial.println("Erreur: mauvais nombre d'arguments.");
        return;
      }
      Arg1 arg1;
      if (!convertArgument<Arg1>(args[0], arg1)) {
        Serial.println("Erreur de conversion pour l'argument 1: " + String(args[0].c_str()));
        return;
      }
      String response = callback(arg1);
      Serial.println(response);
    };
  }

  // Enregistrer une commande avec 2 arguments
  template <typename Arg1, typename Arg2>
  void registerCommand(const std::string& command, std::function<String(Arg1, Arg2)> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<std::string>& args) {
      if (args.size() != 2) {
        Serial.println("Erreur: mauvais nombre d'arguments.");
        return;
      }
      Arg1 arg1;
      Arg2 arg2;
      if (!convertArgument<Arg1>(args[0], arg1)) {
        Serial.println("Erreur de conversion pour l'argument 1: " + String(args[0].c_str()));
        return;
      }
      if (!convertArgument<Arg2>(args[1], arg2)) {
        Serial.println("Erreur de conversion pour l'argument 2: " + String(args[1].c_str()));
        return;
      }
      String response = callback(arg1, arg2);
      Serial.println(response);
    };
  }

  // Enregistrer une commande avec 3 arguments
  template <typename Arg1, typename Arg2, typename Arg3>
  void registerCommand(const std::string& command, std::function<String(Arg1, Arg2, Arg3)> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<std::string>& args) {
      if (args.size() != 3) {
        Serial.println("Erreur: mauvais nombre d'arguments.");
        return;
      }
      Arg1 arg1;
      Arg2 arg2;
      Arg3 arg3;
      if (!convertArgument<Arg1>(args[0], arg1)) {
        Serial.println("Erreur de conversion pour l'argument 1: " + String(args[0].c_str()));
        return;
      }
      if (!convertArgument<Arg2>(args[1], arg2)) {
        Serial.println("Erreur de conversion pour l'argument 2: " + String(args[1].c_str()));
        return;
      }
      if (!convertArgument<Arg3>(args[2], arg3)) {
        Serial.println("Erreur de conversion pour l'argument 3: " + String(args[2].c_str()));
        return;
      }
      String response = callback(arg1, arg2, arg3);
      Serial.println(response);
    };
  }

  // Gérer les commandes série
  void handleSerial() {
    static String command = "";

    if (Serial.available() > 0) {
      String incomingChar = Serial.readString();  // Lire toute la chaîne d'entrée

      // If backspace is pressed remove the last character
      if (incomingChar == "\b") {
        if (command.length() > 0) {
          command.remove(command.length() - 1, 1);
        }
      } else {
        command += incomingChar;  // Ajouter le caractère à la commande
      }

      // Si la commande se termine par un retour à la ligne, on la traite
      if (command.endsWith("\n")) {
        processCommand(command);  // Traiter la commande complète
        command = "";             // Réinitialiser pour la prochaine commande
      }
    }
  }

 private:
  std::map<std::string, std::function<void(const std::vector<std::string>&)>> _commandCallbacks;

  // Traitement de la commande
  void processCommand(const String& commandStr) {
    std::istringstream iss(commandStr.c_str());
    std::vector<std::string> tokens;
    std::string token;

    while (iss >> token) {  // Découpe en mots en ignorant les espaces multiples
      tokens.push_back(token);
    }

    if (tokens.empty())
      return;  // Ignore les lignes vides

    std::string command = tokens[0];
    std::vector<std::string> args(tokens.begin() + 1, tokens.end());  // Récupère tous les arguments (éventuellement vide)

    auto it = _commandCallbacks.find(command);
    if (it != _commandCallbacks.end()) {
      it->second(args);  // Envoie une liste vide si pas d'arguments
    } else {
      Serial.println("Commande inconnue.");
    }
  }

  // Conversion des arguments en fonction de leur type
  template <typename T>
  bool convertArgument(const std::string& arg, T& outValue) {
    std::istringstream iss(arg);

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
};
