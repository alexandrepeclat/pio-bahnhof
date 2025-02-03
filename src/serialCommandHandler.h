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
  void registerCommand(const String& command, std::function<String()> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<String>& args) {
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
  void registerCommand(const String& command, std::function<String(Arg1)> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<String>& args) {
      if (args.size() != 1) {
        Serial.println("Erreur: mauvais nombre d'arguments.");
        return;
      }
      Arg1 arg1;
      if (!convertArgument<Arg1>(args[0], arg1)) {
        Serial.println("Erreur de conversion pour l'argument 1: " + args[0]);
        return;
      }
      String response = callback(arg1);
      Serial.println(response);
    };
  }

  // Enregistrer une commande avec 2 arguments
  template <typename Arg1, typename Arg2>
  void registerCommand(const String& command, std::function<String(Arg1, Arg2)> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<String>& args) {
      if (args.size() != 2) {
        Serial.println("Erreur: mauvais nombre d'arguments.");
        return;
      }
      Arg1 arg1;
      Arg2 arg2;
      if (!convertArgument<Arg1>(args[0], arg1)) {
        Serial.println("Erreur de conversion pour l'argument 1: " + args[0]);
        return;
      }
      if (!convertArgument<Arg2>(args[1], arg2)) {
        Serial.println("Erreur de conversion pour l'argument 2: " + args[1]);
        return;
      }
      String response = callback(arg1, arg2);
      Serial.println(response);
    };
  }

  // Enregistrer une commande avec 3 arguments
  template <typename Arg1, typename Arg2, typename Arg3>
  void registerCommand(const String& command, std::function<String(Arg1, Arg2, Arg3)> callback) {
    _commandCallbacks[command] = [this, callback](const std::vector<String>& args) {
      if (args.size() != 3) {
        Serial.println("Erreur: mauvais nombre d'arguments.");
        return;
      }
      Arg1 arg1;
      Arg2 arg2;
      Arg3 arg3;
      if (!convertArgument<Arg1>(args[0], arg1)) {
        Serial.println("Erreur de conversion pour l'argument 1: " + args[0]);
        return;
      }
      if (!convertArgument<Arg2>(args[1], arg2)) {
        Serial.println("Erreur de conversion pour l'argument 2: " + args[1]);
        return;
      }
      if (!convertArgument<Arg3>(args[2], arg3)) {
        Serial.println("Erreur de conversion pour l'argument 3: " + args[2]);
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
  std::map<String, std::function<void(const std::vector<String>&)>> _commandCallbacks;

  // Traitement de la commande
  void processCommand(const String& commandStr) {
    std::vector<String> tokens;
    String command;
    int start = 0;

    // Découpe la commande en tokens en utilisant l'espace comme séparateur
    while (true) {
      int spaceIndex = commandStr.indexOf(' ', start);
      if (spaceIndex == -1) {
        tokens.push_back(commandStr.substring(start));  // Dernier token
        break;
      }
      tokens.push_back(commandStr.substring(start, spaceIndex));
      start = spaceIndex + 1;
    }

    if (tokens.empty())
      return;  // Ignore les lignes vides

    command = tokens[0];
    std::vector<String> args(tokens.begin() + 1, tokens.end());  // Récupère tous les arguments

    // Recherche de la commande dans la map
    auto it = _commandCallbacks.find(command);
    if (it != _commandCallbacks.end()) {
      it->second(args);  // Envoie les arguments
    } else {
      Serial.println("Commande inconnue.");
    }
  }

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
};
