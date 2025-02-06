#ifndef DEBUG_JSON_BUILDER_H
#define DEBUG_JSON_BUILDER_H
//TODO ajouter option pour pas hasher certains champs (ne forcent pas le changement)

#include <Arduino.h>
#include <CRC32.h>
#include <functional>
#include <variant>
#include <vector>

// Définir le type variant qui peut être un int, float, String, etc.
using DebugFieldValue = std::variant<int, float, String>;

typedef std::vector<std::pair<String, std::function<DebugFieldValue()>>> DebugFields;

class DebugBuilder {
 public:
  DebugBuilder(const DebugFields& fields)
      : fields(fields), previousHash(0) {}

  // Méthode pour construire le JSON
  String buildJson() {
    String json;
    json.reserve(256);  // Préalloue de la mémoire pour éviter les allocations dynamiques
    for (auto it = fields.begin(); it != fields.end(); ++it) {
      if (it != fields.begin()) json += ",";
      json += "\"" + it->first + "\":\"" + escapeString(toString(it->second())) + "\"";
    }
    json += "}";
    return json;
  }

  // Méthode pour calculer un CRC32 basé sur les valeurs des champs
  uint32_t calculateHash() {
    CRC32 crc;
    for (const auto& [key, func] : fields) {
      DebugFieldValue val = func();
      std::visit([&](auto&& arg) {
        // Calcul du CRC pour chaque type
        crc.update(reinterpret_cast<const uint8_t*>(&arg), sizeof(arg));
      }, val);
    }
    return crc.finalize();
  }

  // Vérifie si les valeurs ont changé par rapport au dernier hash
  bool hasChanged() {
    uint32_t currentHash = calculateHash();
    if (currentHash != previousHash) {
      previousHash = currentHash;
      return true;
    }
    return false;
  }

 private:
  DebugFields fields;
  uint32_t previousHash;

  // Fonction pour convertir un DebugFieldValue en String
  String toString(const DebugFieldValue& val) {
    return std::visit([](auto&& arg) -> String {
      return String(arg);  // Convertir tout type en String
    }, val);
  }

  // Fonction pour échapper les guillemets dans les valeurs JSON
  String escapeString(const String& input) {
    String output = input;
    output.replace("\"", "\\\"");
    return output;
  }
};


#endif
