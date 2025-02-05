#ifndef DEBUG_JSON_BUILDER_H
#define DEBUG_JSON_BUILDER_H

#include <Arduino.h>
#include <CRC32.h>
#include <functional>
#include <vector>

typedef std::vector<std::pair<String, std::function<String()>>> DebugFields;

class DebugBuilder { //TODO Comparer les perfs avec système d'avant : commit be1997fe6e899b7f2f3032f2f2788ff617f0a662
 public:
  DebugBuilder(const DebugFields& fields)
      : fields(fields), previousHash(0) {}

  // Méthode pour construire le JSON
  String buildJson() {
    String json;
    json.reserve(256);  // Préalloue de la mémoire pour éviter les allocations dynamiques
    for (auto it = fields.begin(); it != fields.end(); ++it) {
      if (it != fields.begin())
        json += ",";
      json += "\"" + it->first + "\":\"" + escapeString(it->second()) + "\"";
    }
    json += "}";
    return json;
  }

  // Méthode pour calculer un CRC32 basé sur les valeurs des champs
  uint32_t calculateHash() {
    CRC32 crc;
    for (const auto& [key, func] : fields) {
      String val = func();
      crc.update(reinterpret_cast<const uint8_t*>(val.c_str()), val.length());  // Mise à jour du CRC avec les données (cast nécessaire sinon va faire le crc de la référence passée)
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

  // Fonction pour échapper les guillemets dans les valeurs JSON
  String escapeString(const String& input) {
    String output = input;
    output.replace("\"", "\\\"");
    return output;
  }
};

#endif
