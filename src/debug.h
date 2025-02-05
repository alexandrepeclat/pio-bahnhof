#ifndef DEBUG_JSON_BUILDER_H
#define DEBUG_JSON_BUILDER_H

#include <Arduino.h>
#include <CRC32.h>
#include <functional>
#include <map>

class DebugJsonBuilder {
 public:
  DebugJsonBuilder(const std::map<String, std::function<String()>>& fields)
      : fields(fields), previousHash(0) {}

  // Méthode pour construire le JSON
  String buildJson() {
    String json;
    json.reserve(256);  // Préalloue de la mémoire pour éviter les allocations dynamiques
    json = "HASH" + String(previousHash) + "{";
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
      crc.update(reinterpret_cast<const uint8_t*>(val.c_str()), val.length());  // TODO vérifier que c'est bien le hash du résultat de l'appel et non le pointeur qui est hashé et aussi qu'il accepte bien le c_str() sinon faire un     crc.update(reinterpret_cast<const uint8_t*>(concatenatedValues.c_str()), concatenatedValues.length());  // Mise à jour du CRC avec les données
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
  std::map<String, std::function<String()>> fields;
  uint32_t previousHash;

  // Fonction pour échapper les guillemets dans les valeurs JSON
  String escapeString(const String& input) {
    String output = input;
    output.replace("\"", "\\\"");
    return output;
  }
};

#endif
