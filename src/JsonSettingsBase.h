#include <ArduinoJson.h>
#include <LittleFS.h>

/**
 * Base class for settings that can saved and loaded from the file system into a JSON file.
 * Must be inherited by a class that will define the properties to save and load.
 */
class JsonSettingsBase {
 protected:
  String filename;

 public:
  /**
   * Base constructor, must be called from the derived class for the settings file to be set.
   * @param file The file name to use for the settings.
   */
  JsonSettingsBase(const String& file)
      : filename(file) {}

      /**
       * Derived class must implement this method to serialize its properties into the JSON document.
       * @param doc The JSON document to serialize into. //TODO vérifier que c'est correct les commentaires (pour les deux méthodes)
       */
  virtual void serialize(JsonDocument& doc) = 0;

    /**
     * Derived class must implement this method to deserialize its properties from the JSON document.
     * @param doc The JSON document to deserialize from.
     */
  virtual void deserialize(JsonDocument& doc) = 0;

  /**
   * Load the settings from the file system.
   */
  bool load() {
    File file = LittleFS.open(filename, "r");
    if (!file) {
      return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
      return false;
    }

    deserialize(doc);
    return true;
  }

  /**
   * Save the settings to the file system.
   */
  bool save() {
    JsonDocument doc;
    serialize(doc);

    File file = LittleFS.open(filename, "w");
    if (!file) {
      return false;
    }

    serializeJson(doc, file);
    file.close();
    return true;
  }

  /**
   * Returns JSON (readable) content of the settings.
   */
  String getContent() {
    JsonDocument doc;
    serialize(doc);  // Sérialise les propriétés actuelles dans le doc
    String jsonString;
    serializeJson(doc, jsonString);  // Convertit le doc en string
    return jsonString;
  }
};
