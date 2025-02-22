#include <ArduinoJson.h>
#include <LittleFS.h>

class JsonSettingsBase {
protected:
    String filename;

public:
    JsonSettingsBase(const String& file) : filename(file) {}

    virtual void serialize(JsonDocument& doc) = 0;
    virtual void deserialize(JsonDocument& doc) = 0;

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

    String getContent() {
        JsonDocument doc;
        serialize(doc); // Sérialise les propriétés actuelles dans le doc
        String jsonString;
        serializeJson(doc, jsonString); // Convertit le doc en string
        return jsonString;
    }
};
