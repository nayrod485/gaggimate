#include "ProfileManager.h"
#include <ArduinoJson.h>

ProfileManager::ProfileManager(fs::FS &fs, const char *dir, Settings &settings) : _fs(fs), _dir(dir), _settings(settings) {}

bool ProfileManager::begin() { return ensureDirectory(); }

bool ProfileManager::ensureDirectory() {
    if (!_fs.exists(_dir)) {
        return _fs.mkdir(_dir);
    }
    return true;
}

String ProfileManager::profilePath(const String &uuid) { return _dir + "/" + uuid + ".json"; }

std::vector<String> ProfileManager::listProfiles() {
    std::vector<String> uuids;
    File root = _fs.open(_dir);
    if (!root || !root.isDirectory())
        return uuids;

    File file = root.openNextFile();
    while (file) {
        String name = file.name();
        if (name.endsWith(".json")) {
            int start = name.lastIndexOf('/') + 1;
            int end = name.lastIndexOf('.');
            uuids.push_back(name.substring(start, end));
        }
        file = root.openNextFile();
    }
    return uuids;
}

bool ProfileManager::loadProfile(const String &uuid, Profile &outProfile) {
    File file = _fs.open(profilePath(uuid), "r");
    if (!file)
        return false;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err)
        return false;

    return parseProfile(doc.as<JsonObject>(), outProfile);
}

bool ProfileManager::saveProfile(const Profile &profile) {
    if (!ensureDirectory())
        return false;

    File file = _fs.open(profilePath(profile.id), "w");
    if (!file)
        return false;

    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    writeProfile(obj, profile);

    bool ok = serializeJson(doc, file) > 0;
    file.close();
    return ok;
}

bool ProfileManager::deleteProfile(const String &uuid) { return _fs.remove(profilePath(uuid)); }

bool ProfileManager::profileExists(const String &uuid) { return _fs.exists(profilePath(uuid)); }

void ProfileManager::selectProfile(const String &uuid) const { _settings.setSelectedProfile(uuid); }

String ProfileManager::getSelectedProfile() const { return _settings.getSelectedProfile(); }

void ProfileManager::loadSelectedProfile(Profile &outProfile) { loadProfile(_settings.getSelectedProfile(), outProfile); }
