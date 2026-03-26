#pragma once

#include <glm/glm.hpp>
#include <filesystem>
#include <string>
#include <vector>

// Parsed contents of a scenario TOML file.
struct BodyConfig {
    std::string naif;         // NAIF body name, e.g. "EARTH"
    glm::vec3   color;        // RGB [0,1] tint multiplied on top of all textures
    std::string meshPath;     // optional OBJ shape model; empty = procedural sphere
    std::string diffusePath;
    std::string specularPath;
    std::string normalPath;
    std::string cloudsPath;
    std::string heightmapPath;
};

struct ScenarioConfig {
    std::vector<std::string> kernels;       // absolute or relative kernel paths
    std::string              observerBody;  // floating origin tracks this body
    std::string              observerTarget;
    std::string              frame;
    std::string              epoch;         // UTC string for EphemerisTime::fromString
    std::vector<BodyConfig>  bodies;

    // Parse a TOML scenario file.  Kernel paths that are relative are resolved
    // relative to the directory containing the TOML file.
    static ScenarioConfig load(const std::filesystem::path& tomlPath);
};
