#pragma once

#include <glm/glm.hpp>
#include <filesystem>
#include <string>
#include <vector>

// Orbital elements for one spacecraft instance, as parsed from the scenario.
// Units match the TOML: degrees for angles, rev/day for mean motion.
struct OrbitSpec {
    bool        present       = false;
    std::string centralBody   = "EARTH";
    std::string epoch;
    double      inclination   = 0.0;   // deg
    double      raan          = 0.0;   // deg
    double      eccentricity  = 0.0;
    double      argPerigee    = 0.0;   // deg
    double      meanAnomaly   = 0.0;   // deg
    double      meanMotion    = 0.0;   // rev/day
};

// Reference to one spacecraft listed in the scenario.
struct SpacecraftRef {
    std::filesystem::path configPath;  // absolute path to spacecraft TOML
    OrbitSpec             orbit;       // initial state; present = true when supplied
    bool                  player = false;
};

// Physical parameters for one planetary atmosphere.
// All lengths in km; scattering coefficients in km^-1.
struct AtmosphereConfig {
    float     atmosphereRadius = 0.0f;   // km  (planet surface + thickness)
    glm::vec3 rayleigh;                  // β_R km^-1, RGB wavelength channels
    float     rayleighScaleH  = 8.0f;   // H_R km
    float     mieScattering   = 0.0f;   // β_M km^-1
    float     mieExtinction   = 0.0f;   // β_Mext km^-1
    float     mieScaleH       = 1.2f;   // H_M km
    float     mieG            = 0.76f;  // Henyey-Greenstein asymmetry
};

// Parsed contents of a scenario TOML file.
struct BodyConfig {
    std::string naif;         // NAIF body name used for position, e.g. "JUPITER BARYCENTER"
    std::string radiiNaif;    // NAIF name used for PCK radius lookup; defaults to naif if empty
    glm::vec3   color;        // RGB [0,1] tint multiplied on top of all textures
    std::string meshPath;          // optional OBJ shape model; empty = procedural sphere
    std::string ringTexturePath;   // empty = no ring
    float       ringInnerRadius = 0.0f;  // in body radii
    float       ringOuterRadius = 0.0f;
    std::string diffusePath;
    std::string specularPath;
    std::string normalPath;
    std::string cloudsPath;
    std::string heightmapPath;
    bool              hasAtmosphere = false;
    AtmosphereConfig  atmosphere;
};

struct ScenarioConfig {
    std::vector<std::string>   kernels;        // absolute kernel paths
    std::string                observerBody;   // floating origin tracks this body
    std::string                observerTarget;
    std::string                frame;
    std::string                epoch;          // UTC string for EphemerisTime::fromString
    std::vector<BodyConfig>    bodies;
    std::vector<SpacecraftRef> spacecraft;     // in load order; player marked with .player

    // Parse a TOML scenario file.  Kernel paths that are relative are resolved
    // relative to the directory containing the TOML file.
    static ScenarioConfig load(const std::filesystem::path& tomlPath);
};
