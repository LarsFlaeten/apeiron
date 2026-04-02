#include "apeiron/spacecraft/ManifestLoader.h"

#include <toml++/toml.hpp>
#include <stdexcept>
#include <string>

namespace spacecraft {

static glm::vec3 tomlToVec3(const toml::array& arr, const char* field)
{
    if (arr.size() != 3)
        throw std::runtime_error(std::string(field) + " must be an array of 3 floats");
    return {
        static_cast<float>(arr[0].value_or(0.0)),
        static_cast<float>(arr[1].value_or(0.0)),
        static_cast<float>(arr[2].value_or(0.0))
    };
}

static ThrusterType parseType(std::string_view s)
{
    if (s == "main_engine") return ThrusterType::MainEngine;
    if (s == "auxiliary")   return ThrusterType::Auxiliary;
    if (s == "rcs")         return ThrusterType::RCS;
    throw std::runtime_error("Unknown thruster type: " + std::string(s));
}

SpacecraftModel loadManifest(const std::filesystem::path& tomlPath)
{
    const auto doc = toml::parse_file(tomlPath.string());

    // Navigate to spacecraft.orion (first spacecraft key found).
    const auto& sc_root = *doc["spacecraft"].as_table();
    if (sc_root.empty())
        throw std::runtime_error("No spacecraft entries found in manifest");
    const auto& sc = *sc_root.begin()->second.as_table();

    SpacecraftModel model;
    model.massKg = static_cast<float>(sc["mass_kg"].value_or(1.0));

    if (auto* com = sc["center_of_mass"].as_array())
        model.centerOfMass = tomlToVec3(*com, "center_of_mass");

    if (auto* it = sc["inertia_tensor"].as_array())
        model.inertiaDiag = tomlToVec3(*it, "inertia_tensor");

    const auto* thr_arr = sc["thrusters"].as_array();
    if (!thr_arr)
        throw std::runtime_error("No [thrusters] array in manifest");

    for (const auto& entry : *thr_arr) {
        const auto& t = *entry.as_table();
        Thruster thr;
        thr.id        = t["id"].value_or(std::string{});
        thr.type      = parseType(t["type"].value_or(std::string_view{"rcs"}));
        thr.quad      = static_cast<int>(t["quad"].value_or(0LL));
        thr.thrustN   = static_cast<float>(t["thrust_n"].value_or(0.0));
        thr.ispS      = static_cast<float>(t["isp_s"].value_or(0.0));
        thr.refNode      = t["ref_node"].value_or(std::string{});
        thr.exhaustNode  = t["exhaust_node"].value_or(std::string{});
        thr.exhaustScale    = static_cast<float>(t["exhaust_scale"].value_or(1.0));
        thr.plumeIntensity  = static_cast<float>(t["plume_intensity"].value_or(1.0));
        if (auto* col = t["plume_color"].as_array())
            thr.plumeColor = tomlToVec3(*col, "plume_color");

        if (auto* pos = t["position"].as_array())
            thr.position = tomlToVec3(*pos, "position");
        if (auto* dir = t["direction"].as_array())
            thr.direction = glm::normalize(tomlToVec3(*dir, "direction"));

        model.thrusters.push_back(std::move(thr));
    }

    model.buildEffectivenessMatrix();
    return model;
}

} // namespace spacecraft
