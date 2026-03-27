#include "ScenarioConfig.h"

#include <toml++/toml.hpp>
#include <stdexcept>

ScenarioConfig ScenarioConfig::load(const std::filesystem::path& tomlPath)
{
    auto tbl     = toml::parse_file(tomlPath.string());
    auto baseDir = tomlPath.parent_path();

    ScenarioConfig cfg;

    // [[kernels]] — array of tables, each with a "path" key
    if (auto* arr = tbl["kernels"].as_array()) {
        for (auto& elem : *arr) {
            if (auto* t = elem.as_table()) {
                auto p = (*t)["path"].value<std::string>();
                if (!p) throw std::runtime_error("kernel entry missing 'path'");
                std::filesystem::path kp(*p);
                auto resolved = kp.is_absolute() ? kp : std::filesystem::weakly_canonical(baseDir / kp);
                cfg.kernels.push_back(resolved.string());
            }
        }
    }

    // [observer]
    cfg.observerBody   = tbl["observer"]["body"]  .value_or(std::string("EARTH"));
    cfg.observerTarget = tbl["observer"]["target"].value_or(std::string("SUN"));
    cfg.frame          = tbl["observer"]["frame"] .value_or(std::string("J2000"));

    // [time]
    cfg.epoch = tbl["time"]["epoch"].value_or(std::string("2025-01-01T12:00:00"));

    // [[bodies]]
    if (auto* arr = tbl["bodies"].as_array()) {
        for (auto& elem : *arr) {
            auto* t = elem.as_table();
            if (!t) continue;
            BodyConfig bc;
            bc.naif      = (*t)["naif"].value_or(std::string(""));
            bc.radiiNaif = (*t)["radii_naif"].value_or(std::string(""));
            if (auto* col = (*t)["color"].as_array(); col && col->size() == 3) {
                bc.color.r = col->get(0)->value_or(1.0f);
                bc.color.g = col->get(1)->value_or(1.0f);
                bc.color.b = col->get(2)->value_or(1.0f);
            } else {
                bc.color = {1.0f, 1.0f, 1.0f};
            }
            auto resolvePath = [&](std::string_view key) -> std::string {
                auto val = (*t)[key].value<std::string>();
                if (!val) return {};
                std::filesystem::path p(*val);
                return (p.is_absolute() ? p : std::filesystem::weakly_canonical(baseDir / p)).string();
            };
            bc.meshPath         = resolvePath("mesh");
            bc.ringTexturePath  = resolvePath("ring_texture");
            bc.ringInnerRadius  = (*t)["ring_inner_radius"].value_or(0.0f);
            bc.ringOuterRadius  = (*t)["ring_outer_radius"].value_or(0.0f);
            bc.diffusePath   = resolvePath("diffuse");
            bc.specularPath  = resolvePath("specular");
            bc.normalPath    = resolvePath("normals");
            bc.cloudsPath    = resolvePath("clouds");
            bc.heightmapPath = resolvePath("displacement");

            // Optional atmosphere sub-table.
            if (auto* atm = (*t)["atmosphere"].as_table()) {
                bc.hasAtmosphere = true;
                bc.atmosphere.atmosphereRadius = (*atm)["radius_km"].value_or(0.0f);
                if (auto* rgb = (*atm)["rayleigh_rgb"].as_array(); rgb && rgb->size() == 3) {
                    bc.atmosphere.rayleigh.r = rgb->get(0)->value_or(0.0f);
                    bc.atmosphere.rayleigh.g = rgb->get(1)->value_or(0.0f);
                    bc.atmosphere.rayleigh.b = rgb->get(2)->value_or(0.0f);
                }
                bc.atmosphere.rayleighScaleH = (*atm)["rayleigh_scale_h"].value_or(8.0f);
                bc.atmosphere.mieScattering  = (*atm)["mie_scattering"].value_or(0.0f);
                bc.atmosphere.mieExtinction  = (*atm)["mie_extinction"].value_or(0.0f);
                bc.atmosphere.mieScaleH      = (*atm)["mie_scale_h"].value_or(1.2f);
                bc.atmosphere.mieG           = (*atm)["mie_g"].value_or(0.76f);
            }

            cfg.bodies.push_back(bc);
        }
    }

    return cfg;
}
