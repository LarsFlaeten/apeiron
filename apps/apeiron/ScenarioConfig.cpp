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
                cfg.kernels.push_back(
                    kp.is_absolute() ? kp.string() : (baseDir / kp).string());
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
            bc.naif = (*t)["naif"].value_or(std::string(""));
            if (auto* col = (*t)["color"].as_array(); col && col->size() == 3) {
                bc.color.r = col->get(0)->value_or(1.0f);
                bc.color.g = col->get(1)->value_or(1.0f);
                bc.color.b = col->get(2)->value_or(1.0f);
            } else {
                bc.color = {1.0f, 1.0f, 1.0f};
            }
            bc.renderScale = (*t)["render_scale"].value_or(1.0f);
            cfg.bodies.push_back(bc);
        }
    }

    return cfg;
}
