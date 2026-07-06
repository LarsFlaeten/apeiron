#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include "apeiron/render/Camera.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Geometry.h"
#include "apeiron/render/GpuAllocator.h"
#include "apeiron/render/Mesh.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/TonemapPipeline.h"
#include "apeiron/render/Renderer.h"
#include "apeiron/render/BloomPass.h"
#include "apeiron/render/AtmospherePrecompute.h"
#include "apeiron/render/AtmospherePipeline.h"
#include "apeiron/render/StarField.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Texture.h"
#include "apeiron/render/Vertex.h"
#include "apeiron/render/MeshPipeline.h"
#include "apeiron/render/GltfModel.h"

#include "apeiron/universe/BodyProperties.h"
#include "apeiron/universe/CelestialBody.h"
#include "apeiron/universe/KernelPool.h"
#include "apeiron/universe/Observer.h"
#include "apeiron/universe/Scene.h"

#include "astro/Time.h"
#include "astro/SpiceCore.h"
#include "ScenarioConfig.h"
#include "apeiron/spacecraft/PhysicsWorld.h"
#include "apeiron/spacecraft/Spacecraft.h"
#include "MFD.h"
#include "NavMode.h"
#include "NavConsole.h"
#include "NavHUD.h"
#include "NavState.h"
#include "OrbitalMFD.h"
#include "DockingMFD.h"
#include "DockingConstraint.h"
#include "OffscreenCam.h"
#include "CamMFD.h"
#include "TransferMFD.h"
#include "CislunarMFD.h"
#include "MapMFD.h"
#include "SimSave.h"
#include "MFDMenu.h"
#include "VoiceAnnouncer.h"
#include "OBCEventQueue.h"
#include "MFDContext.h"
#include "OBCMFD.h"
#include "apeiron/spacecraft/Autopilot.h"
#include "apeiron/spacecraft/ManifestLoader.h"
#include "apeiron/spacecraft/OrbitLoader.h"
#include "apeiron/spacecraft/SpacecraftModel.h"
#include "apeiron/spacecraft/VehicleConfig.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cspice/SpiceUsr.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>

#ifndef APEIRON_SHADER_DIR
#  error "APEIRON_SHADER_DIR not defined"
#endif
#ifndef APEIRON_SCENARIO_FILE
#  error "APEIRON_SCENARIO_FILE not defined"
#endif
#ifndef APEIRON_STAR_CATALOG
#  error "APEIRON_STAR_CATALOG not defined"
#endif

// ---------------------------------------------------------------------------
// Star catalog loader
// ---------------------------------------------------------------------------

namespace {

// Map B-V color index to linear RGB.
glm::vec3 bvToColor(float bv)
{
    bv = std::clamp(bv, -0.4f, 2.0f);
    // Control points: (bv, linear RGB)
    constexpr struct { float bv; glm::vec3 rgb; } table[] = {
        { -0.4f, { 0.60f, 0.70f, 1.00f } },  // O/B: blue-white
        {  0.0f, { 0.90f, 0.95f, 1.00f } },  // A:   white-blue
        {  0.6f, { 1.00f, 1.00f, 0.85f } },  // G:   yellow-white (Sun ~0.65)
        {  1.2f, { 1.00f, 0.80f, 0.50f } },  // K:   orange
        {  2.0f, { 1.00f, 0.50f, 0.20f } },  // M:   red
    };
    for (int i = 0; i < 4; ++i) {
        if (bv <= table[i + 1].bv) {
            float t = (bv - table[i].bv) / (table[i + 1].bv - table[i].bv);
            return glm::mix(table[i].rgb, table[i + 1].rgb, t);
        }
    }
    return table[4].rgb;
}

std::vector<apeiron::render::StarVertex> loadStars(const std::string& csvPath,
                                                    float              magLimit)
{
    // Get the rotation from J2000 equatorial to ECLIPJ2000 via SPICE.
    // Both frames are inertial so the matrix is constant; et=0 is fine.
    SpiceDouble m[3][3];
    pxform_c("J2000", "ECLIPJ2000", 0.0, m);
    glm::mat3 toEcliptic;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            toEcliptic[c][r] = static_cast<float>(m[r][c]);  // row→col

    std::ifstream file(csvPath);
    if (!file) throw std::runtime_error("Cannot open star catalog: " + csvPath);

    // Parse header to find column indices.
    std::string line;
    std::getline(file, line);
    std::vector<std::string> headers;
    {
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            // Strip surrounding quotes.
            if (tok.size() >= 2 && tok.front() == '"' && tok.back() == '"')
                tok = tok.substr(1, tok.size() - 2);
            headers.push_back(tok);
        }
    }
    auto colOf = [&](const std::string& name) -> int {
        for (int i = 0; i < (int)headers.size(); ++i)
            if (headers[i] == name) return i;
        throw std::runtime_error("Star catalog missing column: " + name);
    };
    int colX   = colOf("x");
    int colY   = colOf("y");
    int colZ   = colOf("z");
    int colMag = colOf("mag");
    int colCI  = colOf("ci");

    std::vector<apeiron::render::StarVertex> stars;
    stars.reserve(40'000);

    while (std::getline(file, line)) {
        std::vector<std::string_view> fields;
        fields.reserve(40);
        std::string_view sv(line);
        std::string_view::size_type start = 0;
        while (true) {
            auto comma = sv.find(',', start);
            fields.push_back(sv.substr(start, comma - start));
            if (comma == std::string_view::npos) break;
            start = comma + 1;
        }

        auto getFloat = [&](int col, float fallback = 0.0f) -> float {
            if (col >= (int)fields.size() || fields[col].empty()) return fallback;
            try { return std::stof(std::string(fields[col])); }
            catch (...) { return fallback; }
        };

        float mag = getFloat(colMag, 99.0f);
        if (mag > magLimit) continue;

        float x = getFloat(colX);
        float y = getFloat(colY);
        float z = getFloat(colZ);
        if (x == 0.0f && y == 0.0f && z == 0.0f) continue;  // Sol row

        // HDR brightness: magnitude 1 → 1.0; capped at 2.0 so even Sirius
        // doesn't produce a bloom box artifact vs the sun (which is ~50).
        float brightness = std::pow(10.0f, (1.0f - mag) / 2.5f);
        brightness = std::min(brightness, 2.0f);

        float bv    = getFloat(colCI, 0.6f);  // default to solar G2 if missing
        glm::vec3 color = bvToColor(bv) * brightness;

        // Rotate equatorial J2000 → ECLIPJ2000 and normalise.
        glm::vec3 dir = glm::normalize(toEcliptic * glm::vec3(x, y, z));

        stars.push_back({ dir, color });
    }

    std::cout << "Loaded " << stars.size() << " stars (mag < " << magLimit << ")\n" << std::flush;
    return stars;
}

} // namespace

int main(int argc, char* argv[])
{
    // Parse --scenario <path> (or -s <path>) before anything else.
    std::filesystem::path scenarioPath = APEIRON_SCENARIO_FILE;
    for (int i = 1; i + 1 < argc; ++i) {
        std::string_view a(argv[i]);
        if (a == "--scenario" || a == "-s") {
            scenarioPath = argv[i + 1];
            std::cout << "[Apeiron] Scenario override: " << scenarioPath << "\n";
            break;
        }
    }

    if (!glfwInit()) { std::cerr << "GLFW init failed\n"; return 1; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE,  GLFW_FALSE);

    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;
    GLFWwindow* window = glfwCreateWindow(kWidth, kHeight, "Apeiron", nullptr, nullptr);
    if (!window) {
        std::cerr << "Window creation failed\n";
        glfwTerminate();
        return 1;
    }

    obc::init();

    try {
        // =================================================================
        // Universe — no GPU resources involved
        // =================================================================
        auto cfg = ScenarioConfig::load(scenarioPath);

        glfwSetWindowTitle(window, "Apeiron — Loading SPICE kernels…");
        glfwPollEvents();
        auto& pool = apeiron::universe::KernelPool::instance();
        for (auto& k : cfg.kernels) {
            pool.load(k);
            glfwPollEvents();
        }

        auto et = astro::EphemerisTime::fromString(cfg.epoch);

        apeiron::universe::Observer observer(
            cfg.observerBody, cfg.observerTarget, cfg.frame);

        apeiron::universe::Scene scene;

        // Plain body info — no GPU resources, safe to declare here.
        struct BodyInfo {
            apeiron::universe::CelestialBody* node;
            float     radiusKm;       // equatorial a-axis
            float     bRadiusKm;      // equatorial b-axis (equals radiusKm for oblate spheroids)
            float     polarRadiusKm;  // polar c-axis
            glm::vec3 color;
            std::string meshPath;          // empty = procedural sphere
            std::string diffusePath;
            std::string specularPath;
            std::string normalPath;
            std::string cloudsPath;
            std::string heightmapPath;
            float       displaceScale = 0.0f;
            std::string ringTexturePath;   // empty = no ring
            float       ringInnerRadius = 0.0f;  // in body radii
            float       ringOuterRadius = 0.0f;
            bool              hasAtmosphere = false;
            AtmosphereConfig  atmosphere;
        };
        std::vector<BodyInfo> bodyInfos;

        // Index of the Sun body in bodyInfos (for computing light direction).
        int sunIndex = -1;

        for (int i = 0; i < static_cast<int>(cfg.bodies.size()); ++i) {
            auto& bc    = cfg.bodies[i];
            auto  props = apeiron::universe::BodyProperties::queryFromSpice(
                              bc.radiiNaif.empty() ? bc.naif : bc.radiiNaif);
            auto& node  = scene.addBody(bc.naif, bc.naif, props.radiusKm,
                                        "SOLAR SYSTEM BARYCENTER", cfg.frame);
            bodyInfos.push_back({ &node,
                                   static_cast<float>(props.radiusKm),
                                   static_cast<float>(props.bRadiusKm),
                                   static_cast<float>(props.polarRadiusKm),
                                   bc.color, bc.meshPath,
                                   bc.diffusePath, bc.specularPath,
                                   bc.normalPath, bc.cloudsPath, bc.heightmapPath,
                                   0.0f,
                                   bc.ringTexturePath, bc.ringInnerRadius, bc.ringOuterRadius,
                                   bc.hasAtmosphere, bc.atmosphere });
            if (bc.naif == "SUN") sunIndex = i;
        }

        auto observerPos = observer.worldPosition(et);
        scene.update(et, observerPos);

        // Observer body radius and GM — used for fallback orbit and camera setup.
        float  observerBodyRadius = 6371.0f;
        double observerBodyGm     = 398600.4418;
        for (auto& bi : bodyInfos) {
            if (bi.node->naifName() == cfg.observerBody) {
                observerBodyRadius = bi.radiusKm;
                break;
            }
        }

        // =================================================================
        // Spacecraft — physics only, no GPU resources
        // Coordinate system: Earth-centred ECLIPJ2000 (km, km/s).
        // =================================================================

        // Earth's GM (km³/s²) — kept for any Earth-specific calculations.
        constexpr double kGM_Earth = 398600.4418;

        // Build gravity body specs from scenario bodies that have a GM in SPICE.
        std::vector<spacecraft::PhysicsWorld::BodySpec> gravBodySpecs;
        {
            for (auto& bi : bodyInfos) {
                const std::string& name = bi.node->naifName();
                int id;
                try { id = astro::Spice().bodyNameToId(name); }
                catch (const astro::SpiceException&) { continue; }
                double gm = 0.0;
                try {
                    astro::Spice().getPlanetaryConstants(id, "GM", gm);
                } catch (const astro::SpiceException&) { continue; }
                if (gm <= 0.0) continue;
                gravBodySpecs.push_back({ name, id, gm,
                                          static_cast<double>(bi.radiusKm) });
            }
        }
        spacecraft::SpiceEphemeris spiceEph;
        spacecraft::PhysicsWorld physicsWorld(std::move(gravBodySpecs),
                                              cfg.observerBody, spiceEph);
        const auto& gravBodies = physicsWorld.gravBodies();
        // Bodies available for transfer planning — planets and barycenters only,
        // excluding the Sun.  Built once from bodyInfos; handed to MFDContext.
        // Filter: barycenters 1-9, or planet centres x99 (199-899).
        std::vector<BodyEntry> transferBodies;
        {
            for (auto& bi : bodyInfos) {
                const std::string& name = bi.node->naifName();
                auto maybeId = astro::Spice().tryBodyNameToId(name);
                if (!maybeId) continue;
                int id = *maybeId;
                // Barycenters 1-9 or planet centres 199,299,...,899.
                const bool isBarycenter   = (id >= 1 && id <= 9);
                const bool isPlanetCentre = (id >= 199 && id <= 899 && id % 100 == 99);
                if (!isBarycenter && !isPlanetCentre) continue;
                // Short name = first word of NAIF name.
                std::string shortName = name.substr(0, name.find(' '));
                transferBodies.push_back({ name, shortName, id });
            }
        }

        // Resolve observer body GM from gravBodies now that it's built.
        for (const auto& gb : gravBodies)
            if (gb.name == cfg.observerBody) { observerBodyGm = gb.gm; break; }

        // Track which body currently dominates gravity for the player spacecraft.
        std::string dominantBodyName = cfg.observerBody;

        // Fallback circular orbit at 400 km above the observer body.
        const double orbitRadius = static_cast<double>(observerBodyRadius) + 400.0;
        const double circularV   = std::sqrt(observerBodyGm / orbitRadius);

        // ---- Load spacecraft configs from scenario ----
        // Convention: player craft is index 0; AI/passive craft follow in order.
        // Non-player craft must have [orbit] so their initial state can be computed.
        std::vector<spacecraft::VehicleConfig> vehicleConfigs;
        size_t playerVehicleIdx = 0;
        for (size_t vi = 0; vi < cfg.spacecraft.size(); ++vi) {
            const auto& ref = cfg.spacecraft[vi];
            if (!std::filesystem::exists(ref.configPath)) {
                std::cerr << "[Apeiron] WARNING: spacecraft config not found: "
                          << ref.configPath << "\n";
                vehicleConfigs.emplace_back();   // empty fallback
                continue;
            }
            auto vc = spacecraft::loadVehicleConfig(ref.configPath);
            if (ref.orbit.present) {
                spacecraft::applyOrbit(vc,
                    ref.orbit.centralBody, ref.orbit.epoch,
                    ref.orbit.inclination, ref.orbit.raan,
                    ref.orbit.eccentricity, ref.orbit.argPerigee,
                    ref.orbit.meanAnomaly, ref.orbit.meanMotion);
            }
            std::cout << "[Apeiron] Loaded spacecraft '" << vc.name << "'"
                      << (ref.player ? " [player]" : "")
                      << ": " << vc.model.thrusters.size() << " thrusters"
                      << (vc.hasOrbit ? ", has orbit" : "") << "\n";
            if (ref.player) playerVehicleIdx = vi;
            vehicleConfigs.push_back(std::move(vc));
        }

        // Convenience alias for the player vehicle.
        auto& playerVehicle = vehicleConfigs[playerVehicleIdx];

        // ---- Non-player craft: propagate all AI orbits to sim epoch ----
        // issIdx = 1: first AI craft occupies physics slot 1 (ISS or equivalent).
        // issR/V_init: first AI craft state, used for player co-elliptic placement.
        const size_t issIdx = 1;
        bool hasAiCraft = false;
        glm::dvec3 issR_init(0.0), issV_init(0.0);
        for (size_t vi = 0; vi < vehicleConfigs.size(); ++vi) {
            if (vi == playerVehicleIdx) continue;
            auto& vc = vehicleConfigs[vi];
            if (!vc.hasOrbit) continue;
            astro::PosState st = spacecraft::vehicleStateAtEt(vc, et);
            std::cout << "[Apeiron] " << vc.name << " initial position: "
                      << st.r.x << ", " << st.r.y << ", " << st.r.z << " km\n";
            if (!hasAiCraft) {
                issR_init = glm::dvec3(st.r);
                issV_init = glm::dvec3(st.v);
                hasAiCraft = true;
            }
        }

        // ---- Player craft initial state ----
        // Priority:
        //   1. Player has [orbit] in its own config → propagate to sim epoch.
        //   2. AI craft present → place 250 m behind it (co-elliptic).
        //   3. Neither → fallback circular 400 km LEO.
        const double shipMass = playerVehicle.massKg > 1.0f
                                ? static_cast<double>(playerVehicle.massKg)
                                : 26500.0;
        const glm::dmat3 shipInertia = glm::length(playerVehicle.inertiaDiag) > 0.0
            ? glm::dmat3(
                glm::dvec3(playerVehicle.inertiaDiag.x, 0, 0),
                glm::dvec3(0, playerVehicle.inertiaDiag.y, 0),
                glm::dvec3(0, 0, playerVehicle.inertiaDiag.z))
            : glm::dmat3(34000.0);
        astro::State shipState;
        if (playerVehicle.hasOrbit) {
            // Use the player's own orbit config.
            astro::PosState ps = spacecraft::vehicleStateAtEt(playerVehicle, et);
            glm::dvec3 pr = glm::dvec3(ps.r), pv = glm::dvec3(ps.v);
            std::cout << "[Apeiron] " << playerVehicle.name << " initial position (own orbit): "
                      << pr.x << ", " << pr.y << ", " << pr.z << " km\n";
            glm::dvec3 T = glm::normalize(pv);
            glm::dvec3 N = glm::normalize(glm::cross(pr, pv));
            glm::dvec3 R = glm::cross(T, N);
            shipState.P.r = pr;
            shipState.P.v = pv;
            shipState.R.q = glm::quat_cast(glm::dmat3(T, -R, N));
            shipState.R.w = glm::dvec3(0.0);
        } else if (hasAiCraft) {
            // Co-elliptic, 250 m behind the first AI craft.
            constexpr double kSepKm = 0.25;
            glm::dvec3 prograde = glm::normalize(issV_init);
            glm::dvec3 T = prograde;
            glm::dvec3 N = glm::normalize(glm::cross(issR_init, issV_init));
            glm::dvec3 R = glm::cross(T, N);
            shipState.P.r = issR_init - kSepKm * prograde;
            shipState.P.v = issV_init;
            shipState.R.q = glm::quat_cast(glm::dmat3(T, -R, N));
            shipState.R.w = glm::dvec3(0.0);
        } else {
            // Fallback: circular orbit 400 km above the observer body.
            const double r0 = static_cast<double>(observerBodyRadius) + 400.0;
            const double v0 = std::sqrt(observerBodyGm / r0);
            shipState.P.r = glm::dvec3(r0, 0.0, 0.0);
            shipState.P.v = glm::dvec3(0.0, v0, 0.0);
            shipState.R.q = glm::dquat(1.0, 0.0, 0.0, 0.0);
            shipState.R.w = glm::dvec3(0.0);
        }

        // Expose the player's SpacecraftModel (thrusters) under the old name
        // so existing thruster-control and plume code below compiles unchanged.
        auto& orionModel = playerVehicle.model;

        std::vector<std::unique_ptr<Spacecraft>> spacecraft;
        spacecraft.push_back(std::make_unique<Spacecraft>(shipMass, shipInertia, shipState));
        // Attractors are set per-frame from gravBodies below.

        // Index of the spacecraft the player controls / camera follows.
        // Not const — can be reassigned on docking handover or control transfer.
        size_t playerIdx = 0;

        // ---- AI craft: create one physics object per non-player vehicle ----
        // Appended in vehicleConfig order so aiGltfs[ai] == spacecraft[ai+1].
        for (size_t vi = 0; vi < vehicleConfigs.size(); ++vi) {
            if (vi == playerVehicleIdx) continue;
            auto& vc = vehicleConfigs[vi];
            astro::State st;
            if (vc.hasOrbit) {
                astro::PosState ps = spacecraft::vehicleStateAtEt(vc, et);
                glm::dvec3 r = glm::dvec3(ps.r), v = glm::dvec3(ps.v);
                glm::dvec3 T = glm::normalize(v);
                glm::dvec3 N = glm::normalize(glm::cross(r, v));
                glm::dvec3 R = glm::cross(T, N);
                st.P.r = r;
                st.P.v = v;
                st.R.q = glm::quat_cast(glm::dmat3(T, -R, N));
                st.R.w = glm::dvec3(0.0);
            }
            // Fallback mass/inertia: generic station-class values.
            const double m = vc.massKg > 1.0f
                ? static_cast<double>(vc.massKg) : 100000.0;
            const glm::dmat3 inertia = glm::length(vc.inertiaDiag) > 0.0
                ? glm::dmat3(
                    glm::dvec3(vc.inertiaDiag.x, 0, 0),
                    glm::dvec3(0, vc.inertiaDiag.y, 0),
                    glm::dvec3(0, 0, vc.inertiaDiag.z))
                : glm::dmat3(1.0e9);
            spacecraft.push_back(std::make_unique<Spacecraft>(m, inertia, st));
        }

        // MFD apps — updated and rendered every frame in Nav view.
        OrbitalMFD orbitalMFD;
        orbitalMFD.setContext(cfg.observerBody.c_str(), "ECLIPJ2000");
        // TGT list: all non-player spacecraft names (indices match spacecraft[]).
        {
            std::vector<std::string> tgtNames;
            for (size_t vi = 0; vi < vehicleConfigs.size(); ++vi)
                if (vi != playerVehicleIdx)
                    tgtNames.push_back(vehicleConfigs[vi].name.empty()
                                       ? "SC" + std::to_string(vi)
                                       : vehicleConfigs[vi].name);
            orbitalMFD.setTargets(tgtNames);
        }

        // Frame options: ECLIPJ2000 (identity) and J2000 (equatorial).
        // pxform_c is constant for inertial frames; et=0 is fine.
        {
            SpiceDouble m[3][3];
            pxform_c("ECLIPJ2000", "J2000", 0.0, m);
            glm::dmat3 eclToJ2000;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    eclToJ2000[c][r] = m[r][c];
            orbitalMFD.setFrames({{"ECLIPJ2000", glm::dmat3(1.0)},
                                   {"J2000",      eclToJ2000}});
        }

        // Local copy of the active reference body — kept in sync with physicsWorld.
        // Type alias for readability.
        using RefBody = spacecraft::PhysicsWorld::RefBody;
        RefBody refBody = physicsWorld.refBody();
        // True when the user manually selected a ref body via the OrbitalMFD REF button.
        // Suppresses automatic SOI-driven ref switches; cleared on natural SOI transitions.
        bool userPinnedRef = false;

        // Thruster parameters — adjustable from the Dev view.
        // Initialise from manifest if loaded, so the slider shows the real value.
        double mainEngineThrust  = (!orionModel.thrusters.empty()
                                    ? static_cast<double>(orionModel.thrusters[0].thrustN)
                                    : 25'700.0);  // N  (SPACE)
        // Aux thrust for scalar fallback — sum of all Auxiliary thruster max values,
        // or ~1760 N (8 × 220 N) if no manifest is loaded.
        double auxEngineThrust   = [&]() -> double {
            double sum = 0.0;
            for (const auto& t : orionModel.thrusters)
                if (t.type == spacecraft::ThrusterType::Auxiliary)
                    sum += static_cast<double>(t.thrustN);
            return sum > 0.0 ? sum : 1'760.0;
        }();
        double rcsThrust         =    400.0;  // N  (WASD/QE)
        double rcsTorque         =  1'000.0;  // N·m (IJKL/UO)
        // In Docking mode WASD desired force is multiplied by this factor so the
        // RCS thrusters get meaningful throttle (pseudoinverse otherwise spreads 400 N
        // across many thrusters → ~14 % each).  Default 5× → ~70 % allocation.
        double dockingTransBoost =      5.0;

        // Physics step constants live in PhysicsWorld.
        constexpr double kPhysStep = spacecraft::PhysicsWorld::kPhysStep;

        // =================================================================
        // GPU stack — destruction order is reverse of declaration order:
        //   renderer → meshes → camera → pipeline → swapchain → allocator → ctx
        // All VMA-backed objects (meshes, swapchain depth image) are freed
        // before the allocator is destroyed.
        // =================================================================
        glfwSetWindowTitle(window, "Apeiron — Initialising GPU…");
        glfwPollEvents();
        apeiron::render::Context      ctx(window);
        apeiron::render::GpuAllocator allocator(ctx);
        apeiron::render::Swapchain    swapchain(ctx, allocator, kWidth, kHeight);
        apeiron::render::Pipeline        pipeline (ctx, swapchain, APEIRON_SHADER_DIR);
        apeiron::render::TonemapPipeline tonemap  (ctx, swapchain, APEIRON_SHADER_DIR);

        // Meshes and textures declared after allocator — destroyed before allocator.
        // Textures declared before renderer so they outlive it (destroyed after renderer).
        using Tex = apeiron::render::Texture;
        struct BodyTextures { Tex diffuse, specular, normal, clouds, height; };

        auto load = [&](const std::string& path, Tex fallback, bool linear = false) -> Tex {
            return path.empty() ? std::move(fallback)
                                : Tex(ctx, allocator, path, linear);
        };

        std::vector<std::unique_ptr<apeiron::render::Mesh>> meshes;
        std::vector<BodyTextures> bodyTextures;
        bodyTextures.reserve(bodyInfos.size());
        for (auto& bi : bodyInfos) {
            glfwSetWindowTitle(window,
                ("Apeiron — Loading " + bi.node->naifName() + "…").c_str());
            glfwPollEvents();

            auto [verts, idxs] = bi.meshPath.empty()
                ? apeiron::render::Geometry::makeSphere(1.0f, 64, 64, bi.color)
                : apeiron::render::Geometry::loadObj(bi.meshPath, bi.color);
            meshes.push_back(std::make_unique<apeiron::render::Mesh>(
                allocator, verts, idxs));
            bodyTextures.push_back({
                load(bi.diffusePath,   Tex::makeWhite        (ctx, allocator)),
                load(bi.specularPath,  Tex::makeBlack        (ctx, allocator)),
                load(bi.normalPath,    Tex::makeNeutralNormal(ctx, allocator)),
                load(bi.cloudsPath,    Tex::makeBlack        (ctx, allocator)),
                load(bi.heightmapPath, Tex::makeBlack        (ctx, allocator), /*linear=*/true)
            });
        }

        // Ring resources — parallel to bodyInfos; null unique_ptr = no ring.
        std::vector<std::unique_ptr<apeiron::render::Mesh>> ringMeshes(bodyInfos.size());
        std::vector<std::unique_ptr<Tex>>                   ringTextures(bodyInfos.size());
        for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
            auto& bi = bodyInfos[i];
            if (bi.ringTexturePath.empty()) continue;
            glfwSetWindowTitle(window,
                ("Apeiron — Loading " + bi.node->naifName() + " rings…").c_str());
            glfwPollEvents();
            auto [verts, idxs] = apeiron::render::Geometry::makeRing(
                bi.ringInnerRadius, bi.ringOuterRadius, 256);
            ringMeshes[i]   = std::make_unique<apeiron::render::Mesh>(allocator, verts, idxs);
            ringTextures[i] = std::make_unique<Tex>(ctx, allocator, bi.ringTexturePath);
        }

        // Orbit camera: azimuth/elevation around the focused body, Z = ecliptic north.
        struct OrbitCamera {
            float azimuthDeg   =   0.0f;
            float elevationDeg =  60.0f;
            float distanceKm   = 1'000'000.0f;

            glm::vec3 offset() const {
                float az = glm::radians(azimuthDeg);
                float el = glm::radians(elevationDeg);
                return distanceKm * glm::vec3(
                    std::cos(el) * std::cos(az),
                    std::cos(el) * std::sin(az),
                    std::sin(el));
            }
        } orbit;

        // Active camera node for Nav view (cycles through cam_* nodes with C key).
        std::vector<std::string> navCamNodes;
        int navCamIdx = 0;
        // Populated after GLB load below.

        // Ship inspection orbit camera (F11 view).
        struct ShipOrbit {
            float azimuthDeg   =  45.0f;
            float elevationDeg =  20.0f;
            float distanceM    =  30.0f;   // metres (ship is ~10 m scale)
            bool  bodyFrame    =  true;    // true = orbit in ship body frame

            // Returns camera offset in either body frame (caller rotates to inertial)
            // or directly in render space depending on bodyFrame flag.
            glm::vec3 localOffset() const {
                float az = glm::radians(azimuthDeg);
                float el = glm::radians(elevationDeg);
                // Convert metres → km (render space is in km).
                float dKm = distanceM * 1e-3f;
                return dKm * glm::vec3(
                    std::cos(el) * std::cos(az),
                    std::cos(el) * std::sin(az),
                    std::sin(el));
            }
        } shipOrbit;

        // Index of the spacecraft shown in F11 inspection view.
        int inspectIdx = static_cast<int>(playerIdx);

        // Default focus: the observer body (first body matching cfg.observerBody).
        int selectedBodyIndex = 0;
        for (int i = 0; i < static_cast<int>(bodyInfos.size()); ++i)
            if (bodyInfos[i].node->naifName() == cfg.observerBody) { selectedBodyIndex = i; break; }

        glm::vec3 focusRenderPos{0.0f};

        apeiron::render::Camera camera(
            focusRenderPos + orbit.offset(),
            focusRenderPos,
            glm::vec3(0.0f, 0.0f, 1.0f),   // Z = ecliptic north
            45.0f,
            static_cast<float>(kWidth) / static_cast<float>(kHeight),
            0.1f, 1.0e9f  // must match C_NEAR / C_FAR in triangle.frag
        );

        // Bloom post-process (constructed before Renderer so Renderer can bind its output).
        apeiron::render::BloomPass bloom(ctx, allocator, swapchain, APEIRON_SHADER_DIR);

        // Atmosphere pipeline (shares HDR render pass with body pipeline).
        apeiron::render::AtmospherePipeline atmospherePipeline(
            ctx, pipeline.renderPass(), APEIRON_SHADER_DIR);

        // Atmosphere LUT precomputation (per-planet; null where no atmosphere).
        using AtmPre = apeiron::render::AtmospherePrecompute;
        std::vector<std::unique_ptr<AtmPre>> atmPrecompute(bodyInfos.size());
        for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
            auto& bi = bodyInfos[i];
            if (!bi.hasAtmosphere) continue;
            glfwSetWindowTitle(window,
                ("Apeiron — Precomputing " + bi.node->naifName() + " atmosphere…").c_str());
            glfwPollEvents();
            auto& a = bi.atmosphere;
            atmPrecompute[i] = std::make_unique<AtmPre>(ctx, allocator,
                APEIRON_SHADER_DIR,
                AtmPre::Params{
                    bi.radiusKm,
                    a.atmosphereRadius,
                    a.rayleigh,
                    a.rayleighScaleH,
                    a.mieScattering,
                    a.mieExtinction,
                    a.mieScaleH,
                    a.mieG
                });
        }

        // Unit-sphere mesh used for all atmosphere shells (lower resolution than bodies).
        auto [atmVerts, atmIdxs] = apeiron::render::Geometry::makeSphere(
            1.0f, 64, 32, glm::vec3(1.0f));
        apeiron::render::Mesh atmShellMesh(allocator, atmVerts, atmIdxs);

        // Star field — loaded after SPICE kernels are in memory (pxform_c needs them).
        glfwSetWindowTitle(window, "Apeiron — Loading star catalog…");
        glfwPollEvents();
        auto starVertices = loadStars(APEIRON_STAR_CATALOG, 7.5f);
        apeiron::render::StarField starField(ctx, allocator,
                                             pipeline.renderPass(), swapchain,
                                             APEIRON_SHADER_DIR, std::move(starVertices));

        // Mesh pipeline for rigid glTF models (spacecraft, etc.).
        apeiron::render::MeshPipeline meshPipeline(ctx, pipeline, APEIRON_SHADER_DIR);

        // Load player (Orion) glTF model — path from vehicle config.
        apeiron::render::GltfModel orionGltf;
        {
            std::filesystem::path glb = playerVehicle.glbPath.empty()
                ? std::filesystem::path{}
                : std::filesystem::path(APEIRON_DATA_DIR) / playerVehicle.glbPath;
            if (!glb.empty() && std::filesystem::exists(glb)) {
                glfwSetWindowTitle(window, ("Apeiron — Loading " + playerVehicle.name + " model…").c_str());
                glfwPollEvents();
                orionGltf.load(ctx, allocator, meshPipeline, glb);
                std::cout << "[Apeiron] Loaded " << playerVehicle.name << " glTF: "
                          << orionGltf.nodes().size() << " nodes\n";
                // Hide all exhaust plume nodes at startup (throttle = 0).
                for (const auto& t : orionModel.thrusters)
                    if (!t.exhaustNode.empty())
                        orionGltf.setNodeVisible(t.exhaustNode, false);

                // Collect all cam_* nodes in order.
                for (const auto& n : orionGltf.nodes())
                    if (n.name.rfind("cam_", 0) == 0)
                        navCamNodes.push_back(n.name);
                std::sort(navCamNodes.begin(), navCamNodes.end());
                // Put cam_nav_main first.
                auto mainIt = std::find(navCamNodes.begin(), navCamNodes.end(), "cam_nav_main");
                if (mainIt != navCamNodes.end())
                    std::rotate(navCamNodes.begin(), mainIt, mainIt + 1);
                std::cout << "[Apeiron] Nav cameras:";
                for (auto& c : navCamNodes) std::cout << " " << c;
                std::cout << "\n";
            } else {
                std::cerr << "[Apeiron] WARNING: " << playerVehicle.name
                          << " GLB not found at " << glb << "\n";
            }
        }

        // Load AI craft glTF models — one entry per non-player vehicle (index = scIdx - 1).
        std::vector<std::unique_ptr<apeiron::render::GltfModel>> aiGltfs;
        for (size_t vi = 1; vi < vehicleConfigs.size(); ++vi) {
            auto& vc = vehicleConfigs[vi];
            auto gltf = std::make_unique<apeiron::render::GltfModel>();
            std::filesystem::path glb = vc.glbPath.empty()
                ? std::filesystem::path{}
                : std::filesystem::path(APEIRON_DATA_DIR) / vc.glbPath;
            if (!glb.empty() && std::filesystem::exists(glb)) {
                glfwSetWindowTitle(window, ("Apeiron — Loading " + vc.name + " model…").c_str());
                glfwPollEvents();
                gltf->load(ctx, allocator, meshPipeline, glb);
                std::cout << "[Apeiron] Loaded " << vc.name << " glTF: "
                          << gltf->nodes().size() << " nodes\n";
            } else if (!glb.empty()) {
                std::cerr << "[Apeiron] WARNING: " << vc.name
                          << " GLB not found at " << glb << "\n";
            }
            aiGltfs.push_back(std::move(gltf));
        }

        // OffscreenCams declared BEFORE Renderer so they are destroyed AFTER Renderer
        // (C++ reverse-construction order).  Renderer::~Renderer() calls device.waitIdle()
        // first, ensuring the GPU is idle when OffscreenCam resources are released.
        OffscreenCam offscreenCam;
        OffscreenCam dockingOffscreenCam;

        // Renderer last — its destructor calls waitIdle before anything else frees.
        apeiron::render::Renderer renderer(ctx, swapchain, pipeline, tonemap, bloom,
                                           atmospherePipeline);
        renderer.initImGui(window);
        glfwSetWindowTitle(window, "Apeiron");

        // Descriptor sets — allocated from Renderer's pool, freed with it.
        std::vector<vk::DescriptorSet> descriptorSets;
        for (auto& bt : bodyTextures)
            descriptorSets.push_back(renderer.allocateDescriptorSet(
                {bt.diffuse.imageView(), bt.specular.imageView(),
                 bt.normal.imageView(),  bt.clouds.imageView(),
                 bt.height.imageView()},
                {bt.diffuse.sampler(),   bt.specular.sampler(),
                 bt.normal.sampler(),    bt.clouds.sampler(),
                 bt.height.sampler()}));

        // Ring descriptor sets — null handle where body has no ring.
        std::vector<vk::DescriptorSet> ringDescSets(bodyInfos.size());
        for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
            if (!ringTextures[i]) continue;
            auto& rt = *ringTextures[i];
            ringDescSets[i] = renderer.allocateDescriptorSet(
                {rt.imageView(), rt.imageView(), rt.imageView(), rt.imageView(), rt.imageView()},
                {rt.sampler(),   rt.sampler(),   rt.sampler(),   rt.sampler(),   rt.sampler()});
        }

        // Atmosphere descriptor sets — null handle where body has no atmosphere.
        std::vector<vk::DescriptorSet> atmDescSets(bodyInfos.size());
        for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
            if (!atmPrecompute[i]) continue;
            auto& ap = *atmPrecompute[i];
            atmDescSets[i] = renderer.allocateAtmosphereDescriptorSet(
                ap.lutView(), ap.lutSampler());
        }

        // Register transmittance LUTs with ImGui for the inspector window.
        // ImGui_ImplVulkan_AddTexture returns a VkDescriptorSet castable to ImTextureID.
        std::vector<VkDescriptorSet> atmLutImGuiSets(bodyInfos.size(), VK_NULL_HANDLE);
        for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
            if (!atmPrecompute[i]) continue;
            auto& ap = *atmPrecompute[i];
            atmLutImGuiSets[i] = ImGui_ImplVulkan_AddTexture(
                static_cast<VkSampler>   (ap.lutSampler()),
                static_cast<VkImageView> (ap.lutView()),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }

        // Register body diffuse textures with ImGui so MapMFD can display them.
        // Body textures are equirectangular maps — perfect for the flat map view.
        std::unordered_map<std::string, ImTextureID> bodyDiffuseTexIds;
        for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
            std::string nm = bodyInfos[i].node->naifName();
            for (auto& c : nm) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            auto& tex = bodyTextures[i].diffuse;
            if (tex.imageView() && tex.sampler()) {
                bodyDiffuseTexIds[nm] = reinterpret_cast<ImTextureID>(
                    ImGui_ImplVulkan_AddTexture(
                        static_cast<VkSampler>  (tex.sampler()),
                        static_cast<VkImageView>(tex.imageView()),
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL));
            }
        }

        // Initialise offscreen camera render targets (must follow initImGui so that
        // ImGui_ImplVulkan_AddTexture is available for the per-frame color images).
        offscreenCam.init(ctx, allocator, pipeline, swapchain);
        dockingOffscreenCam.init(ctx, allocator, pipeline, swapchain);

        // View mode: F1=Nav, F2=MFD fullscreen, F3=MFD fullscreen 2, F11=Ship inspect, F12=Dev.
        enum class ViewMode { Dev, Nav, MfdFull, MfdFull2, ShipInspect };
        ViewMode viewMode = ViewMode::Dev;

        // Navigation sub-mode state.
        NavState nav;

        // Docking ports per spacecraft (index parallel to spacecraft[]).
        // Extracted from the glTF nodes named docking_port_active_* / docking_port_passive_*.
        using DockPort = apeiron::render::GltfModel::DockingPort;
        std::vector<std::vector<DockPort>> scPorts(spacecraft.size());
        scPorts[playerIdx] = orionGltf.dockingPorts();
        // Orion's GLB is exported with a 90° Rx rollFix applied during rendering
        // (attRot * Rx(90°) * model).  Bake the same rotation into the port axes so
        // that DockingMFD can use body-frame directions directly.
        {
            const glm::mat3 rf = glm::mat3(glm::rotate(glm::mat4(1.0f),
                                                        glm::radians(90.0f),
                                                        glm::vec3(1.0f, 0.0f, 0.0f)));
            for (auto& p : scPorts[playerIdx]) {
                p.posM  = rf * p.posM;
                p.axisX = glm::normalize(rf * p.axisX);
                p.axisZ = glm::normalize(rf * p.axisZ);
            }
        }
        for (size_t ai = 0; ai < aiGltfs.size() && ai + 1 < scPorts.size(); ++ai)
            if (aiGltfs[ai]->isLoaded())
                scPorts[ai + 1] = aiGltfs[ai]->dockingPorts();
        for (size_t i = 0; i < scPorts.size(); ++i) {
            std::cout << "[Apeiron] Spacecraft[" << i << "] docking ports: "
                      << scPorts[i].size() << "\n";
            for (auto& p : scPorts[i])
                std::cout << "  " << p.nodeName
                          << " active=" << p.active
                          << " pos=(" << p.posM.x << "," << p.posM.y << "," << p.posM.z << ")\n";
        }

        // Spacecraft display names (index parallel to spacecraft[]).
        std::vector<std::string> kSpacecraftNames;
        for (const auto& vc : vehicleConfigs)
            kSpacecraftNames.push_back(vc.name.empty() ? "Spacecraft" : vc.name);
        double simSpeedTarget          = 1.0; // set instantly by t/T keys
        double simSecondsPerRealSecond = 1.0; // smoothly tracks target (log-space)
        OBCEventQueue obcEventQueue;          // scheduled mission events (TMI, MOI, TCM…)

        // Window state shared between callbacks.
        struct WindowState {
            apeiron::render::Renderer*  renderer;
            double                      scrollDelta = 0.0;
            ViewMode*                   viewMode    = nullptr;
            double*                     simSpeed    = nullptr;
            std::vector<std::string>*   navCams     = nullptr;
            int*                        navCamIdx   = nullptr;
            spacecraft::Autopilot*      autopilot     = nullptr;
            int*                        inspectIdx    = nullptr;  // F11 object cycler
            int                         numSpacecraft = 0;
            float*                      inspectDist   = nullptr;  // reset on object cycle
            NavState*                   nav           = nullptr;
            DockingConstraint*          dockConstraint = nullptr;
        } windowState{&renderer, 0.0, &viewMode, &simSpeedTarget,
                      &navCamNodes, &navCamIdx, nullptr};

        bool isFullscreen = false;
        bool needsResize  = false;
        int  windowedX = 100, windowedY = 100;

        glfwSetWindowUserPointer(window, &windowState);
        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int scancode, int action, int mods) {
            ImGui_ImplGlfw_KeyCallback(w, key, scancode, action, mods);
            if (action != GLFW_PRESS) return;
            // Suppress all game hotkeys while an ImGui text field has focus,
            // so typing e.g. "MARS" doesn't trigger Shift+M/Shift+R/Shift+S.
            if (ImGui::GetIO().WantCaptureKeyboard) return;
            auto* s = static_cast<WindowState*>(glfwGetWindowUserPointer(w));
            if (key == GLFW_KEY_ESCAPE)
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            else if (key == GLFW_KEY_GRAVE_ACCENT)
                s->renderer->setWireframe(!s->renderer->wireframe());
            else if (key == GLFW_KEY_F1)
                *s->viewMode = ViewMode::Nav;
            else if (key == GLFW_KEY_F2)
                *s->viewMode = ViewMode::MfdFull;
            else if (key == GLFW_KEY_F3)
                *s->viewMode = ViewMode::MfdFull2;
            else if (key == GLFW_KEY_F11)
                *s->viewMode = ViewMode::ShipInspect;
            else if (key == GLFW_KEY_TAB &&
                     *s->viewMode == ViewMode::ShipInspect &&
                     s->inspectIdx && s->numSpacecraft > 0) {
                *s->inspectIdx = (*s->inspectIdx + 1) % s->numSpacecraft;
                // Default view distances: Orion ~30 m, ISS ~300 m for others.
                if (s->inspectDist)
                    *s->inspectDist = (*s->inspectIdx == 0) ? 30.0f : 300.0f;
            }
            else if (key == GLFW_KEY_C &&
                     (*s->viewMode == ViewMode::Nav ||
                      *s->viewMode == ViewMode::MfdFull ||
                      *s->viewMode == ViewMode::MfdFull2) &&
                     !s->navCams->empty())
                *s->navCamIdx = (*s->navCamIdx + 1) % static_cast<int>(s->navCams->size());
            else if (key == GLFW_KEY_F12)
                *s->viewMode = ViewMode::Dev;
            else if (key == GLFW_KEY_K && (mods & GLFW_MOD_SHIFT)) {
                using M = spacecraft::AutopilotMode;
                s->autopilot->mode = (s->autopilot->mode == M::Killrot) ? M::Off : M::Killrot;
            }
            else if (key == GLFW_KEY_P && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Orbit) {
                    using M = spacecraft::AutopilotMode;
                    s->autopilot->mode = (s->autopilot->mode == M::Prograde) ? M::Off : M::Prograde;
                }
            }
            else if (key == GLFW_KEY_R && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Orbit) {
                    using M = spacecraft::AutopilotMode;
                    s->autopilot->mode = (s->autopilot->mode == M::Retrograde) ? M::Off : M::Retrograde;
                }
            }
            else if (key == GLFW_KEY_N && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Orbit) {
                    using M = spacecraft::AutopilotMode;
                    s->autopilot->mode = (s->autopilot->mode == M::NormalPlus) ? M::Off : M::NormalPlus;
                }
            }
            else if (key == GLFW_KEY_M && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Orbit) {
                    using M = spacecraft::AutopilotMode;
                    s->autopilot->mode = (s->autopilot->mode == M::NormalMinus) ? M::Off : M::NormalMinus;
                }
            }
            else if (key == GLFW_KEY_G && (mods & GLFW_MOD_SHIFT)) {
                using M = spacecraft::AutopilotMode;
                s->autopilot->mode = (s->autopilot->mode == M::TcmPlus) ? M::Off : M::TcmPlus;
            }
            else if (key == GLFW_KEY_C && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Tcm)
                    s->nav->tcmCoarseToggle = true;
            }
            else if (key == GLFW_KEY_F && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Tcm)
                    s->nav->tcmFineToggle = true;
            }
            else if (key == GLFW_KEY_V && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Docking && s->nav->dockTgtIdx >= 0) {
                    using M = spacecraft::AutopilotMode;
                    s->autopilot->mode = (s->autopilot->mode == M::NullV) ? M::Off : M::NullV;
                    if (s->autopilot->mode == M::Off)
                        s->autopilot->secondaryMode = M::Off;
                }
            }
            // Shift+= (+V) and Shift+- (-V): relative velocity attitude hold (docking)
            else if ((key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD) && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Docking && s->nav->dockTgtIdx >= 0) {
                    using M = spacecraft::AutopilotMode;
                    const bool isNullV = (s->autopilot->mode == M::NullV);
                    if (isNullV) {
                        s->autopilot->secondaryMode =
                            (s->autopilot->secondaryMode == M::RelVelPlus) ? M::Off : M::RelVelPlus;
                    } else {
                        s->autopilot->secondaryMode = M::Off;
                        s->autopilot->mode =
                            (s->autopilot->mode == M::RelVelPlus) ? M::Off : M::RelVelPlus;
                    }
                }
            }
            else if ((key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_SUBTRACT) && (mods & GLFW_MOD_SHIFT)) {
                if (s->nav && s->nav->navMode == NavMode::Docking && s->nav->dockTgtIdx >= 0) {
                    using M = spacecraft::AutopilotMode;
                    const bool isNullV = (s->autopilot->mode == M::NullV);
                    if (isNullV) {
                        s->autopilot->secondaryMode =
                            (s->autopilot->secondaryMode == M::RelVelMinus) ? M::Off : M::RelVelMinus;
                    } else {
                        s->autopilot->secondaryMode = M::Off;
                        s->autopilot->mode =
                            (s->autopilot->mode == M::RelVelMinus) ? M::Off : M::RelVelMinus;
                    }
                }
            }
            else if (key == GLFW_KEY_T) {
                if (mods & GLFW_MOD_SHIFT)
                    *s->simSpeed = std::max(1.0,   *s->simSpeed / 10.0);  // T = slower
                else {
                    *s->simSpeed = std::min(1.0e6, *s->simSpeed * 10.0);  // t = faster
                    // Disengage autopilot above 10× — controllers become unstable at larger dt.
                    if (*s->simSpeed > 10.0)
                        s->autopilot->mode = spacecraft::AutopilotMode::Off;
                }
            }
            else if (key == GLFW_KEY_ENTER &&
                     s->nav && s->nav->navMode == NavMode::Docking &&
                     s->dockConstraint) {
                s->dockConstraint->initiateHardCapture();
            }
            else if (key == GLFW_KEY_BACKSPACE &&
                     s->nav && s->nav->navMode == NavMode::Docking &&
                     s->dockConstraint) {
                s->dockConstraint->release();
            }
        });
        glfwSetScrollCallback(window, [](GLFWwindow* w, double xoff, double yoff) {
            ImGui_ImplGlfw_ScrollCallback(w, xoff, yoff);
            auto* s = static_cast<WindowState*>(glfwGetWindowUserPointer(w));
            s->scrollDelta += yoff;
        });

        // Forward remaining events ImGui needs (char input, mouse buttons, cursor).
        glfwSetCharCallback(window, [](GLFWwindow* w, unsigned int c) {
            ImGui_ImplGlfw_CharCallback(w, c);
        });
        glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int btn, int action, int mods) {
            ImGui_ImplGlfw_MouseButtonCallback(w, btn, action, mods);
        });
        glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y) {
            ImGui_ImplGlfw_CursorPosCallback(w, x, y);
        });
        glfwSetWindowFocusCallback(window, [](GLFWwindow* w, int focused) {
            ImGui_ImplGlfw_WindowFocusCallback(w, focused);
        });
        glfwSetCursorEnterCallback(window, [](GLFWwindow* w, int entered) {
            ImGui_ImplGlfw_CursorEnterCallback(w, entered);
        });
        glfwSetMonitorCallback([](GLFWmonitor* m, int event) {
            ImGui_ImplGlfw_MonitorCallback(m, event);
        });

        // =================================================================
        // Main loop
        // =================================================================
        double simElapsed = 0.0; // accumulated simulation seconds

        auto prevFrameTime = std::chrono::steady_clock::now();
        double prevMouseX{}, prevMouseY{};
        glfwGetCursorPos(window, &prevMouseX, &prevMouseY);

        // Autopilot.
        spacecraft::Autopilot autopilot;
        bool prevSettleClamp = false;

        // Calibrate rcsAuthorityNm from actual thruster geometry so the
        // parabolic braking-distance estimate in the bang-bang controller
        // matches what the RCS system actually delivers.
        //
        // Saturate a torque demand on each axis, solve RCS-only, then compute
        // the resulting torque analytically from throttle × thrustN × moment arm.
        // This mirrors simulateRcsForce() in SpacecraftModel.cpp — critically,
        // it does NOT use t.firing (set by stepPWM), which would give zero here.
        if (!orionModel.thrusters.empty()) {
            double minAuth = std::numeric_limits<double>::max();
            for (int axis = 0; axis < 3; ++axis) {
                spacecraft::Wrench tw{};
                tw[3 + axis] = 1.0e9;
                orionModel.solveAllocationRcsOnly(tw);

                double auth = 0.0;
                for (const auto& t : orionModel.thrusters) {
                    if (t.throttle < 1e-6f) continue;
                    glm::vec3 arm    = t.position - orionModel.centerOfMass;
                    glm::vec3 torque = glm::cross(arm, t.direction * t.thrustN);
                    auth += static_cast<double>(torque[axis]) * t.throttle;
                }
                auth = std::abs(auth);
                if (auth > 1.0) minAuth = std::min(minAuth, auth);
            }
            for (auto& t : orionModel.thrusters) t.throttle = 0.0f;
            if (minAuth < std::numeric_limits<double>::max()) {
                autopilot.rcsAuthorityNm = minAuth;
                // maxTorqueNm is only used for the direction of tau, not the switch
                // calculation.  Keep it larger than rcsAuthorityNm so the allocator
                // is always saturated and delivers exactly rcsAuthorityNm.
                autopilot.maxTorqueNm = minAuth * 10.0;
            }
        }

        windowState.autopilot     = &autopilot;
        windowState.inspectIdx    = &inspectIdx;
        windowState.numSpacecraft = static_cast<int>(spacecraft.size());
        windowState.inspectDist   = &shipOrbit.distanceM;
        windowState.nav           = &nav;

        NavConsole navConsole;
        NavHUD     navHUD;

        // MFD apps and persistent panels (panels must outlive the loop so
        // isInMenu / app selection survive across frames).
        DockingMFD         dockingMFD;
        dockingMFD.setCamNodes(navCamNodes);   // let DockingMFD cycle / auto-select
        DockingConstraint  dockingConstraint;
        dockingMFD.setConstraint(&dockingConstraint);
        windowState.dockConstraint = &dockingConstraint;

        CamMFD camMFD;
        camMFD.setCamNodes(navCamNodes);

        TransferMFD transferMFD;
        transferMFD.setEpoch(et);

        CislunarMFD cislunarMFD;

        MapMFD mapMFD;
        OBCMFD obcMFD;

        MFDMenu    mfdMenu;
        mfdMenu.addApp(&orbitalMFD,  "ORB");
        mfdMenu.addApp(&dockingMFD,  "DOCK");
        mfdMenu.addApp(&camMFD,      "CAM");
        mfdMenu.addApp(&transferMFD,  "XFER");
        mfdMenu.addApp(&cislunarMFD,  "LUNAR");
        mfdMenu.addApp(&mapMFD,       "MAP");
        mfdMenu.addApp(&obcMFD,      "OBC");

        MFDPanel mfdFullPanel;
        mfdFullPanel.app     = &orbitalMFD;
        mfdFullPanel.menuApp = &mfdMenu;

        MFDPanel mfdFullPanel2;
        mfdFullPanel2.app     = &transferMFD;
        mfdFullPanel2.menuApp = &mfdMenu;

        MFDPanel mfdLeftPanel;
        mfdLeftPanel.app     = &orbitalMFD;
        mfdLeftPanel.menuApp = &mfdMenu;

        MFDPanel mfdRightPanel;
        mfdRightPanel.app     = &dockingMFD;
        mfdRightPanel.menuApp = &mfdMenu;

        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();

            if (needsResize) {
                int fbW = 0, fbH = 0;
                glfwGetFramebufferSize(window, &fbW, &fbH);
                if (fbW > 0 && fbH > 0) {
                    ctx.device().waitIdle();
                    swapchain.recreate(static_cast<uint32_t>(fbW),
                                       static_cast<uint32_t>(fbH));
                    bloom.recreate(swapchain);
                    renderer.recreateFramebuffers();
                    camera.setAspect(static_cast<float>(fbW) /
                                     static_cast<float>(fbH));
                    needsResize = false;
                }
            }

            auto  now     = std::chrono::steady_clock::now();
            float frameDt = std::chrono::duration<float>(now - prevFrameTime).count();
            prevFrameTime = now;
            // Cap frameDt so that defocusing / alt-tabbing doesn't produce a multi-second
            // timestep that blows up the autopilot bang-bang controller or the physics integrator.
            frameDt = std::min(frameDt, 0.1f);

            // --- Orbit camera input (ignored when ImGui is using the mouse) ---
            double mx{}, my{};
            glfwGetCursorPos(window, &mx, &my);
            float dx = static_cast<float>(mx - prevMouseX);
            float dy = static_cast<float>(my - prevMouseY);
            prevMouseX = mx; prevMouseY = my;

            if (viewMode == ViewMode::Dev) {
                if (!ImGui::GetIO().WantCaptureMouse) {
                    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                        orbit.azimuthDeg   -= dx * 0.3f;
                        orbit.elevationDeg += dy * 0.3f;
                        orbit.elevationDeg  = glm::clamp(orbit.elevationDeg, -85.0f, 85.0f);
                    }
                    if (windowState.scrollDelta != 0.0) {
                        orbit.distanceKm *= std::exp(-0.2f * static_cast<float>(windowState.scrollDelta));
                    }
                }

                if (selectedBodyIndex >= 0 && selectedBodyIndex < (int)bodyInfos.size()) {
                    focusRenderPos = scene.origin().toRenderSpace(
                        bodyInfos[selectedBodyIndex].node->worldPosition());
                }
                camera.setPosition(focusRenderPos + orbit.offset());
                camera.setTarget  (focusRenderPos);
                camera.setUp      (glm::vec3(0.0f, 0.0f, 1.0f));  // ecliptic north
            }
            windowState.scrollDelta = 0.0;

            // Tick OBC event queue BEFORE the time advance, using the
            // pre-advance (previous-frame) ET.  This lets the queue see events
            // that are within the current frame's step window and clamp the
            // warp speed before we fly past them.  Both simSpeedTarget and
            // simSecondsPerRealSecond are clamped immediately so the cap
            // takes effect on *this* frame's advance, not just the next one.
            {
                auto preAdvanceEt = et + astro::TimeDelta(simElapsed);
                obcEventQueue.tick(preAdvanceEt.getETValue(),
                                   static_cast<double>(frameDt),
                                   simSpeedTarget, simSecondsPerRealSecond);
            }

            // Cap time accel during an automated burn to 10× so the attitude
            // controller remains stable and C3 cutoff is not overshot badly.
            {
                double burnCap = transferMFD.maxSimSpeed();
                const double lunarCap = cislunarMFD.maxSimSpeed();
                if (lunarCap > 0.0 && (burnCap <= 0.0 || lunarCap < burnCap))
                    burnCap = lunarCap;
                if (burnCap > 0.0) {
                    simSpeedTarget          = std::min(simSpeedTarget,          burnCap);
                    simSecondsPerRealSecond = std::min(simSecondsPerRealSecond, burnCap);
                }
            }

            // Drop to 1× on SOI entry so the pilot can arm the arrival burn,
            // then schedule the fine approach TCM reminder sequence.
            if (transferMFD.consumeSoiEntry()) {
                simSpeedTarget          = 1.0;
                simSecondsPerRealSecond = 1.0;
                transferMFD.scheduleFineApproachTcms();
            }

            // Drop to 1× on Moon SOI entry during cislunar coast so the pilot
            // can switch to the LOI page and arm the capture burn.
            if (cislunarMFD.consumeSoiEntry()) {
                simSpeedTarget          = 1.0;
                simSecondsPerRealSecond = 1.0;
            }

            // Drop to 1× on TCM warning rising edge; pilot can speed up freely after.
            if (transferMFD.consumeTcmWarnDrop()) {
                simSpeedTarget          = 1.0;
                simSecondsPerRealSecond = 1.0;
            }

            // Smooth time acceleration: converge in log-space toward target.
            // Time constant ~0.15 s real time — fast enough to feel responsive,
            // slow enough to avoid the one-frame orbital skip on t/T presses.
            // Note: simSecondsPerRealSecond may already have been hard-clamped
            // above; the interpolation then runs from the clamped value.
            {
                const double logCurr = std::log(simSecondsPerRealSecond);
                const double logTarg = std::log(simSpeedTarget);
                const double logNew  = logCurr + (logTarg - logCurr)
                                       * std::min(1.0, static_cast<double>(frameDt) / 0.15);
                simSecondsPerRealSecond = std::exp(logNew);
            }

            simElapsed += frameDt * simSecondsPerRealSecond;
            auto currentEt = et + astro::TimeDelta(simElapsed);

            // ---- Per-frame gravity update ----------------------------------------
            {
                std::vector<Spacecraft*> scPtrs;
                for (auto& s : spacecraft) scPtrs.push_back(s.get());
                auto soiEvt = physicsWorld.updateGravity(
                    scPtrs, spacecraft[playerIdx]->position(), currentEt);
                if (soiEvt.changed) {
                    dominantBodyName = soiEvt.newRefBody.name;
                    if (!userPinnedRef) {
                        // Follow the physics SOI automatically.
                        refBody = soiEvt.newRefBody;
                        orbitalMFD.setContext(refBody.name.c_str(), "");
                    }
                    // Note: userPinnedRef stays set — the user keeps their chosen
                    // display ref until they type a new one or the next natural
                    // SOI transition into a body they haven't pinned.
                }
            }
            // -----------------------------------------------------------------------

            // ---- Thruster input (active in Nav/MfdFull/ShipInspect at 1x only) ----
            // Controls are disabled at time acceleration > 1x — thrust at 1000x makes no sense.
            // Autopilot runs in all three views so switching to MFD fullscreen doesn't freeze it.
            bool mainEngineOn = false;
            glm::dvec3 shipForce(0.0);   // body-frame thrust force (N), hoisted for nav console
            // Active burn-direction for TCM+ autopilot and HUD marker.
            // Active nav ΔV: Lambert TCM while in transit; B-plane Pe targeting in
            // final approach.  Switch to B-plane exclusively above 97% TOF progress
            // (or whenever Pe < 50,000 km) so shift-G stays locked on the approach
            // correction without a jarring mid-course jump.
            const glm::dvec3 activeNavDv = [&]() -> glm::dvec3 {
                // CislunarMFD coast TCM takes priority when it has a valid correction.
                const glm::dvec3 cislunarTcm = cislunarMFD.getTcmDv();
                if (glm::length(cislunarTcm) > 1e-9)
                    return cislunarTcm;

                glm::dvec3 bpl        = transferMFD.getBplaneDv();
                double     peNow      = transferMFD.getBplanePeCurrentKm();
                double     progress   = transferMFD.getTransferProgress();
                const bool lateApproach = (progress >= 0.97);
                const bool bplUsable  = (glm::length(bpl) > 1e-9);
                (void)peNow; (void)lateApproach;
                return bplUsable ? bpl : transferMFD.getTcmDv();
            }();
            {
                glm::dvec3 shipTorque(0.0);

                // Disengage autopilot above 10× — controllers are unstable at larger dt.
                // At ≤10× prograde hold and burn autopilot remain usable.
                if (simSpeedTarget > 10.0)
                    autopilot.mode = spacecraft::AutopilotMode::Off;

                if ((viewMode == ViewMode::Nav || viewMode == ViewMode::MfdFull ||
                     viewMode == ViewMode::MfdFull2 || viewMode == ViewMode::ShipInspect)
                    && simSpeedTarget <= 10.0
                    && !ImGui::GetIO().WantCaptureKeyboard) {

                    if (!orionModel.thrusters.empty()) {
                        // ---- Control allocation path (Orion manifest loaded) ----
                        // Orion model space: +X = aft, engine pointing +X.
                        spacecraft::Wrench desired{};

                        // SHIFT+SPACE fires aux thrusters; plain SPACE fires main engine.
                        const bool shiftHeld =
                            glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS
                            || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
                        const bool spaceDown = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;

                        // ---- TCM burn AP toggle and tick ----
                        {
                            const auto toggleTcmBurn = [&](bool coarse) {
                                if (transferMFD.tcmBurnActive() && transferMFD.tcmBurnCoarse() == coarse)
                                    transferMFD.disarmTcmBurn();
                                else {
                                    transferMFD.armTcmBurn(coarse);
                                    autopilot.mode = spacecraft::AutopilotMode::TcmPlus;
                                }
                            };
                            if (nav.tcmCoarseToggle) { nav.tcmCoarseToggle = false; toggleTcmBurn(true);  }
                            if (nav.tcmFineToggle)   { nav.tcmFineToggle   = false; toggleTcmBurn(false); }

                            if (transferMFD.tcmBurnActive()) {
                                if (autopilot.mode != spacecraft::AutopilotMode::TcmPlus) {
                                    // User changed autopilot mode — abort TCM burn AP.
                                    transferMFD.disarmTcmBurn();
                                } else {
                                    // Use activeNavDv — same vector the attitude AP targets.
                                    // In late approach this is the B-plane dV, not getTcmDv().
                                    // Using getTcmDv() here would mismatch the attitude target
                                    // and keep attErr above threshold, preventing thrust.
                                    const double tcmDvMs = glm::length(activeNavDv) * 1000.0;
                                    double attErr = M_PI;
                                    if (tcmDvMs > 1e-3) {
                                        const glm::dvec3 tcmDirNorm = glm::normalize(activeNavDv);
                                        const glm::dvec3 shipFwd = glm::dvec3(
                                            spacecraft[playerIdx]->attitude() * glm::dvec3(1.0, 0.0, 0.0));
                                        attErr = std::acos(std::clamp(
                                            glm::dot(shipFwd, tcmDirNorm), -1.0, 1.0));
                                    }
                                    if (transferMFD.tickTcmBurn(tcmDvMs, attErr)) {
                                        autopilot.mode = spacecraft::AutopilotMode::Killrot;
                                        transferMFD.consumeTcmDone();
                                    }
                                }
                            }
                            nav.tcmBurnPhase  = transferMFD.tcmBurnPhase();
                            nav.tcmBurnCoarse = transferMFD.tcmBurnCoarse();
                            nav.lambertFlip   = transferMFD.lambertFlipActive();
                        }

                        const bool auxEngineKey  = (spaceDown && shiftHeld)
                                                   || transferMFD.requestTcmAuxEngine();
                        // Main engine (SPACE) — tracked separately so allocation
                        // always uses the full matrix when the main engine is firing.
                        const bool mainEngineKey =
                            (spaceDown && !shiftHeld)
                            || transferMFD.requestMainEngine()
                            || transferMFD.requestTcmMainEngine()
                            || cislunarMFD.requestMainEngine();
                        if (mainEngineKey) {
                            desired[0] += mainEngineThrust;
                            mainEngineOn = true;
                        }
                        if (auxEngineKey) {
                            desired[0] += auxEngineThrust;
                        }
                        // RCS translation (WASD/QE) — always solved RCS-only so the
                        // main engine is never allocated for lateral / retrograde moves.
                        // Aggressive mode scales up desired force so the pseudoinverse
                        // gives each thruster meaningful throttle (fine = ~14%, aggr = ~70%).
                        const double transF = (nav.transMode == TransMode::Aggressive)
                                              ? rcsThrust * dockingTransBoost
                                              : rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) desired[0] += transF;
                        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) desired[0] -= transF;
                        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) desired[1] += transF;
                        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) desired[1] -= transF;
                        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) desired[2] += transF;
                        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) desired[2] -= transF;

                        // Always update relVelBody and RCS authority when a docking target
                        // is selected — T-NV / D-NV must be correct regardless of autopilot mode.
                        if (nav.dockTgtIdx >= 0) {
                            auto& ship = *spacecraft[playerIdx];
                            glm::dquat att = ship.attitude();
                            glm::dvec3 relVel_inertial =
                                (ship.velocity() - spacecraft[static_cast<size_t>(nav.dockTgtIdx)]->velocity())
                                * 1000.0;  // km/s → m/s
                            autopilot.relVelBody = glm::dvec3(glm::conjugate(att) * relVel_inertial);

                            const double vMag = glm::length(autopilot.relVelBody);
                            if (vMag > 1e-6) {
                                // Desired direction opposes relative velocity (braking).
                                // simulateRcsForce internally scales to the just-saturating
                                // point so no arbitrary large constant is needed.
                                glm::dvec3 fDir = -autopilot.relVelBody / vMag;
                                spacecraft::Wrench testW{};
                                testW[0] = fDir.x; testW[1] = fDir.y; testW[2] = fDir.z;
                                glm::vec3 achieved = orionModel.simulateRcsForce(testW);
                                double auth = glm::dot(glm::dvec3(achieved), fDir);
                                autopilot.nullVAuthN = std::max(1.0, auth);
                            }
                        }

                        if (autopilot.active()) {
                            auto& ship = *spacecraft[playerIdx];
                            glm::dquat att    = ship.attitude();
                            glm::dvec3 w_body = glm::conjugate(att) * ship.angularVelocity();

                            // Auto-cancel NullV once done.
                            if (autopilot.nullVDone) {
                                autopilot.mode          = spacecraft::AutopilotMode::Off;
                                autopilot.secondaryMode = spacecraft::AutopilotMode::Off;
                            }

                            // Update autopilot targets for this frame.
                            // Use reference-body-relative state so Prograde/Normal modes
                            // are correct in Mars SOI, heliocentric, etc.
                            glm::dvec3 apR = ship.position();
                            glm::dvec3 apV = ship.velocity();
                            if (refBody.naifId != 399) {
                                static const astro::ReferenceFrame kEclipJ2000ap =
                                    astro::ReferenceFrame::createEclipJ2000();
                                astro::PosState refSt;
                                try {
                                    astro::Spice().getRelativeGeometricState(
                                        static_cast<int>(refBody.naifId), 399,
                                        currentEt, refSt, kEclipJ2000ap);
                                    apR -= glm::dvec3(refSt.r.x, refSt.r.y, refSt.r.z);
                                    apV -= glm::dvec3(refSt.v.x, refSt.v.y, refSt.v.z);
                                } catch (...) {}
                            }
                            autopilot.updateOrbitalTarget(apR, apV, att);
                            autopilot.updateRelVelTarget(att);
                            autopilot.tcmDirInertial = activeNavDv;
                            autopilot.updateTcmTarget(att);

                            bool settleClamp = false;
                            spacecraft::Wrench apWrench = autopilot.compute(
                                att, w_body, glm::dvec3(orionModel.inertiaDiag), frameDt, settleClamp);
                            if (settleClamp) {
                                glm::dvec3 w_ff_inertial = att * autopilot.omegaFF;
                                ship.setAngularVelocity(w_ff_inertial);
                            }
                            prevSettleClamp = settleClamp;
                            // NullV fills [0..2] (translation); attitude modes fill [3..5].
                            for (int i = 0; i < 3; ++i) desired[i] += apWrench[i];
                            for (int i = 3; i < 6; ++i) desired[i]  = apWrench[i];
                        } else {
                            if (prevSettleClamp) {
                                auto& ship = *spacecraft[playerIdx];
                                ship.setAngularVelocity(ship.attitude() * autopilot.omegaFF);
                            }
                            prevSettleClamp = false;
                            // Manual rotation — guard against Shift held, since Shift+I/K/J/L/U/O
                            // are autopilot toggle commands; testing the bare key while Shift is
                            // down causes a one-frame torque spike on AP disengage.
                            if (!shiftHeld) {
                                if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) desired[4] += rcsTorque;
                                if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) desired[4] -= rcsTorque;
                                if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) desired[5] += rcsTorque;
                                if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) desired[5] -= rcsTorque;
                                if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) desired[3] -= rcsTorque;
                                if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) desired[3] += rcsTorque;
                            }
                        }

                        // Allocation strategy:
                        //
                        // Attitude control (autopilot or manual rotation keys) is ALWAYS
                        // RCS-only.  Aux thrusters must never fire for attitude because
                        // they produce significant net linear acceleration (they are not
                        // arranged symmetrically for pure torque), which destabilises
                        // prograde hold and contaminates the trajectory during burns.
                        //
                        // Main engine (SPACE): two-pass split —
                        //   pass 1: full matrix for forward thrust → main engine throttle set
                        //   pass 2: RCS-only for attitude correction → aux stay zero
                        //   then restore main engine throttle (pass 2 zeroed it)
                        //
                        // Aux thrusters (SHIFT+SPACE): same two-pass logic but pass 1
                        //   uses aux-only matrix instead of the full matrix.
                        //
                        // Manual WASD translation: RCS-only (same reason).
                        if (mainEngineKey) {
                            // Pass 1: forward thrust via full matrix.
                            spacecraft::Wrench thrustOnly{};
                            thrustOnly[0] = desired[0];
                            thrustOnly[1] = desired[1];
                            thrustOnly[2] = desired[2];
                            orionModel.solveAllocation(thrustOnly);
                            const float mainThrottle = orionModel.thrusters.empty()
                                                       ? 0.0f : orionModel.thrusters[0].throttle;

                            // Compute residual torque the main engine will produce at
                            // this throttle due to its position offset from the CoM.
                            // Pass 2 must cancel it in addition to the commanded torque.
                            glm::dvec3 mainResidualTorque(0.0);
                            if (!orionModel.thrusters.empty() && mainThrottle > 0.0f) {
                                const auto& me = orionModel.thrusters[0];
                                glm::vec3 arm = me.position - orionModel.centerOfMass;
                                glm::vec3 forceVec = me.direction
                                                   * (me.thrustN * mainThrottle);
                                mainResidualTorque = glm::dvec3(glm::cross(arm, forceVec));
                            }

                            // Pass 2: RCS counteracts desired torque + main engine offset.
                            spacecraft::Wrench torqueOnly{};
                            torqueOnly[3] = desired[3] - mainResidualTorque.x;
                            torqueOnly[4] = desired[4] - mainResidualTorque.y;
                            torqueOnly[5] = desired[5] - mainResidualTorque.z;
                            orionModel.solveAllocationRcsOnly(torqueOnly);
                            if (!orionModel.thrusters.empty())
                                orionModel.thrusters[0].throttle = mainThrottle;
                        } else if (auxEngineKey) {
                            // Pass 1: forward thrust via aux-only matrix.
                            spacecraft::Wrench thrustOnly{};
                            thrustOnly[0] = desired[0];
                            thrustOnly[1] = desired[1];
                            thrustOnly[2] = desired[2];
                            orionModel.solveAllocationAuxOnly(thrustOnly);

                            // Pass 2: RCS handles attitude (aux throttles are preserved).
                            spacecraft::Wrench torqueOnly{};
                            torqueOnly[3] = desired[3];
                            torqueOnly[4] = desired[4];
                            torqueOnly[5] = desired[5];
                            // solveAllocationRcsOnly zeroes all throttles first, so save
                            // aux throttles, run RCS solve, then restore.
                            std::vector<float> auxSaved;
                            auxSaved.reserve(orionModel.thrusters.size());
                            for (const auto& t : orionModel.thrusters)
                                auxSaved.push_back(t.throttle);
                            orionModel.solveAllocationRcsOnly(torqueOnly);
                            for (std::size_t i = 0; i < orionModel.thrusters.size(); ++i)
                                if (orionModel.thrusters[i].type == spacecraft::ThrusterType::Auxiliary)
                                    orionModel.thrusters[i].throttle = auxSaved[i];
                        } else {
                            orionModel.solveAllocationRcsOnly(desired);
                        }
                        orionModel.stepPWM(frameDt);
                        glm::vec3 F{}, T{};
                        orionModel.accumulateWrench(F, T);
                        shipForce  = glm::dvec3(F);
                        shipTorque = glm::dvec3(T);
                    } else {
                        // ---- Scalar fallback (no manifest) ----
                        const bool shiftHeldFb =
                            glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS
                            || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
                        const bool spaceDownFb = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
                        if (spaceDownFb && shiftHeldFb) {
                            shipForce.x += auxEngineThrust;  // SHIFT+SPACE — aux only
                        } else if (spaceDownFb && !shiftHeldFb
                            || transferMFD.requestMainEngine()) {
                            shipForce.x += mainEngineThrust;
                            mainEngineOn = true;
                        }
                        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) shipForce.x += rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) shipForce.x -= rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) shipForce.y += rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) shipForce.y -= rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) shipForce.z += rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) shipForce.z -= rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) shipTorque.y += rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) shipTorque.y -= rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) shipTorque.z += rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) shipTorque.z -= rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) shipTorque.x -= rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) shipTorque.x += rcsTorque;
                    }
                }

                spacecraft[playerIdx]->setBodyForce(shipForce);
                spacecraft[playerIdx]->setBodyTorque(shipTorque);
            }

            // ---- Fixed-step spacecraft physics (simulation time) ----
            {
                const double simDt = static_cast<double>(frameDt) * simSecondsPerRealSecond;
                std::vector<Spacecraft*> scPtrs;
                for (auto& s : spacecraft) scPtrs.push_back(s.get());
                physicsWorld.integrate(
                    simDt, currentEt, scPtrs,
                    [&](double dt) { dockingConstraint.update(dt); },
                    [&](double)    { dockingConstraint.enforcePostStep(); });
            }

            // Pin non-player orbiting spacecraft to their analytical Keplerian orbit.
            // Prevents Lyapunov instability (e.g. NRHO) and RKF78 accumulation errors
            // from diverging at high time warp. Applied after RKF78 so the physics world
            // gravity/SOI detection still runs normally (using the player's position).
            {
                for (size_t vi = 0; vi < vehicleConfigs.size(); ++vi) {
                    if (vi == playerIdx) continue;
                    const auto& vc = vehicleConfigs[vi];
                    if (!vc.hasOrbit) continue;
                    astro::PosState st = spacecraft::vehicleStateAtEt(vc, currentEt);
                    spacecraft[vi]->setPosition(glm::dvec3(st.r));
                    spacecraft[vi]->setVelocity(glm::dvec3(st.v));
                }
                // Re-enforce rigid docking lock so the active (child) spacecraft
                // follows the Keplerian-corrected passive position.
                if (dockingConstraint.phase() == DockingConstraint::Phase::HardCapture)
                    dockingConstraint.enforcePostStep();
            }

            observerPos = observer.worldPosition(currentEt);
            scene.update(currentEt, observerPos);

            // Cache Earth's world position AFTER scene.update() so node positions are
            // current.  All spacecraft positions are Earth-centred ECI (vehicleStateAtEt
            // always converts to Earth-ECI regardless of the orbit's central_body), so
            // this must always be Earth — not the scenario observer body.
            glm::dvec3 refBodyWorld(0.0);
            for (auto& bi : bodyInfos)
                if (bi.node->naifName() == "EARTH") { refBodyWorld = bi.node->worldPosition(); break; }

            // ---- Docking port proximity & matching logic ----
            // Runs every frame; keeps nav.dockPortIdx and nav.compatiblePortCount fresh.
            if (nav.navMode == NavMode::Docking && nav.dockTgtIdx >= 0) {
                auto& player = *spacecraft[playerIdx];
                auto& tgt    = *spacecraft[static_cast<size_t>(nav.dockTgtIdx)];
                constexpr double kPortSelectKm = 5.0;
                double distKm = glm::length(player.position() - tgt.position());

                // Count compatible ports: player must have ≥1 port, and we count
                // target ports whose active flag is opposite to any player port.
                bool playerHasActive  = false;
                bool playerHasPassive = false;
                for (auto& pp : scPorts[playerIdx]) {
                    if (pp.active)  playerHasActive  = true;
                    else            playerHasPassive = true;
                }

                int compatible = 0;
                if (distKm <= kPortSelectKm) {
                    for (auto& tp : scPorts[static_cast<size_t>(nav.dockTgtIdx)]) {
                        // active/passive match: probe docks with drogue
                        if ((tp.active && playerHasPassive) ||
                            (!tp.active && playerHasActive))
                            ++compatible;
                    }
                }
                nav.compatiblePortCount = compatible;

                // If we move outside 5 km, fall back to spacecraft-level target.
                if (distKm > kPortSelectKm && nav.dockPortIdx >= 0) {
                    nav.dockPortIdx = -1;
                }
                // If no compatible ports, clear any port selection.
                if (compatible == 0 && nav.dockPortIdx >= 0) {
                    nav.dockPortIdx = -1;
                }
            } else {
                nav.compatiblePortCount = 0;
            }

            // ---- Docking constraint arm/disarm ----
            // Configure the constraint whenever docking mode has a valid target + port.
            // arm() sets phase to Unarmed; the user must press ARM in the MFD to activate
            // capture monitoring.  Hard capture is self-sustaining — never auto-disarm it.
            {
                using DP = DockingConstraint::Phase;
                static DP    lastDockPhase = DP::Idle;
                static int   lastArmedTgt  = -2;
                static int   lastArmedPort = -2;
                // Range callout thresholds (metres), descending.
                static const float kRangeCallouts[] = { 1000.f, 500.f, 300.f, 200.f,
                                                         100.f,  50.f,  20.f,  10.f,
                                                           5.f,   2.f,   1.f };
                static int lastRangeBand = -1;  // index of last announced threshold

                const bool hardCaptured =
                    dockingConstraint.phase() == DP::HardCapture;

                const bool shouldConfigure =
                    !hardCaptured &&
                    nav.navMode == NavMode::Docking &&
                    nav.dockTgtIdx >= 0 &&
                    nav.dockPortIdx >= 0 &&
                    !scPorts[playerIdx].empty() &&
                    nav.dockPortIdx < static_cast<int>(
                        scPorts[static_cast<size_t>(nav.dockTgtIdx)].size());

                if (shouldConfigure) {
                    if (nav.dockTgtIdx != lastArmedTgt || nav.dockPortIdx != lastArmedPort) {
                        // Port selection changed — reconfigure (resets to Unarmed).
                        dockingConstraint.arm(
                            spacecraft[playerIdx].get(),
                            scPorts[playerIdx][0],
                            spacecraft[static_cast<size_t>(nav.dockTgtIdx)].get(),
                            scPorts[static_cast<size_t>(nav.dockTgtIdx)][
                                static_cast<size_t>(nav.dockPortIdx)]);
                        lastArmedTgt  = nav.dockTgtIdx;
                        lastArmedPort = nav.dockPortIdx;
                        lastRangeBand = -1;

                        // Announce target + port selection with current range.
                        const auto& tgtName  = (nav.dockTgtIdx < static_cast<int>(kSpacecraftNames.size()))
                                               ? kSpacecraftNames[nav.dockTgtIdx] : "target";
                        const auto& portLabel = scPorts[static_cast<size_t>(nav.dockTgtIdx)]
                                                       [static_cast<size_t>(nav.dockPortIdx)].label;
                        float rangeM = static_cast<float>(
                            glm::length(spacecraft[static_cast<size_t>(nav.dockTgtIdx)]->position()
                                        - spacecraft[playerIdx]->position()) * 1000.0);
                        char buf[128];
                        if (rangeM < 1000.f)
                            std::snprintf(buf, sizeof(buf), "Port %s on %s. Range %.0f metres.",
                                          portLabel.c_str(), tgtName.c_str(), rangeM);
                        else
                            std::snprintf(buf, sizeof(buf), "Port %s on %s. Range %.1f kilometres.",
                                          portLabel.c_str(), tgtName.c_str(), rangeM * 1e-3f);
                        obc::speak(buf);
                    }
                } else if (!hardCaptured &&
                           dockingConstraint.phase() != DP::Idle) {
                    dockingConstraint.disarm();
                    lastArmedTgt = lastArmedPort = -2;
                    lastRangeBand = -1;
                }

                // ---- Phase-change announcements ----
                const DP curPhase = dockingConstraint.phase();
                if (curPhase != lastDockPhase) {
                    switch (curPhase) {
                        case DP::Armed:
                            obc::speak("Armed. Capture monitoring active.");
                            break;
                        case DP::SoftCapture:
                            obc::speakImmediate("Soft capture.");
                            break;
                        case DP::HardCapture:
                            obc::speakImmediate("Hard capture confirmed.");
                            break;
                        case DP::Unarmed:
                            if (lastDockPhase == DP::HardCapture ||
                                lastDockPhase == DP::SoftCapture)
                                obc::speak("Released.");
                            break;
                        default: break;
                    }
                    lastDockPhase = curPhase;
                }

                // ---- Range callouts (Armed phase, approaching) ----
                if (curPhase == DP::Armed || curPhase == DP::Unarmed) {
                    // Use constraint's port-to-port range when Armed; CoM distance otherwise.
                    const float rangeM = dockingConstraint.rangeM() > 0.0f
                        ? dockingConstraint.rangeM()
                        : (nav.dockTgtIdx >= 0
                            ? static_cast<float>(
                                glm::length(spacecraft[static_cast<size_t>(nav.dockTgtIdx)]->position()
                                            - spacecraft[playerIdx]->position()) * 1000.0)
                            : 0.0f);
                    constexpr int kN = static_cast<int>(
                        sizeof(kRangeCallouts) / sizeof(kRangeCallouts[0]));
                    // Find the deepest (smallest) threshold we have crossed.
                    // Iterate all thresholds and keep the last match — this gives the
                    // most-recently-crossed threshold as range decreases.
                    int band = -1;  // -1 = above all thresholds, no callout
                    for (int i = 0; i < kN; ++i) {
                        if (rangeM < kRangeCallouts[i]) band = i;
                    }
                    // Initialise on first run after arm (suppress spurious callout).
                    if (lastRangeBand == -1) lastRangeBand = band;
                    // Announce when we cross into a new (deeper) band.
                    if (band > lastRangeBand) {
                        char buf[64];
                        if (kRangeCallouts[band] >= 1000.f)
                            std::snprintf(buf, sizeof(buf), "%.0f kilometre.",
                                          kRangeCallouts[band] * 1e-3f);
                        else
                            std::snprintf(buf, sizeof(buf), "%.0f %s.",
                                          kRangeCallouts[band],
                                          kRangeCallouts[band] < 1.5f ? "metre" : "metres");
                        obc::speak(buf);
                        lastRangeBand = band;
                    }
                    // Reset if range increases back above the last threshold.
                    if (band < lastRangeBand) lastRangeBand = band;
                }
            }

            // Recenter the floating origin.
            // In Nav view, track the player ship; in Map view, track the selected body.
            if (viewMode == ViewMode::Nav || viewMode == ViewMode::MfdFull ||
                viewMode == ViewMode::MfdFull2) {
                // Recenter on the interpolated position — same point the camera sits at.
                // Using the true physics position would leave a step-size-dependent offset
                // between camera and origin, causing Earth to jump when time accel changes.
                scene.origin().recenter(refBodyWorld + spacecraft[playerIdx]->position());
            } else if (viewMode == ViewMode::ShipInspect) {
                // Recenter on the inspected object so its render-space position is always
                // near the float origin — prevents precision jitter for far-away objects.
                scene.origin().recenter(refBodyWorld + spacecraft[inspectIdx]->position());
            } else if (selectedBodyIndex >= 0 && selectedBodyIndex < (int)bodyInfos.size()) {
                scene.origin().recenter(
                    bodyInfos[selectedBodyIndex].node->worldPosition());
            }

            // Sun's render-space position (used as light source for all bodies).
            glm::vec3 sunRenderPos{0.0f};
            if (sunIndex >= 0) {
                sunRenderPos = scene.origin().toRenderSpace(
                    bodyInfos[sunIndex].node->worldPosition());
            }

            // Near plane: planet views need 0.1 km (avoids z-fight at orbital distance);
            // ship inspect needs 0.00001 km (1 cm) to see a 10 m object at 30 m.
            camera.setNear(viewMode == ViewMode::ShipInspect ? 1e-3f : 0.1f);

            // ---- Nav / MfdFull camera: driven by cam_* node from the glTF model ----
            if (viewMode == ViewMode::Nav || viewMode == ViewMode::MfdFull ||
                viewMode == ViewMode::MfdFull2) {
                auto& ship = *spacecraft[playerIdx];
                glm::vec3 shipRp = scene.origin().toRenderSpace(
                    refBodyWorld + ship.position());

                glm::mat3 attRot3 = glm::mat3_cast(glm::fquat(ship.attitude()));

                // rollFix matches the one applied in the draw call.
                glm::mat4 rollFix = glm::rotate(glm::mat4(1.0f),
                                                glm::radians(90.0f),
                                                glm::vec3(1.0f, 0.0f, 0.0f));
                glm::mat4 shipRot = glm::mat4(glm::mat3(attRot3)) * rollFix;

                if (orionGltf.isLoaded() && !navCamNodes.empty()) {
                    // In MfdFull mode, let DockingMFD override the camera node.
                    const std::string& dockCam = dockingMFD.preferredCamNode();
                    const std::string& activeCam =
                        (viewMode == ViewMode::MfdFull
                         && mfdFullPanel.app == static_cast<MFDApp*>(&dockingMFD)
                         && !dockCam.empty())
                        ? dockCam
                        : navCamNodes[navCamIdx];
                    // Node transform is in metres (model space); directions are unit vectors.
                    glm::mat4 nodeTf = orionGltf.nodeWorldTransform(activeCam);
                    glm::mat4 camWorld = shipRot * nodeTf;

                    // Position: translate to ship, then apply node offset (metres → km).
                    glm::vec3 pos = shipRp + glm::vec3(camWorld[3]) * 1e-3f;
                    glm::vec3 fwd = glm::normalize(glm::vec3(camWorld[0]));  // +X = forward
                    glm::vec3 up  = glm::normalize(glm::vec3(camWorld[1]));  // +Y in glTF = Blender +Z = up

                    camera.setPosition(pos);
                    camera.setTarget  (pos + fwd);
                    camera.setUp      (up);
                } else {
                    // Fallback: bow camera at ship centre.
                    glm::vec3 fwd = glm::mat3(shipRot) * glm::vec3(1.0f, 0.0f, 0.0f);
                    glm::vec3 up  = glm::mat3(shipRot) * glm::vec3(0.0f, 1.0f, 0.0f);
                    camera.setPosition(shipRp);
                    camera.setTarget  (shipRp + fwd);
                    camera.setUp      (up);
                }
            }

            // ---- F11 ship inspection camera ----
            if (viewMode == ViewMode::ShipInspect) {
                auto& ship = *spacecraft[inspectIdx];
                glm::vec3 shipRp = scene.origin().toRenderSpace(
                    refBodyWorld + ship.position());

                // Accept drag (left-button) only when ImGui isn't consuming mouse,
                // but always accept scroll so touchpad two-finger zoom works even
                // when the HUD overlay has focus.
                if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS &&
                    !ImGui::GetIO().WantCaptureMouse) {
                    shipOrbit.azimuthDeg   -= dx * 0.4f;
                    shipOrbit.elevationDeg += dy * 0.4f;
                    shipOrbit.elevationDeg  = glm::clamp(shipOrbit.elevationDeg, -85.0f, 85.0f);
                }
                if (windowState.scrollDelta != 0.0)
                    shipOrbit.distanceM *= std::exp(-0.3f * static_cast<float>(windowState.scrollDelta));

                glm::vec3 offset = shipOrbit.localOffset();  // in km, local frame

                glm::mat3 rot = glm::mat3_cast(glm::fquat(ship.attitude()));
                if (shipOrbit.bodyFrame)
                    offset = rot * offset;  // rotate into inertial render space

                // Up vector: body +Z in body frame, ecliptic north in inertial.
                glm::vec3 up = shipOrbit.bodyFrame
                    ? rot * glm::vec3(0.0f, 0.0f, 1.0f)
                    : glm::vec3(0.0f, 0.0f, 1.0f);

                camera.setPosition(shipRp + offset);
                camera.setTarget  (shipRp);
                camera.setUp      (up);
            }

            // --- ImGui frame ---
            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            if (viewMode == ViewMode::Dev) ImGui::Begin("Dev View");
            if (viewMode == ViewMode::Dev) {
            ImGui::Text("%.1f fps  (%.2f ms)", ImGui::GetIO().Framerate,
                        1000.0f / ImGui::GetIO().Framerate);
            if (ImGui::Checkbox("Fullscreen", &isFullscreen)) {
                if (isFullscreen) {
                    glfwGetWindowPos(window, &windowedX, &windowedY);
                    GLFWmonitor* mon = glfwGetPrimaryMonitor();
                    const GLFWvidmode* mode = glfwGetVideoMode(mon);
                    glfwSetWindowMonitor(window, mon, 0, 0,
                                        mode->width, mode->height, mode->refreshRate);
                } else {
                    glfwSetWindowMonitor(window, nullptr,
                                        windowedX, windowedY, kWidth, kHeight, 0);
                }
                needsResize = true;
            }
            ImGui::Separator();

            ImGui::Text("Simulation");
            ImGui::SliderScalar("Speed (s/s)", ImGuiDataType_Double,
                                &simSpeedTarget,
                                (const double[]){1.0}, (const double[]){1e6},
                                "%.0f", ImGuiSliderFlags_Logarithmic);

            ImGui::Separator();
            ImGui::Text("Rendering");
            float exposure = renderer.exposure();
            if (ImGui::SliderFloat("Exposure", &exposure, 0.01f, 1000.0f,
                                   "%.2f", ImGuiSliderFlags_Logarithmic))
                renderer.setExposure(exposure);

            ImGui::Separator();
            ImGui::Text("Displacement");
            // displaceScale is in fractions of body radius — 0.006 ≈ 20 km on Mars.
            ImGui::SliderFloat("Displace scale", &bodyInfos[selectedBodyIndex].displaceScale,
                               0.0f, 0.02f, "%.4f");

            ImGui::Separator();
            ImGui::Text("Bloom");
            bool bloomEnabled = bloom.strength() > 0.0f;
            if (ImGui::Checkbox("Bloom enabled", &bloomEnabled))
                bloom.setStrength(bloomEnabled ? 0.3f : 0.0f);
            if (bloomEnabled) {
                float bloomStrength = bloom.strength();
                if (ImGui::SliderFloat("Strength",  &bloomStrength, 0.0f, 2.0f, "%.2f"))
                    bloom.setStrength(bloomStrength);
                float bloomThresh = bloom.threshold();
                if (ImGui::SliderFloat("Threshold", &bloomThresh,   0.1f, 5.0f, "%.2f"))
                    bloom.setThreshold(bloomThresh);
            }

            ImGui::Separator();
            ImGui::Text("Camera");
            ImGui::Text("  Frame    : %s", cfg.frame.c_str());
            ImGui::Text("  Focus    : %s", cfg.observerBody.c_str());
            ImGui::Text("  Distance : %.0f km  /  %.4f AU",
                        orbit.distanceKm, orbit.distanceKm / 149'597'870.7f);
            ImGui::Text("  Azimuth  : %.1f°", orbit.azimuthDeg);
            ImGui::Text("  Elevation: %.1f°", orbit.elevationDeg);

            ImGui::Separator();
            ImGui::Text("Bodies  (click to focus)");
            if (ImGui::BeginTable("bodies", 3,
                                  ImGuiTableFlags_Borders |
                                  ImGuiTableFlags_RowBg   |
                                  ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Dist from cam (km)");
                ImGui::TableSetupColumn("Dist from cam (AU)");
                ImGui::TableHeadersRow();
                for (int i = 0; i < static_cast<int>(bodyInfos.size()); ++i) {
                    auto& bi = bodyInfos[i];
                    glm::vec3 rp   = scene.origin().toRenderSpace(bi.node->worldPosition());
                    float     dist = glm::length(rp - camera.position());

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    bool selected = (i == selectedBodyIndex);
                    if (ImGui::Selectable(bi.node->naifName().c_str(), selected,
                                         ImGuiSelectableFlags_SpanAllColumns)) {
                        selectedBodyIndex = i;
                        orbit.distanceKm  = bi.radiusKm * 5.0f;
                        // Auto-exposure: normalise so a body at 1 AU looks like Earth.
                        // lightIntensity = 1/d², so exposure ∝ d².
                        if (sunIndex >= 0 && i != sunIndex) {
                            glm::vec3 bPos = scene.origin().toRenderSpace(
                                bi.node->worldPosition());
                            glm::vec3 sPos = scene.origin().toRenderSpace(
                                bodyInfos[sunIndex].node->worldPosition());
                            float dAU = glm::length(bPos - sPos) / 149'597'870.7f;
                            renderer.setExposure(std::clamp(dAU * dAU, 0.01f, 1000.0f));
                        }
                    }
                    ImGui::TableSetColumnIndex(1); ImGui::Text("%.0f", dist);
                    ImGui::TableSetColumnIndex(2); ImGui::Text("%.4f", dist / 149'597'870.7f);
                }
                ImGui::EndTable();
            }
            ImGui::End();
            }  // viewMode == Dev

            // ---- Dev: Spacecraft / thruster tuning panel ----
            if (viewMode == ViewMode::Dev) {
                ImGui::Begin("Spacecraft");

                ImGui::Text("Ship mass: %.0f kg", shipMass);
                ImGui::Separator();

                // Autopilot tuning.
                ImGui::Text("Autopilot");
                ImGui::TextDisabled("  (calibrated from thruster geometry at startup)");
                ImGui::SliderScalar("Max torque (N·m)", ImGuiDataType_Double,
                    &autopilot.maxTorqueNm,
                    (const double[]){1000.0}, (const double[]){500'000.0},
                    "%.0f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderScalar("RCS authority (N·m)", ImGuiDataType_Double,
                    &autopilot.rcsAuthorityNm,
                    (const double[]){100.0}, (const double[]){50'000.0},
                    "%.0f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderScalar("Kd att (s)", ImGuiDataType_Double,
                    &autopilot.KdAtt,
                    (const double[]){1.0}, (const double[]){60.0},
                    "%.1f");
                ImGui::SliderScalar("Att fire threshold (°)", ImGuiDataType_Double,
                    &autopilot.attFireThreshold,
                    (const double[]){0.001}, (const double[]){1.0},
                    "%.3f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderScalar("Rate fire threshold (°/s)", ImGuiDataType_Double,
                    &autopilot.rateFireThreshold,
                    (const double[]){0.0001}, (const double[]){0.1},
                    "%.4f", ImGuiSliderFlags_Logarithmic);
                ImGui::Separator();

                ImGui::Text("Main engine (SPACE)");
                ImGui::SliderScalar("Main thrust (N)", ImGuiDataType_Double,
                    &mainEngineThrust,
                    (const double[]){1'000.0}, (const double[]){2'000'000.0},
                    "%.0f", ImGuiSliderFlags_Logarithmic);
                if (!orionModel.thrusters.empty())
                    orionModel.thrusters[0].thrustN = static_cast<float>(mainEngineThrust);
                // Show resulting acceleration for quick sanity check.
                ImGui::TextDisabled("  → %.2f m/s²  (%.3f g)",
                    mainEngineThrust / shipMass,
                    mainEngineThrust / shipMass / 9.80665);

                ImGui::Separator();
                ImGui::Text("RCS (WASD/QE)");
                ImGui::SliderScalar("RCS thrust (N)", ImGuiDataType_Double,
                    &rcsThrust,
                    (const double[]){10.0}, (const double[]){50'000.0},
                    "%.0f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderScalar("Docking trans boost", ImGuiDataType_Double,
                    &dockingTransBoost,
                    (const double[]){1.0}, (const double[]){20.0},
                    "%.1f×");
                ImGui::TextDisabled("  Docking WASD = RCS thrust × boost");

                ImGui::Separator();
                ImGui::Text("RCS attitude (IJKL/UO)");
                ImGui::SliderScalar("RCS torque (N·m)", ImGuiDataType_Double,
                    &rcsTorque,
                    (const double[]){10.0}, (const double[]){100'000.0},
                    "%.0f", ImGuiSliderFlags_Logarithmic);

                ImGui::End();
            }

            // ---- Dev: Scenario save / load panel ----
            if (viewMode == ViewMode::Dev) {
                static const std::string kSavePath =
                    std::string(APEIRON_DATA_DIR) + "/saves/quicksave.toml";
                static std::string scenarioStatusMsg;

                ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_FirstUseEver);
                ImGui::Begin("Scenario");

                ImGui::TextDisabled("%s", kSavePath.c_str());
                ImGui::Spacing();

                if (ImGui::Button("Save scenario", ImVec2(-1, 0))) {
                    SimSaveData sd;
                    sd.simElapsed     = simElapsed;
                    sd.simSpeedTarget = simSpeedTarget;
                    sd.refBodyName    = refBody.name;
                    for (const auto& sc : spacecraft) {
                        SimSaveData::ScState st;
                        st.position        = sc->position();
                        st.velocity        = sc->velocity();
                        st.attitude        = sc->attitude();
                        st.angularVelocity = sc->angularVelocity();
                        sd.spacecraft.push_back(st);
                    }
                    sd.plan         = transferMFD.getPlan();
                    sd.cislunarPlan = cislunarMFD.getPlan();
                    // Docking: record hard-captured pair so restore can re-lock them.
                    if (dockingConstraint.phase() == DockingConstraint::Phase::HardCapture) {
                        auto* act = dockingConstraint.activeSc();
                        auto* pas = dockingConstraint.passiveSc();
                        int ai = -1, pi = -1;
                        for (int i = 0; i < static_cast<int>(spacecraft.size()); ++i) {
                            if (spacecraft[i].get() == act) ai = i;
                            if (spacecraft[i].get() == pas) pi = i;
                        }
                        if (ai >= 0 && pi >= 0) {
                            sd.docking.active         = true;
                            sd.docking.activeScIdx    = ai;
                            sd.docking.passiveScIdx   = pi;
                            sd.docking.activePortIdx  = 0;  // player always uses port 0
                            sd.docking.passivePortIdx = nav.dockPortIdx >= 0 ? nav.dockPortIdx : 0;
                        }
                    }
                    for (const auto& ev : obcEventQueue.events()) {
                        SimSaveData::SavedEvent se;
                        se.name        = ev.name;
                        se.eventET     = ev.eventET;
                        se.ann10min    = ev.announced10min;
                        se.ann5min     = ev.announced5min;
                        se.ann1min     = ev.announced1min;
                        se.ann10s      = ev.announced10s;
                        se.ann0        = ev.announced0;
                        se.countdown10s= ev.countdown10s;
                        sd.events.push_back(se);
                    }
                    scenarioStatusMsg = saveSimState(kSavePath, sd) ? "Saved." : "Save failed!";
                }

                ImGui::Spacing();
                if (ImGui::Button("Load scenario", ImVec2(-1, 0))) {
                    SimSaveData sd;
                    if (loadSimState(kSavePath, sd)) {
                        simElapsed     = sd.simElapsed;
                        simSpeedTarget = sd.simSpeedTarget;
                        const size_t n = std::min(sd.spacecraft.size(), spacecraft.size());
                        for (size_t i = 0; i < n; ++i) {
                            const auto& st = sd.spacecraft[i];
                            spacecraft[i]->setPosition(st.position);
                            spacecraft[i]->setVelocity(st.velocity);
                            spacecraft[i]->setAttitude(st.attitude);
                            spacecraft[i]->setAngularVelocity(st.angularVelocity);
                        }
                        // Restore docking before plan (plan restore may reconfigure nav).
                        if (sd.docking.active) {
                            const int ai = sd.docking.activeScIdx;
                            const int pi = sd.docking.passiveScIdx;
                            const int aPort = sd.docking.activePortIdx;
                            const int pPort = sd.docking.passivePortIdx;
                            const bool validIdx =
                                ai >= 0 && ai < static_cast<int>(spacecraft.size()) &&
                                pi >= 0 && pi < static_cast<int>(spacecraft.size()) &&
                                ai != pi &&
                                !scPorts[ai].empty() &&
                                pPort < static_cast<int>(scPorts[pi].size());
                            if (validIdx) {
                                dockingConstraint.arm(
                                    spacecraft[ai].get(), scPorts[ai][aPort],
                                    spacecraft[pi].get(), scPorts[pi][pPort]);
                                dockingConstraint.restoreHardCapture();
                                // Mirror into nav so the docking MFD shows the right target.
                                nav.dockTgtIdx  = pi;
                                nav.dockPortIdx = pPort;
                            }
                        }

                        if (sd.plan.valid)
                            transferMFD.restorePlan(sd.plan);
                        if (sd.cislunarPlan.valid)
                            cislunarMFD.restorePlan(sd.cislunarPlan);
                        {
                            std::vector<ScheduledEvent> restored;
                            for (const auto& se : sd.events) {
                                ScheduledEvent ev;
                                ev.name           = se.name;
                                ev.eventET        = se.eventET;
                                ev.announced10min = se.ann10min;
                                ev.announced5min  = se.ann5min;
                                ev.announced1min  = se.ann1min;
                                ev.announced10s   = se.ann10s;
                                ev.announced0     = se.ann0;
                                ev.countdown10s   = se.countdown10s;
                                restored.push_back(ev);
                            }
                            obcEventQueue.restoreEvents(std::move(restored));
                        }
                        // Restore the saved reference body so the OrbitalMFD shows
                        // the same body the user had selected at save time.
                        if (!sd.refBodyName.empty()) {
                            physicsWorld.setDominantBody(sd.refBodyName);
                            refBody = physicsWorld.refBody();
                            dominantBodyName = refBody.name;
                            orbitalMFD.setContext(refBody.name.c_str(), "");
                        }
                        scenarioStatusMsg = "Loaded.";
                    } else {
                        scenarioStatusMsg = "Load failed — no save file?";
                    }
                }

                if (!scenarioStatusMsg.empty()) {
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f), "%s", scenarioStatusMsg.c_str());
                }

                ImGui::End();
            }

            // ---- F11 Ship inspection view ----
            if (viewMode == ViewMode::ShipInspect) {
                ImGuiIO& io = ImGui::GetIO();

                const std::string& objName = (inspectIdx < static_cast<int>(kSpacecraftNames.size()))
                    ? kSpacecraftNames[inspectIdx]
                    : "Object " + std::to_string(inspectIdx);

                constexpr auto kFlags =
                    ImGuiWindowFlags_NoDecoration    |
                    ImGuiWindowFlags_NoSavedSettings |
                    ImGuiWindowFlags_AlwaysAutoResize|
                    ImGuiWindowFlags_NoFocusOnAppearing;

                ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.55f);
                ImGui::Begin("##ShipInspect", nullptr, kFlags);

                ImGui::TextColored({0.0f, 0.82f, 0.30f, 1.0f},
                    "SHIP INSPECT  [F11]  %s (%d/%d)",
                    objName.c_str(), inspectIdx + 1, static_cast<int>(spacecraft.size()));
                ImGui::Separator();

                ImGui::Text("Dist: %.1f m", shipOrbit.distanceM);
                ImGui::Text("Az:   %.1f°", shipOrbit.azimuthDeg);
                ImGui::Text("El:   %.1f°", shipOrbit.elevationDeg);
                ImGui::Separator();

                ImGui::Text("Orbit frame:");
                ImGui::SameLine();
                if (ImGui::RadioButton("Body", shipOrbit.bodyFrame))
                    shipOrbit.bodyFrame = true;
                ImGui::SameLine();
                if (ImGui::RadioButton("Inertial", !shipOrbit.bodyFrame))
                    shipOrbit.bodyFrame = false;

                ImGui::Separator();
                ImGui::TextDisabled("Drag to orbit  Scroll/pinch to zoom  Tab to cycle");

                // Show active thrusters.
                if (!orionModel.thrusters.empty()) {
                    ImGui::Separator();
                    ImGui::Text("Active thrusters:");
                    int activeCount = 0;
                    for (const auto& t : orionModel.thrusters) {
                        if (t.throttle >= 0.05f) {  // matches stepPWM minDutyCycle
                            ImGui::TextColored({1.0f, 0.5f, 0.1f, 1.0f},
                                "  %s  %.0f%%", t.id.c_str(), t.throttle * 100.0f);
                            ++activeCount;
                        }
                    }
                    if (activeCount == 0)
                        ImGui::TextDisabled("  (none)");
                }

                ImGui::End();

                // Sim speed indicator top-right.
                ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 10.0f, 10.0f),
                    ImGuiCond_Always, ImVec2(1.0f, 0.0f));
                ImGui::SetNextWindowBgAlpha(0.45f);
                ImGui::Begin("##SIclock", nullptr, kFlags);
                ImGui::TextColored({1.0f, 0.85f, 0.4f, 1.0f}, "%s UTC",
                                   currentEt.toISOUTCString(0).c_str());
                if (simSpeedTarget == 1.0)
                    ImGui::Text("1x real-time");
                else
                    ImGui::Text("%.0fx  [t / T]", simSpeedTarget);
                ImGui::End();
            }

            // ── Once-per-frame MFD context update ─────────────────────────────────
            // Builds MFDContext and pushes state to OrbitalMFD, MapMFD, TransferMFD.
            // Consolidated here so SPICE queries run once regardless of view mode,
            // and the pending-ref / pending-tgt input handling is not duplicated
            // across view branches.
            // Also declares shipRelRef at this scope so navHUD can read it.
            astro::PosState shipRelRef;
            {
                auto& ship = *spacecraft[playerIdx];

                // 1. Handle pending reference-body change typed by the user.
                {
                    std::string pending = orbitalMFD.consumePendingRef();
                    if (!pending.empty()) {
                        for (auto& c : pending)
                            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
                        if (const auto* gb = physicsWorld.findBody(pending)) {
                            // Update the display ref without changing the physics
                            // dominant body — allows inspecting any body's orbital
                            // elements even when outside its SOI (e.g. hyperbolic approach).
                            refBody = { gb->name, gb->gm, gb->radiusKm, gb->naifId };
                            userPinnedRef = true;
                            orbitalMFD.setContext(refBody.name.c_str(), "");
                        }
                    }
                }

                // 2. Build MFDContext.
                MFDContext mfdCtx;
                mfdCtx.currentEt       = currentEt;
                mfdCtx.simElapsed      = simElapsed;
                mfdCtx.refBodyName     = refBody.name;
                mfdCtx.refBodyMu       = refBody.mu;
                mfdCtx.refBodyRadiusKm = refBody.radiusKm;
                mfdCtx.shipGeoR        = ship.position();
                mfdCtx.shipGeoV        = ship.velocity();
                mfdCtx.mainThrustN     = mainEngineThrust;
                mfdCtx.shipMassKg      = shipMass;
                mfdCtx.eventQueue        = &obcEventQueue;
                mfdCtx.autopilot         = &autopilot;
                mfdCtx.solarSystemBodies = transferBodies;

                // Ship state relative to reference body.
                // Also compute the geocentric position of the reference body
                // so non-player spacecraft can be expressed in the same frame.
                glm::dvec3 refBodyGeoR(0.0), refBodyGeoV(0.0); // zero = ref IS Earth
                {
                    astro::PosState rel(ship.position(), ship.velocity());
                    if (refBody.naifId != 399) {
                        static const astro::ReferenceFrame kEclipRef =
                            astro::ReferenceFrame::createEclipJ2000();
                        astro::PosState refState;
                        try {
                            astro::Spice().getRelativeGeometricState(
                                static_cast<int>(refBody.naifId), 399,
                                currentEt, refState, kEclipRef);
                            refBodyGeoR = glm::dvec3(refState.r.x, refState.r.y, refState.r.z);
                            refBodyGeoV = glm::dvec3(refState.v.x, refState.v.y, refState.v.z);
                            rel = astro::PosState(
                                ship.position() - refBodyGeoR,
                                ship.velocity() - refBodyGeoV);
                        } catch (...) {}
                    }
                    mfdCtx.shipRelRef = rel;
                    shipRelRef        = rel;   // expose to navHUD below
                }

                // Non-player spacecraft states relative to the reference body.
                {
                    mfdCtx.actors.clear();
                    for (int ai = 0; ai < static_cast<int>(spacecraft.size()); ++ai) {
                        if (ai == playerIdx) continue;
                        ActorState a;
                        a.name      = vehicleConfigs[ai].name;
                        a.posRelRef = spacecraft[ai]->position() - refBodyGeoR;
                        a.velRelRef = spacecraft[ai]->velocity() - refBodyGeoV;
                        mfdCtx.actors.push_back(a);
                    }
                }

                // Body-fixed orientation, rotation rate, sun direction.
                {
                    const double etVal   = currentEt.getETValue();
                    const std::string iau = "IAU_" + refBody.name;
                    {
                        SpiceDouble m[3][3];
                        pxform_c("ECLIPJ2000", iau.c_str(), etVal, m);
                        if (!failed_c()) {
                            for (int ri = 0; ri < 3; ++ri)
                                for (int ci = 0; ci < 3; ++ci)
                                    mfdCtx.inertialToBody[ci][ri] = m[ri][ci];
                        }
                        reset_c();
                    }
                    {
                        SpiceInt n = 0;
                        SpiceDouble pm[3] = {0.0, 0.0, 0.0};
                        bodvrd_c(refBody.name.c_str(), "PM", 3, &n, pm);
                        if (!failed_c() && n >= 2)
                            mfdCtx.mapRotRateRadSec = pm[1] * (M_PI / 180.0) / 86400.0;
                        reset_c();
                    }
                    try {
                        static const astro::ReferenceFrame kEclipMap =
                            astro::ReferenceFrame::createEclipJ2000();
                        astro::PosState sunState;
                        astro::Spice().getRelativeGeometricState(
                            10, static_cast<int>(refBody.naifId), currentEt, sunState, kEclipMap);
                        glm::dvec3 sv(sunState.r.x, sunState.r.y, sunState.r.z);
                        double len = glm::length(sv);
                        if (len > 0.0) mfdCtx.sunDirInertial = sv / len;
                    } catch (...) {}
                }

                // Heliocentric ship state.
                {
                    static const astro::ReferenceFrame kEclipHelio =
                        astro::ReferenceFrame::createEclipJ2000();
                    astro::PosState earthHelio;
                    try {
                        astro::Spice().getRelativeGeometricState(399, 10, currentEt,
                                                                 earthHelio, kEclipHelio);
                        mfdCtx.shipHelioR =
                            glm::dvec3(earthHelio.r.x, earthHelio.r.y, earthHelio.r.z)
                            + ship.position();
                        mfdCtx.shipHelioV =
                            glm::dvec3(earthHelio.v.x, earthHelio.v.y, earthHelio.v.z)
                            + ship.velocity();
                    } catch (...) {}
                }

                // 3. Set MapMFD texture and ref name, then update all MFDs.
                mapMFD.setRefName(refBody.name.c_str());
                if (auto it = bodyDiffuseTexIds.find(refBody.name);
                    it != bodyDiffuseTexIds.end())
                    mapMFD.setMapTexture(it->second);

                orbitalMFD.update(mfdCtx);
                mapMFD.update(mfdCtx);
                transferMFD.update(mfdCtx);   // calls tickTcm + tickBplane internally
                cislunarMFD.update(mfdCtx);
                obcMFD.update(mfdCtx);

                // 4. Resolve pending TGT input.
                {
                    std::string pendingTgt = orbitalMFD.consumePendingTgt();
                    if (!pendingTgt.empty()) {
                        // Case-insensitive helper.
                        auto iequal = [](const std::string& a, const std::string& b) {
                            if (a.size() != b.size()) return false;
                            for (size_t k = 0; k < a.size(); ++k)
                                if (std::tolower((unsigned char)a[k])
                                 != std::tolower((unsigned char)b[k])) return false;
                            return true;
                        };

                        // Search non-player spacecraft; tgtIdx = position in that list.
                        bool foundSC = false;
                        int nonPlayerCount = 0;
                        for (int vi = 0; vi < static_cast<int>(spacecraft.size()); ++vi) {
                            if (vi == playerVehicleIdx) continue;
                            if (iequal(kSpacecraftNames[vi], pendingTgt)) {
                                orbitalMFD.setTgtIdx(nonPlayerCount);
                                foundSC = true;
                                break;
                            }
                            ++nonPlayerCount;
                        }
                        if (!foundSC) {
                            auto maybeId = astro::Spice().tryBodyNameToId(pendingTgt);
                            if (!maybeId) {
                                std::string up = pendingTgt;
                                for (auto& c : up)
                                    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
                                maybeId = astro::Spice().tryBodyNameToId(up);
                            }
                            if (maybeId) orbitalMFD.setTgtBody(*maybeId, pendingTgt);
                        }
                    }
                }

                // 5. Update orbital target state.
                // tgtNames was built by skipping playerVehicleIdx, so
                // targetIndex() i maps to the i-th non-player spacecraft.
                if (orbitalMFD.tgtBodyNaifId() >= 0) {
                    try {
                        static const astro::ReferenceFrame kEclipTgt =
                            astro::ReferenceFrame::createEclipJ2000();
                        astro::PosState bodyState;
                        astro::Spice().getRelativeGeometricState(
                            orbitalMFD.tgtBodyNaifId(),
                            static_cast<int>(refBody.naifId),
                            currentEt, bodyState, kEclipTgt);
                        orbitalMFD.updateTarget(bodyState, refBody.mu, refBody.radiusKm);
                    } catch (...) {
                        orbitalMFD.clearTarget();
                    }
                } else {
                    const int tgtIdx = orbitalMFD.targetIndex();
                    // Map tgtIdx (in the tgtNames list) to the actual spacecraft index.
                    int scIdx = -1, nonPlayerCount = 0;
                    for (int vi = 0; vi < static_cast<int>(spacecraft.size()); ++vi) {
                        if (vi == playerVehicleIdx) continue;
                        if (nonPlayerCount == tgtIdx) { scIdx = vi; break; }
                        ++nonPlayerCount;
                    }
                    if (scIdx >= 0) {
                        auto& tgt = *spacecraft[scIdx];
                        // State must be relative to the current reference body,
                        // not geocentric — use refBodyGeoR/V computed above.
                        orbitalMFD.updateTarget(
                            astro::PosState(tgt.position() - refBodyGeoR,
                                            tgt.velocity() - refBodyGeoV),
                            refBody.mu, refBody.radiusKm);
                    } else {
                        orbitalMFD.clearTarget();
                    }
                }

                // 6. Update DockingMFD (same for all views).
                dockingMFD.update(*spacecraft[playerIdx], scPorts, playerIdx, nav, spacecraft);
                {
                    using CP = DockingMFD::CapturePhase;
                    using DP = DockingConstraint::Phase;
                    CP cp = CP::None;
                    switch (dockingConstraint.phase()) {
                        case DP::Unarmed:     cp = CP::Unarmed;     break;
                        case DP::Armed:       cp = CP::Armed;       break;
                        case DP::SoftCapture: cp = CP::SoftCapture; break;
                        case DP::HardCapture: cp = CP::HardCapture; break;
                        default: break;
                    }
                    dockingMFD.setCapturePhase(cp);
                    const DockingConstraintParams& dp = DockingConstraintParams{};
                    dockingMFD.setCaptureThresholds(dp.maxRangeM, dp.maxClosureMs, dp.maxAttErrDeg);
                }
            }

            // ---- MFD fullscreen view (F2 / F3) ----
            if (viewMode == ViewMode::MfdFull || viewMode == ViewMode::MfdFull2) {
                ImGuiIO& io = ImGui::GetIO();
                const float W = io.DisplaySize.x;
                const float H = io.DisplaySize.y;

                if (viewMode == ViewMode::MfdFull) {
                    mfdFullPanel.pos  = { 0.0f, 0.0f };
                    mfdFullPanel.size = { W, H };
                    mfdFullPanel.render("##MFDFull");
                }
            }

            if (viewMode == ViewMode::MfdFull2) {
                ImGuiIO& io = ImGui::GetIO();
                const float W = io.DisplaySize.x;
                const float H = io.DisplaySize.y;
                mfdFullPanel2.pos  = { 0.0f, 0.0f };
                mfdFullPanel2.size = { W, H };
                mfdFullPanel2.render("##MFDFull2");
            }

            // ---- Nav view HUD ----
            if (viewMode == ViewMode::Nav) {
                ImGuiIO& io = ImGui::GetIO();
                const float W = io.DisplaySize.x;
                const float H = io.DisplaySize.y;

                constexpr auto kOverlayFlags =
                    ImGuiWindowFlags_NoDecoration    |
                    ImGuiWindowFlags_NoInputs        |
                    ImGuiWindowFlags_AlwaysAutoResize|
                    ImGuiWindowFlags_NoSavedSettings |
                    ImGuiWindowFlags_NoFocusOnAppearing;

                // ---- Top-right: mission clock + time acceleration ----
                ImGui::SetNextWindowPos(ImVec2(W - 10.0f, 10.0f),
                                        ImGuiCond_Always, ImVec2(1.0f, 0.0f));
                ImGui::SetNextWindowBgAlpha(0.45f);
                ImGui::Begin("##NavClock", nullptr, kOverlayFlags);
                ImGui::TextColored({1.0f, 0.85f, 0.4f, 1.0f}, "%s UTC",
                                   currentEt.toISOUTCString(0).c_str());
                if (simSpeedTarget == 1.0)
                    ImGui::Text("1x real-time");
                else
                    ImGui::Text("%.0fx  [t / T]", simSpeedTarget);
                if (!navCamNodes.empty())
                    ImGui::TextColored({0.5f, 0.9f, 0.5f, 0.8f}, "[C] %s",
                                       navCamNodes[navCamIdx].c_str());
                // Current nav mode indicator
                if (nav.navMode == NavMode::Orbit)
                    ImGui::TextColored({0.0f, 0.82f, 0.30f, 0.9f}, "ORBIT");
                else
                    ImGui::TextColored({0.2f, 0.8f, 1.0f, 0.9f}, "DOCKING");
                ImGui::End();

                // ---- MFD panels — flush to screen left/right edges ----

                // Each MFD occupies 1/3 of screen width; height derived from 16:9.
                const float kMfdW = std::round(W / 3.0f);
                const float kMfdH = std::round(kMfdW * (9.0f / 16.0f));
                const float mfdY  = H - kMfdH;

                mfdLeftPanel.pos  = { 0.0f, mfdY };
                mfdLeftPanel.size = { kMfdW, kMfdH };
                mfdLeftPanel.render("##MFD0");

                mfdRightPanel.pos  = { W - kMfdW, mfdY };
                mfdRightPanel.size = { kMfdW, kMfdH };
                mfdRightPanel.render("##MFD1");

                // ---- Nav Console — bottom centre ----
                {
                    navConsole.update(*spacecraft[playerIdx], shipForce, shipMass, frameDt);
                    navConsole.render(kMfdW, H, kMfdW, nav, autopilot,
                                      spacecraft, playerIdx, kSpacecraftNames, scPorts, mainEngineOn,
                                      activeNavDv);
                }

                // ---- Helmet HUD overlay ----
                navHUD.render(*spacecraft[playerIdx], spacecraft, nav,
                              refBodyWorld, scene, camera, W, H, kSpacecraftNames, scPorts,
                              refBody.name.c_str(), activeNavDv,
                              glm::dvec3(shipRelRef.v.x, shipRelRef.v.y, shipRelRef.v.z),
                              glm::dvec3(shipRelRef.r.x, shipRelRef.r.y, shipRelRef.r.z));
            }

            // ---- Atmosphere LUT inspector (Map view only) ----
            bool anyAtm = false;
            for (auto& ap : atmPrecompute) if (ap) { anyAtm = true; break; }
            if (anyAtm && viewMode == ViewMode::Dev) {
                ImGui::Begin("Atmosphere LUTs");
                for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
                    if (!atmPrecompute[i]) continue;
                    auto& bi = bodyInfos[i];
                    ImGui::Text("%s transmittance LUT (256×64)",
                                bi.node->naifName().c_str());
                    ImGui::Text("Rg=%.0f km  Ra=%.0f km  betaR_B=%.4e km⁻¹",
                                bi.radiusKm,
                                atmPrecompute[i]->params().atmosphereRadius,
                                atmPrecompute[i]->params().rayleigh.b);
                    // Display LUT (u=altitude, v=cosZenith).
                    // Width stretched to panel width; height proportional.
                    float panelW = ImGui::GetContentRegionAvail().x;
                    float lutH   = panelW * (64.0f / 256.0f);
                    if (atmLutImGuiSets[i] != VK_NULL_HANDLE) {
                        ImGui::Image(
                            reinterpret_cast<ImTextureID>(atmLutImGuiSets[i]),
                            ImVec2(panelW, lutH));
                        if (ImGui::IsItemHovered()) {
                            ImVec2 uv = ImGui::GetMousePos();
                            ImVec2 itemPos = ImGui::GetItemRectMin();
                            ImVec2 itemSz  = ImGui::GetItemRectSize();
                            float u = (uv.x - itemPos.x) / itemSz.x;
                            float v = (uv.y - itemPos.y) / itemSz.y;
                            ImGui::SetTooltip("alt=%.2f  cosZ=%.2f\n(u=%.3f  v=%.3f)",
                                              u, v * 2.0f - 1.0f, u, v);
                        }
                    }
                    ImGui::Separator();
                }
                ImGui::End();
            }

            ImGui::Render();

            // --- GPU frame ---
            const glm::mat4 vp = camera.viewProjection();

            // Rotation-only VP for stars (translation zeroed → stars at infinity).
            glm::mat4 viewRot = camera.viewMatrix();
            viewRot[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            const glm::mat4 vpRot = camera.projectionMatrix() * viewRot;

            // ---- Pre-compute model matrices for the offscreen cam pre-passes ----
            // These duplicate a small amount of work from the spacecraft draw section
            // below, but keep the main draw code unchanged.
            glm::vec3 offSunDir = (sunIndex >= 0)
                ? glm::normalize(sunRenderPos - scene.origin().toRenderSpace(refBodyWorld))
                : glm::vec3(0.0f, 1.0f, 0.0f);

            // Helper: build VP + camPos for a named cam node on the player ship.
            // Returns false (leaves vp/camPos unchanged) if the node is not found.
            glm::mat4 offOrion = glm::mat4(1.0f);
            {
                auto& sc  = *spacecraft[playerIdx];
                glm::vec3 rp = scene.origin().toRenderSpace(refBodyWorld + sc.position());
                glm::mat3 ar = glm::mat3_cast(glm::fquat(sc.attitude()));
                const glm::mat4 rf = glm::rotate(glm::mat4(1.0f),
                                                  glm::radians(90.0f),
                                                  glm::vec3(1.0f, 0.0f, 0.0f));
                offOrion = glm::translate(glm::mat4(1.0f), rp)
                         * glm::mat4(ar) * rf
                         * glm::scale(glm::mat4(1.0f), glm::vec3(1e-3f));
            }
            std::vector<glm::mat4> offAi(aiGltfs.size(), glm::mat4(1.0f));
            for (size_t ai = 0; ai < aiGltfs.size() && ai + 1 < spacecraft.size(); ++ai) {
                if (!aiGltfs[ai]->isLoaded()) continue;
                auto& sc  = *spacecraft[ai + 1];
                glm::vec3 rp = scene.origin().toRenderSpace(refBodyWorld + sc.position());
                glm::mat3 ar = glm::mat3_cast(glm::fquat(sc.attitude()));
                offAi[ai] = glm::translate(glm::mat4(1.0f), rp)
                           * glm::mat4(ar)
                           * glm::scale(glm::mat4(1.0f), glm::vec3(1e-3f));
            }

            // Build VP for a named cam node on the player ship (square 1:1 aspect).
            // vpRotOut (optional): rotation-only VP for star rendering (no translation).
            auto buildCamVP = [&](const std::string& nodeName, float fovDeg,
                                  glm::mat4& vpOut, glm::vec3& camPosOut,
                                  glm::mat4* vpRotOut = nullptr,
                                  float slewYawDeg = 0.0f, float slewPitchDeg = 0.0f) {
                if (!orionGltf.isLoaded() || nodeName.empty()) return;
                auto& sc  = *spacecraft[playerIdx];
                glm::vec3 rp = scene.origin().toRenderSpace(refBodyWorld + sc.position());
                glm::mat3 ar = glm::mat3_cast(glm::fquat(sc.attitude()));
                const glm::mat4 rf = glm::rotate(glm::mat4(1.0f),
                                                  glm::radians(90.0f),
                                                  glm::vec3(1.0f, 0.0f, 0.0f));
                glm::mat4 nodeTf   = orionGltf.nodeWorldTransform(nodeName);
                glm::mat4 camWorld = glm::mat4(ar) * rf * nodeTf;
                glm::vec3 pos = rp + glm::vec3(camWorld[3]) * 1e-3f;
                glm::vec3 fwd = glm::normalize(glm::vec3(camWorld[0]));
                glm::vec3 up  = glm::normalize(glm::vec3(camWorld[1]));
                // Apply slew offsets: yaw rotates around the camera's up axis,
                // pitch rotates around the camera's right axis.
                if (slewYawDeg != 0.0f || slewPitchDeg != 0.0f) {
                    glm::mat4 yawMat = glm::rotate(glm::mat4(1.0f),
                                                   glm::radians(slewYawDeg), up);
                    fwd = glm::normalize(glm::vec3(yawMat * glm::vec4(fwd, 0.0f)));
                    glm::vec3 right = glm::normalize(glm::cross(up, fwd));
                    glm::mat4 pitchMat = glm::rotate(glm::mat4(1.0f),
                                                     -glm::radians(slewPitchDeg), right);
                    fwd = glm::normalize(glm::vec3(pitchMat * glm::vec4(fwd, 0.0f)));
                    up  = glm::normalize(glm::vec3(pitchMat * glm::vec4(up,  0.0f)));
                }
                camPosOut = pos;
                // near: 0.1 m (1e-4 km) for close spacecraft views.
                // far: 1e9 km — must match C_FAR in the planet/mesh shaders which use
                // logarithmic depth; the projection far plane is just the hardware clip limit.
                glm::mat4 proj = glm::perspective(glm::radians(fovDeg), 1.0f, 1e-4f, 1.0e9f);
                proj[1][1] *= -1.0f;
                glm::mat4 view = glm::lookAt(pos, pos + fwd, up);
                vpOut = proj * view;
                if (vpRotOut) {
                    glm::mat4 viewRot = view;
                    viewRot[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
                    *vpRotOut = proj * viewRot;
                }
            };

            glm::mat4 offVP      = glm::mat4(1.0f);
            glm::mat4 offVPRot   = glm::mat4(1.0f);
            glm::vec3 offCamPos  = glm::vec3(0.0f);
            glm::mat4 dockVP     = glm::mat4(1.0f);
            glm::mat4 dockVPRot  = glm::mat4(1.0f);
            glm::vec3 dockCamPos = glm::vec3(0.0f);
            buildCamVP(camMFD.activeCamNode(), camMFD.fovDeg(), offVP, offCamPos, &offVPRot,
                       camMFD.slewYaw(), camMFD.slewPitch());
            buildCamVP(dockingMFD.preferredCamNode(), dockingMFD.fovDeg(),    dockVP, dockCamPos, &dockVPRot);

            if (!renderer.acquireFrame()) continue;

            // ---- Offscreen cam pre-passes (before the main HDR pass) ----
            {
                const int fi = renderer.currentFrameIndex();

                // Determine which offscreen passes are actually needed this frame.
                // Dev and ShipInspect views show no MFD panels — skip both passes.
                auto appOnScreen = [&](MFDApp* app) -> bool {
                    if (viewMode == ViewMode::MfdFull)   return mfdFullPanel.app  == app;
                    if (viewMode == ViewMode::MfdFull2)  return mfdFullPanel2.app == app;
                    if (viewMode == ViewMode::Nav)
                        return mfdLeftPanel.app == app || mfdRightPanel.app == app;
                    return false;
                };
                const bool needCamPass     = appOnScreen(&camMFD);
                const bool needDockingPass = appOnScreen(&dockingMFD);

                // CamMFD offscreen — only when the CAM panel is on screen.
                if (needCamPass) {
                offscreenCam.begin(renderer.currentCmd(), fi);

                // Stars (drawn first — depth write disabled, stars at infinity).
                starField.draw(renderer.currentCmd(), offVPRot);

                // Celestial bodies — same model/lighting logic as the main HDR pass
                // but using the offscreen VP and camera position.
                // Rings and atmospheres are skipped (nice-to-have, add later).
                for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
                    auto& bi = bodyInfos[i];
                    glm::vec3 renderPos = scene.origin().toRenderSpace(
                        bi.node->worldPosition());
                    float distKm = glm::length(renderPos - offCamPos);
                    float offApparentPx = (bi.radiusKm / distKm)
                        * (static_cast<float>(OffscreenCam::kHeight)
                           / std::tan(glm::radians(70.0f) * 0.5f));
                    float sizeScale = (offApparentPx < 2.0f)
                                    ? (2.0f / offApparentPx) : 1.0f;
                    glm::mat4 model = glm::translate(glm::mat4(1.0f), renderPos);
                    model = model * glm::mat4(bi.node->orientation());
                    glm::vec3 bodyScale = bi.meshPath.empty()
                        ? glm::vec3(bi.radiusKm, bi.bRadiusKm, bi.polarRadiusKm)
                        : glm::vec3(bi.radiusKm);
                    model = glm::scale(model, bodyScale * sizeScale);
                    bool isEmissive = (sunIndex >= 0 && static_cast<int>(i) == sunIndex);
                    glm::vec3 sunDir      = glm::vec3(0.0f, 1.0f, 0.0f);
                    float     lightIntens = 1.0f;
                    if (sunIndex >= 0 && !isEmissive) {
                        sunDir = glm::normalize(sunRenderPos - renderPos);
                        float distAU = glm::length(sunRenderPos - renderPos) / 149'597'870.7f;
                        lightIntens  = 1.0f / (distAU * distAU);
                    }
                    glm::vec3 viewDir = glm::normalize(offCamPos - renderPos);
                    float dispScale = (sizeScale > 1.0f) ? 0.0f : bi.displaceScale;
                    renderer.draw(offVP * model, model, sunDir, viewDir,
                                  isEmissive, lightIntens, dispScale,
                                  descriptorSets[i], *meshes[i]);
                }

                // Spacecraft models drawn last (in front of background).
                if (orionGltf.isLoaded())
                    orionGltf.draw(renderer.currentCmd(), meshPipeline,
                                   offVP, offOrion, offSunDir, offCamPos);
                for (size_t ai = 0; ai < aiGltfs.size() && ai + 1 < spacecraft.size(); ++ai)
                    if (aiGltfs[ai]->isLoaded())
                        aiGltfs[ai]->draw(renderer.currentCmd(), meshPipeline,
                                          offVP, offAi[ai], offSunDir, offCamPos);
                offscreenCam.end(renderer.currentCmd());
                camMFD.setTexture(offscreenCam.imguiTexture(fi));
                } // needCamPass

                // DockingMFD offscreen — only when the DOCK panel is on screen.
                if (needDockingPass) {
                dockingOffscreenCam.begin(renderer.currentCmd(), fi);

                starField.draw(renderer.currentCmd(), dockVPRot);

                for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
                    auto& bi = bodyInfos[i];
                    glm::vec3 renderPos = scene.origin().toRenderSpace(
                        bi.node->worldPosition());
                    float distKm = glm::length(renderPos - dockCamPos);
                    float dockApparentPx = (bi.radiusKm / distKm)
                        * (static_cast<float>(OffscreenCam::kHeight)
                           / std::tan(glm::radians(dockingMFD.fovDeg()) * 0.5f));
                    float sizeScale = (dockApparentPx < 2.0f)
                                    ? (2.0f / dockApparentPx) : 1.0f;
                    glm::mat4 model = glm::translate(glm::mat4(1.0f), renderPos);
                    model = model * glm::mat4(bi.node->orientation());
                    glm::vec3 bodyScale = bi.meshPath.empty()
                        ? glm::vec3(bi.radiusKm, bi.bRadiusKm, bi.polarRadiusKm)
                        : glm::vec3(bi.radiusKm);
                    model = glm::scale(model, bodyScale * sizeScale);
                    bool isEmissive = (sunIndex >= 0 && static_cast<int>(i) == sunIndex);
                    glm::vec3 sunDir      = glm::vec3(0.0f, 1.0f, 0.0f);
                    float     lightIntens = 1.0f;
                    if (sunIndex >= 0 && !isEmissive) {
                        sunDir = glm::normalize(sunRenderPos - renderPos);
                        float distAU = glm::length(sunRenderPos - renderPos) / 149'597'870.7f;
                        lightIntens  = 1.0f / (distAU * distAU);
                    }
                    glm::vec3 viewDir = glm::normalize(dockCamPos - renderPos);
                    float dispScale = (sizeScale > 1.0f) ? 0.0f : bi.displaceScale;
                    renderer.draw(dockVP * model, model, sunDir, viewDir,
                                  isEmissive, lightIntens, dispScale,
                                  descriptorSets[i], *meshes[i]);
                }

                if (orionGltf.isLoaded())
                    orionGltf.draw(renderer.currentCmd(), meshPipeline,
                                   dockVP, offOrion, offSunDir, dockCamPos);
                for (size_t ai = 0; ai < aiGltfs.size() && ai + 1 < spacecraft.size(); ++ai)
                    if (aiGltfs[ai]->isLoaded())
                        aiGltfs[ai]->draw(renderer.currentCmd(), meshPipeline,
                                          dockVP, offAi[ai], offSunDir, dockCamPos);
                dockingOffscreenCam.end(renderer.currentCmd());
                dockingMFD.setTexture(dockingOffscreenCam.imguiTexture(fi));
                } // needDockingPass
            }

            renderer.beginHDRPass();

            starField.draw(renderer.currentCmd(), vpRot);

            for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
                auto& bi = bodyInfos[i];

                glm::vec3 renderPos = scene.origin().toRenderSpace(
                    bi.node->worldPosition());

                // T * R * S: translate to body position, rotate by IAU orientation,
                // then scale to physical radius.
                // Screen-space minimum size: clamp so the body always covers at least
                // 2 pixels in diameter, preventing sub-pixel flickering for distant moons.
                float distKm = glm::length(renderPos - camera.position());
                float fovYRad = glm::radians(45.0f);
                float apparentPixels = (bi.radiusKm / distKm)
                                     * (static_cast<float>(swapchain.extent().height)
                                        / std::tan(fovYRad * 0.5f));
                float sizeScale = (apparentPixels < 2.0f) ? (2.0f / apparentPixels) : 1.0f;

                glm::mat4 model = glm::translate(glm::mat4(1.0f), renderPos);
                model = model * glm::mat4(bi.node->orientation());
                // OBJ meshes already encode the true shape; scale uniformly by the
                // a-axis radius.  Procedural spheres need per-axis scaling for oblate bodies.
                glm::vec3 bodyScale = bi.meshPath.empty()
                    ? glm::vec3(bi.radiusKm, bi.bRadiusKm, bi.polarRadiusKm)
                    : glm::vec3(bi.radiusKm);
                model = glm::scale(model, bodyScale * sizeScale);

                bool isEmissive = (sunIndex >= 0 && static_cast<int>(i) == sunIndex);

                // Per-body sun direction and 1/d² light intensity.
                glm::vec3 sunDir       = glm::vec3(0.0f, 1.0f, 0.0f);
                float     lightIntensity = 1.0f;
                if (sunIndex >= 0 && !isEmissive) {
                    sunDir = glm::normalize(sunRenderPos - renderPos);
                    float distAU = glm::length(sunRenderPos - renderPos) / 149'597'870.7f;
                    lightIntensity = 1.0f / (distAU * distAU);
                }

                glm::vec3 viewDir = glm::normalize(camera.position() - renderPos);
                // Disable displacement when the size-clamping scale is active
                // (body is sub-pixel) to avoid amplifying the effect.
                float dispScale = (sizeScale > 1.0f) ? 0.0f : bi.displaceScale;
                renderer.draw(vp * model, model, sunDir, viewDir,
                              isEmissive, lightIntensity, dispScale,
                              descriptorSets[i], *meshes[i]);

                // Draw ring after the planet so depth occlusion is correct.
                if (ringMeshes[i]) {
                    glm::mat4 ringModel = glm::translate(glm::mat4(1.0f), renderPos);
                    ringModel = ringModel * glm::mat4(bi.node->orientation());
                    ringModel = glm::scale(ringModel, glm::vec3(bi.radiusKm * sizeScale));
                    renderer.drawRing(vp * ringModel, ringModel, sunDir, viewDir,
                                      lightIntensity, ringDescSets[i], *ringMeshes[i]);
                }

                // Draw atmosphere shell after the planet body (depth test culls occluded
                // fragments).  Skip when the size-clamping scale is active; atmosphere
                // would be grossly wrong at sub-pixel distances anyway.
                if (atmPrecompute[i] && sizeScale <= 1.0f) {
                    auto& ap  = *atmPrecompute[i];
                    float Ra  = ap.params().atmosphereRadius;   // km

                    // Build the atmosphere model matrix (scaled to Ra, not bi.radiusKm).
                    glm::mat4 atmModel = glm::translate(glm::mat4(1.0f), renderPos);
                    atmModel = atmModel * glm::mat4(bi.node->orientation());
                    atmModel = glm::scale(atmModel, glm::vec3(Ra));

                    // Transform camera and sun direction to local space (atm sphere = radius 1).
                    glm::mat4 invAtm  = glm::inverse(atmModel);
                    glm::vec3 camLocal = glm::vec3(invAtm * glm::vec4(camera.position(), 1.0f));
                    glm::vec3 sunLocal = glm::normalize(
                        glm::vec3(invAtm * glm::vec4(sunDir, 0.0f)));

                    // Convert Bruneton km parameters to local units (per unit = per Ra km).
                    auto& a = ap.params();
                    glm::vec3 betaR = a.rayleigh * Ra;
                    float scaleHR   = a.rayleighScaleH / Ra;
                    float betaM     = a.mieScattering  * Ra;
                    float betaMext  = a.mieExtinction  * Ra;
                    float scaleHM   = a.mieScaleH      / Ra;
                    float groundRatio = bi.radiusKm / Ra;

                    renderer.drawAtmosphere(
                        vp * atmModel,
                        camLocal, groundRatio,
                        sunLocal, lightIntensity,
                        betaR, scaleHR,
                        betaM, betaMext, scaleHM, a.mieG,
                        atmDescSets[i], atmShellMesh);
                }
            }

            // ---- Draw spacecraft ----
            // Positions are observer-body-centred ECI (km); convert via refBodyWorld offset.
            {
                glm::vec3 scSunDir = (sunIndex >= 0)
                    ? glm::normalize(sunRenderPos - scene.origin().toRenderSpace(refBodyWorld))
                    : glm::vec3(0.0f, 1.0f, 0.0f);

                // Helper: draw a fallback emissive sphere for any spacecraft.
                auto drawFallback = [&](const glm::vec3& rp, const glm::mat3& attRot) {
                    constexpr float kShipRadiusKm = 0.01f;
                    glm::mat4 model = glm::translate(glm::mat4(1.0f), rp)
                                    * glm::mat4(attRot)
                                    * glm::scale(glm::mat4(1.0f), glm::vec3(kShipRadiusKm));
                    glm::vec3 viewDir = glm::normalize(camera.position() - rp);
                    renderer.draw(vp * model, model, scSunDir, viewDir,
                                  /*emissive=*/true, 1.0f, 0.0f,
                                  descriptorSets[0], *meshes[0]);
                };

                // ---- Orion (player) ----
                {
                    auto& sc = *spacecraft[playerIdx];
                    glm::dvec3 worldPos = refBodyWorld + sc.position();
                    glm::vec3  rp       = scene.origin().toRenderSpace(worldPos);
                    glm::mat3  attRot   = glm::mat3_cast(glm::fquat(sc.attitude()));
                    constexpr float kModelToKm = 1e-3f;
                    glm::mat4 rollFix = glm::rotate(glm::mat4(1.0f),
                                                    glm::radians(90.0f),
                                                    glm::vec3(1.0f, 0.0f, 0.0f));
                    glm::mat4 shipModel = glm::translate(glm::mat4(1.0f), rp)
                                       * glm::mat4(attRot)
                                       * rollFix
                                       * glm::scale(glm::mat4(1.0f), glm::vec3(kModelToKm));
                    if (orionGltf.isLoaded()) {
                        if (!orionModel.thrusters.empty()) {
                            for (const auto& t : orionModel.thrusters) {
                                if (t.exhaustNode.empty()) continue;
                                orionGltf.setNodeVisible(t.exhaustNode, t.firing);
                                orionGltf.setNodeColor(t.exhaustNode,
                                                       t.plumeColor, t.plumeIntensity);
                            }
                        }
                        orionGltf.draw(renderer.currentCmd(), meshPipeline,
                                       vp, shipModel, scSunDir, camera.position());
                    } else {
                        drawFallback(rp, attRot);
                    }
                }

                // ---- AI craft ----
                for (size_t ai = 0; ai < aiGltfs.size() && ai + 1 < spacecraft.size(); ++ai) {
                    auto& sc = *spacecraft[ai + 1];
                    glm::dvec3 worldPos = refBodyWorld + sc.position();
                    glm::vec3  rp       = scene.origin().toRenderSpace(worldPos);
                    glm::mat3  attRot   = glm::mat3_cast(glm::fquat(sc.attitude()));
                    constexpr float kScToKm = 1e-3f;
                    glm::mat4 scModel = glm::translate(glm::mat4(1.0f), rp)
                                      * glm::mat4(attRot)
                                      * glm::scale(glm::mat4(1.0f), glm::vec3(kScToKm));
                    if (aiGltfs[ai]->isLoaded()) {
                        aiGltfs[ai]->draw(renderer.currentCmd(), meshPipeline,
                                          vp, scModel, scSunDir, camera.position());
                    } else {
                        drawFallback(rp, attRot);
                    }
                }
            }

            renderer.endFrame();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        obc::shutdown();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    obc::shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
