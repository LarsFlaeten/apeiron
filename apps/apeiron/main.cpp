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
#include "ScenarioConfig.h"
#include "Spacecraft.h"
#include "MFD.h"
#include "OrbitalMFD.h"
#include "apeiron/spacecraft/ManifestLoader.h"
#include "apeiron/spacecraft/SpacecraftModel.h"

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

int main()
{
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

    try {
        // =================================================================
        // Universe — no GPU resources involved
        // =================================================================
        auto cfg = ScenarioConfig::load(APEIRON_SCENARIO_FILE);

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

        // Earth radius for camera setup.
        float earthRadius = 6371.0f;
        for (auto& bi : bodyInfos)
            if (bi.node->naifName() == "EARTH") { earthRadius = bi.radiusKm; break; }

        // =================================================================
        // Spacecraft — physics only, no GPU resources
        // Coordinate system: Earth-centred ECLIPJ2000 (km, km/s).
        // =================================================================

        // Earth's GM (km³/s²) and attractor at origin of ECI frame.
        constexpr double kGM_Earth = 398600.4418;
        astro::Attractor earthAttractor{ glm::dvec3(0.0), kGM_Earth };

        // Circular LEO at 400 km altitude (r = earthRadius + 400 km).
        const double orbitRadius = static_cast<double>(earthRadius) + 400.0;
        const double circularV   = std::sqrt(kGM_Earth / orbitRadius);

        // Player ship: starts at +X, velocity along +Y (prograde, ecliptic plane).
        // Mass ~12 500 kg (Crew Dragon placeholder), inertia rough sphere estimate.
        const double shipMass = 12519.0;
        const double shipI    = 0.4 * shipMass * 4.5 * 4.5;  // kg·m²  (solid sphere, r=4.5 m)
        astro::State shipState;
        shipState.P.r = glm::dvec3(orbitRadius, 0.0, 0.0);
        shipState.P.v = glm::dvec3(0.0, circularV, 0.0);
        // Align body frame to RTN: +X=prograde, +Y=nadir, +Z=orbit-normal.
        {
            glm::dvec3 T = glm::normalize(shipState.P.v);                        // prograde
            glm::dvec3 N = glm::normalize(glm::cross(shipState.P.r, shipState.P.v)); // orbit normal
            glm::dvec3 R = glm::cross(T, N);                                     // radial outward
            // Rotation matrix columns = where body axes land in inertial frame.
            shipState.R.q = glm::quat_cast(glm::dmat3(T, -R, N));
        }
        shipState.R.w = glm::dvec3(0.0);

        std::vector<std::unique_ptr<Spacecraft>> spacecraft;
        spacecraft.push_back(std::make_unique<Spacecraft>(shipMass, glm::dmat3(shipI), shipState));
        spacecraft[0]->addAttractor(earthAttractor);

        // Index of the spacecraft the player controls / camera follows.
        const size_t playerIdx = 0;

        // Load Orion thruster manifest.  Falls back gracefully if the file is missing.
        spacecraft::SpacecraftModel orionModel;
        {
            std::filesystem::path toml =
                std::filesystem::path(APEIRON_DATA_DIR)
                / "spacecraft/orion/CM_SM06_thrusters.toml";
            if (std::filesystem::exists(toml)) {
                orionModel = spacecraft::loadManifest(toml);
                std::cout << "[Apeiron] Loaded Orion manifest: "
                          << orionModel.thrusters.size() << " thrusters\n";
            } else {
                std::cerr << "[Apeiron] WARNING: Orion manifest not found at "
                          << toml << " — using scalar fallback\n";
            }
        }

        // MFD apps — updated and rendered every frame in Nav view.
        OrbitalMFD orbitalMFD;
        orbitalMFD.setContext("EARTH", "ECLIPJ2000");

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

        // Reference body for the OrbitalMFD — updated when the user types a new NAIF name.
        struct RefBody {
            std::string name;
            double      mu;        // km³/s²
            double      radiusKm;  // equatorial
            SpiceInt    naifId;    // NAIF body id
        };
        RefBody refBody{ "EARTH", kGM_Earth,
                         static_cast<double>(earthRadius), 399 };

        // Thruster parameters — adjustable from the Dev view.
        double mainEngineThrust = 22'000.0;   // N  (SPACE)
        double rcsThrust        =    400.0;   // N  (WASD/QE)
        double rcsTorque        =  1'000.0;   // N·m (IJKL/UO)

        // Physics fixed-step accumulator (100 Hz simulation).
        constexpr double kPhysStep   = 0.01;  // seconds
        double           physAccum   = 0.0;

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

        // Load Orion glTF model.
        apeiron::render::GltfModel orionGltf;
        {
            std::filesystem::path glb =
                std::filesystem::path(APEIRON_DATA_DIR)
                / "spacecraft/orion/CM_SM06.glb";
            if (std::filesystem::exists(glb)) {
                glfwSetWindowTitle(window, "Apeiron — Loading Orion model…");
                glfwPollEvents();
                orionGltf.load(allocator, glb);
                std::cout << "[Apeiron] Loaded Orion glTF: "
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
                std::cerr << "[Apeiron] WARNING: Orion GLB not found at " << glb << "\n";
            }
        }

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

        // View mode: F1=Nav, F2=MFD fullscreen, F11=Ship inspect, F12=Dev.
        enum class ViewMode { Dev, Nav, MfdFull, ShipInspect };
        ViewMode viewMode = ViewMode::Dev;
        double simSpeedTarget          = 1.0; // set instantly by t/T keys
        double simSecondsPerRealSecond = 1.0; // smoothly tracks target (log-space)

        // Window state shared between callbacks.
        struct WindowState {
            apeiron::render::Renderer* renderer;
            double     scrollDelta = 0.0;
            ViewMode*  viewMode    = nullptr;
            double*    simSpeed    = nullptr;
            std::vector<std::string>* navCams    = nullptr;
            int*                      navCamIdx  = nullptr;
        } windowState{&renderer, 0.0, &viewMode, &simSpeedTarget,
                      &navCamNodes, &navCamIdx};

        bool isFullscreen = false;
        bool needsResize  = false;
        int  windowedX = 100, windowedY = 100;

        glfwSetWindowUserPointer(window, &windowState);
        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int mods) {
            if (action != GLFW_PRESS) return;
            auto* s = static_cast<WindowState*>(glfwGetWindowUserPointer(w));
            if (key == GLFW_KEY_ESCAPE)
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            else if (key == GLFW_KEY_GRAVE_ACCENT)
                s->renderer->setWireframe(!s->renderer->wireframe());
            else if (key == GLFW_KEY_F1)
                *s->viewMode = ViewMode::Nav;
            else if (key == GLFW_KEY_F2)
                *s->viewMode = ViewMode::MfdFull;
            else if (key == GLFW_KEY_F11)
                *s->viewMode = ViewMode::ShipInspect;
            else if (key == GLFW_KEY_C &&
                     (*s->viewMode == ViewMode::Nav ||
                      *s->viewMode == ViewMode::MfdFull) &&
                     !s->navCams->empty())
                *s->navCamIdx = (*s->navCamIdx + 1) % static_cast<int>(s->navCams->size());
            else if (key == GLFW_KEY_F12)
                *s->viewMode = ViewMode::Dev;
            else if (key == GLFW_KEY_T) {
                if (mods & GLFW_MOD_SHIFT)
                    *s->simSpeed = std::max(1.0,   *s->simSpeed / 10.0);  // T = slower
                else
                    *s->simSpeed = std::min(1.0e6, *s->simSpeed * 10.0);  // t = faster
            }
        });
        glfwSetScrollCallback(window, [](GLFWwindow* w, double, double yoff) {
            auto* s = static_cast<WindowState*>(glfwGetWindowUserPointer(w));
            s->scrollDelta += yoff;
        });

        // =================================================================
        // Main loop
        // =================================================================
        double simElapsed = 0.0; // accumulated simulation seconds

        auto prevFrameTime = std::chrono::steady_clock::now();
        double prevMouseX{}, prevMouseY{};
        glfwGetCursorPos(window, &prevMouseX, &prevMouseY);

        // For finite-differencing angular and linear rates in the Nav console.
        glm::dvec3 prevAngVelInertial(0.0);
        glm::dvec3 prevLinVelKms(0.0);
        float      prevNavDt = 0.016f;

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

            // --- Orbit camera input (ignored when ImGui is using the mouse) ---
            double mx{}, my{};
            glfwGetCursorPos(window, &mx, &my);
            float dx = static_cast<float>(mx - prevMouseX);
            float dy = static_cast<float>(my - prevMouseY);
            prevMouseX = mx; prevMouseY = my;

            // Cache Earth world position (used for both camera modes and rendering).
            glm::dvec3 earthWorld(0.0);
            for (auto& bi : bodyInfos)
                if (bi.node->naifName() == "EARTH") { earthWorld = bi.node->worldPosition(); break; }

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

            // Smooth time acceleration: converge in log-space toward target.
            // Time constant ~0.15 s real time — fast enough to feel responsive,
            // slow enough to avoid the one-frame orbital skip on t/T presses.
            {
                const double logCurr = std::log(simSecondsPerRealSecond);
                const double logTarg = std::log(simSpeedTarget);
                const double logNew  = logCurr + (logTarg - logCurr)
                                       * std::min(1.0, static_cast<double>(frameDt) / 0.15);
                simSecondsPerRealSecond = std::exp(logNew);
            }

            simElapsed += frameDt * simSecondsPerRealSecond;
            auto currentEt = et + astro::TimeDelta(simElapsed);

            // ---- Thruster input (active in Nav view at 1x only) ----
            // Controls are disabled at time acceleration > 1x — thrust at 1000x makes no sense.
            bool mainEngineOn = false;
            {
                glm::dvec3 shipForce(0.0), shipTorque(0.0);

                if ((viewMode == ViewMode::Nav || viewMode == ViewMode::ShipInspect)
                    && simSpeedTarget <= 1.0
                    && !ImGui::GetIO().WantCaptureKeyboard) {

                    if (!orionModel.thrusters.empty()) {
                        // ---- Control allocation path (Orion manifest loaded) ----
                        // Orion model space: +X = aft, engine pointing +X.
                        spacecraft::Wrench desired{};
                        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
                            desired[0] += static_cast<double>(orionModel.thrusters[0].thrustN);
                            mainEngineOn = true;
                        }
                        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) desired[0] += rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) desired[0] -= rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) desired[1] += rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) desired[1] -= rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) desired[2] += rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) desired[2] -= rcsThrust;
                        if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) desired[4] += rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) desired[4] -= rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) desired[5] += rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) desired[5] -= rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) desired[3] -= rcsTorque;
                        if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) desired[3] += rcsTorque;
                        orionModel.solveAllocation(desired);
                        orionModel.stepPWM(frameDt);
                        glm::vec3 F{}, T{};
                        orionModel.accumulateWrench(F, T);
                        shipForce  = glm::dvec3(F);
                        shipTorque = glm::dvec3(T);
                    } else {
                        // ---- Scalar fallback (no manifest) ----
                        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
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
            // Accumulator runs in sim-seconds so the orbit scales with time acceleration.
            // Step size grows at high time accel to keep steps/frame ≤ kMaxStepsPerFrame.
            // No render interpolation: from a first-person bow camera at orbital altitude
            // one physics step (< 100 m) subtends < 0.001° — completely invisible.
            {
                constexpr int kMaxStepsPerFrame = 100;
                const double  simDt = static_cast<double>(frameDt) * simSecondsPerRealSecond;
                const double  step  = std::max(kPhysStep, simDt / kMaxStepsPerFrame);

                physAccum += simDt;
                while (physAccum >= step) {
                    for (auto& sc : spacecraft)
                        if (!sc->parentId())
                            sc->update(step, currentEt);
                    physAccum -= step;
                }
            }

            observerPos = observer.worldPosition(currentEt);
            scene.update(currentEt, observerPos);

            // Recenter the floating origin.
            // In Nav view, track the player ship; in Map view, track the selected body.
            if (viewMode == ViewMode::Nav || viewMode == ViewMode::MfdFull ||
                viewMode == ViewMode::ShipInspect) {
                // Recenter on the interpolated position — same point the camera sits at.
                // Using the true physics position would leave a step-size-dependent offset
                // between camera and origin, causing Earth to jump when time accel changes.
                scene.origin().recenter(earthWorld + spacecraft[playerIdx]->position());
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
            if (viewMode == ViewMode::Nav || viewMode == ViewMode::MfdFull) {
                auto& ship = *spacecraft[playerIdx];
                glm::vec3 shipRp = scene.origin().toRenderSpace(
                    earthWorld + ship.position());

                glm::mat3 attRot3 = glm::mat3_cast(glm::fquat(ship.attitude()));

                // rollFix matches the one applied in the draw call.
                glm::mat4 rollFix = glm::rotate(glm::mat4(1.0f),
                                                glm::radians(90.0f),
                                                glm::vec3(1.0f, 0.0f, 0.0f));
                glm::mat4 shipRot = glm::mat4(glm::mat3(attRot3)) * rollFix;

                if (orionGltf.isLoaded() && !navCamNodes.empty()) {
                    // Node transform is in metres (model space); directions are unit vectors.
                    glm::mat4 nodeTf = orionGltf.nodeWorldTransform(navCamNodes[navCamIdx]);
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
                auto& ship = *spacecraft[playerIdx];
                glm::vec3 shipRp = scene.origin().toRenderSpace(
                    earthWorld + ship.position());

                if (!ImGui::GetIO().WantCaptureMouse) {
                    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                        shipOrbit.azimuthDeg   -= dx * 0.4f;
                        shipOrbit.elevationDeg += dy * 0.4f;
                        shipOrbit.elevationDeg  = glm::clamp(shipOrbit.elevationDeg, -85.0f, 85.0f);
                    }
                    if (windowState.scrollDelta != 0.0)
                        shipOrbit.distanceM *= std::exp(-0.2f * static_cast<float>(windowState.scrollDelta));
                }

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

                ImGui::Text("Main engine (SPACE)");
                ImGui::SliderScalar("Main thrust (N)", ImGuiDataType_Double,
                    &mainEngineThrust,
                    (const double[]){1'000.0}, (const double[]){2'000'000.0},
                    "%.0f", ImGuiSliderFlags_Logarithmic);
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

                ImGui::Separator();
                ImGui::Text("RCS attitude (IJKL/UO)");
                ImGui::SliderScalar("RCS torque (N·m)", ImGuiDataType_Double,
                    &rcsTorque,
                    (const double[]){10.0}, (const double[]){100'000.0},
                    "%.0f", ImGuiSliderFlags_Logarithmic);

                ImGui::End();
            }

            // ---- F11 Ship inspection view ----
            if (viewMode == ViewMode::ShipInspect) {
                auto& ship = *spacecraft[playerIdx];
                ImGuiIO& io = ImGui::GetIO();

                constexpr auto kFlags =
                    ImGuiWindowFlags_NoDecoration    |
                    ImGuiWindowFlags_NoSavedSettings |
                    ImGuiWindowFlags_AlwaysAutoResize|
                    ImGuiWindowFlags_NoFocusOnAppearing;

                ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.55f);
                ImGui::Begin("##ShipInspect", nullptr, kFlags);

                ImGui::TextColored({0.0f, 0.82f, 0.30f, 1.0f}, "SHIP INSPECT  [F11]");
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
                ImGui::TextDisabled("Drag to orbit  Scroll to zoom");

                // Show active thrusters.
                if (!orionModel.thrusters.empty()) {
                    ImGui::Separator();
                    ImGui::Text("Active thrusters:");
                    int activeCount = 0;
                    for (const auto& t : orionModel.thrusters) {
                        if (t.throttle > 0.01f) {
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
                ImGui::End();
            }

            // ---- MFD fullscreen view (F2) ----
            if (viewMode == ViewMode::MfdFull) {
                auto& ship = *spacecraft[playerIdx];
                ImGuiIO& io = ImGui::GetIO();
                const float W = io.DisplaySize.x;
                const float H = io.DisplaySize.y;

                // Shared ref-body update (same logic as Nav).
                {
                    std::string pending = orbitalMFD.consumePendingRef();
                    if (!pending.empty()) {
                        SpiceInt    id;  SpiceBoolean found;
                        bodn2c_c(pending.c_str(), &id, &found);
                        if (found) {
                            SpiceInt    n;
                            SpiceDouble muArr[1], radii[3];
                            bodvrd_c(pending.c_str(), "GM",    1, &n, muArr);
                            bodvrd_c(pending.c_str(), "RADII", 3, &n, radii);
                            refBody = { pending, muArr[0], radii[0], id };
                            orbitalMFD.setContext(refBody.name.c_str(), "");
                        }
                    }
                }

                astro::PosState shipRelRef(ship.position(), ship.velocity());
                if (refBody.naifId != 399) {
                    SpiceDouble refState[6], lt;
                    spkgeo_c(refBody.naifId, currentEt.getETValue(),
                             "ECLIPJ2000", 399, refState, &lt);
                    shipRelRef = astro::PosState(
                        ship.position() - glm::dvec3(refState[0], refState[1], refState[2]),
                        ship.velocity() - glm::dvec3(refState[3], refState[4], refState[5]));
                }
                orbitalMFD.update(shipRelRef, currentEt, refBody.mu, refBody.radiusKm);

                MFDPanel fullPanel;
                fullPanel.pos  = { 0.0f, 0.0f };
                fullPanel.size = { W, H };
                fullPanel.app  = &orbitalMFD;
                fullPanel.render("##MFDFull");
            }

            // ---- Nav view HUD ----
            if (viewMode == ViewMode::Nav) {
                auto& ship = *spacecraft[playerIdx];
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
                ImGui::End();

                // ---- MFD panels — flush to screen left/right edges ----

                // Check if the user changed the reference body.
                {
                    std::string pending = orbitalMFD.consumePendingRef();
                    if (!pending.empty()) {
                        SpiceInt    id;  SpiceBoolean found;
                        bodn2c_c(pending.c_str(), &id, &found);
                        if (found) {
                            SpiceInt    n;
                            SpiceDouble muArr[1], radii[3];
                            bodvrd_c(pending.c_str(), "GM",    1, &n, muArr);
                            bodvrd_c(pending.c_str(), "RADII", 3, &n, radii);
                            refBody = { pending, muArr[0], radii[0], id };
                            orbitalMFD.setContext(refBody.name.c_str(), "");
                        }
                        // If not found, silently ignore (body stays unchanged).
                    }
                }

                // Compute spacecraft state relative to the reference body.
                astro::PosState shipRelRef(ship.position(), ship.velocity());
                if (refBody.naifId != 399) {  // 399 = Earth (already the origin)
                    SpiceDouble refState[6], lt;
                    spkgeo_c(refBody.naifId,
                             currentEt.getETValue(),
                             "ECLIPJ2000", 399,
                             refState, &lt);
                    shipRelRef = astro::PosState(
                        ship.position() - glm::dvec3(refState[0], refState[1], refState[2]),
                        ship.velocity() - glm::dvec3(refState[3], refState[4], refState[5]));
                }

                orbitalMFD.update(shipRelRef, currentEt,
                                  refBody.mu, refBody.radiusKm);

                // Each MFD occupies 1/3 of screen width; height derived from 16:9.
                const float kMfdW = std::round(W / 3.0f);
                const float kMfdH = std::round(kMfdW * (9.0f / 16.0f));
                const float mfdY  = H - kMfdH;

                MFDPanel leftPanel;
                leftPanel.pos  = { 0.0f, mfdY };
                leftPanel.size = { kMfdW, kMfdH };
                leftPanel.app  = &orbitalMFD;
                leftPanel.render("##MFD0");

                MFDPanel rightPanel;
                rightPanel.pos  = { W - kMfdW, mfdY };
                rightPanel.size = { kMfdW, kMfdH };
                rightPanel.app  = &orbitalMFD;
                rightPanel.render("##MFD1");

                // ---- Nav Console — bottom centre ----
                {
                    auto& ship = *spacecraft[playerIdx];

                    // Body-frame angular velocity (convert from inertial via attitude).
                    glm::dquat  att   = ship.attitude();
                    glm::dvec3  w_in  = ship.angularVelocity();
                    glm::dvec3  w_bod = glm::conjugate(att) * w_in;  // inertial → body

                    // Angular acceleration from finite difference.
                    // frameDt is the real-time frame delta (always > 0).
                    glm::dvec3 alpha_bod(0.0);
                    if (prevNavDt > 1e-6f) {
                        glm::dvec3 prevW_bod = glm::conjugate(att) * prevAngVelInertial;
                        alpha_bod = (w_bod - prevW_bod) / static_cast<double>(prevNavDt);
                    }

                    // Linear acceleration in inertial frame (km/s²) → convert to g.
                    constexpr double kGInKms2 = 9.80665e-3;  // 1 g in km/s²
                    glm::dvec3 linVel = ship.velocity();
                    glm::dvec3 linAcc_g(0.0);
                    if (prevNavDt > 1e-6f)
                        linAcc_g = (linVel - prevLinVelKms) / (static_cast<double>(prevNavDt) * kGInKms2);

                    // Prograde error: angle between ship nose (+X body in inertial) and velocity.
                    glm::dvec3 noseInertial = att * glm::dvec3(1.0, 0.0, 0.0);
                    double vMag = glm::length(linVel);
                    double progradeErrDeg = 0.0;
                    if (vMag > 1e-6) {
                        double c = glm::clamp(glm::dot(noseInertial, linVel / vMag), -1.0, 1.0);
                        progradeErrDeg = std::acos(c) * 180.0 / std::numbers::pi;
                    }

                    // Store for next frame.
                    prevAngVelInertial = w_in;
                    prevLinVelKms      = linVel;
                    prevNavDt          = frameDt;

                    // Convert to °/s and °/s².
                    constexpr double kR2D = 180.0 / std::numbers::pi;
                    glm::dvec3 w_degs  = w_bod  * kR2D;
                    glm::dvec3 al_degs = alpha_bod * kR2D;

                    // Vertical speed (km/s).
                    double vsKms = 0.0;
                    {
                        // Use Earth position as reference (current ref body would be better
                        // but Earth is always the floating origin here).
                        glm::dvec3 rHat = glm::length(ship.position()) > 1e-6
                            ? glm::normalize(ship.position()) : glm::dvec3(0, 0, 1);
                        vsKms = glm::dot(linVel, rHat);
                    }

                    // --- Draw the console panel ---
                    const float consW = kMfdW;
                    const ImU32 kGrn  = IM_COL32(  0, 210,  75, 210);
                    const ImU32 kDim  = IM_COL32(  0, 160,  55, 150);
                    const ImU32 kYel  = IM_COL32(220, 200,   0, 240);
                    const ImU32 kOra  = IM_COL32(240, 130,  20, 240);

                    constexpr auto kConFlags =
                        ImGuiWindowFlags_NoDecoration    |
                        ImGuiWindowFlags_NoSavedSettings |
                        ImGuiWindowFlags_NoFocusOnAppearing|
                        ImGuiWindowFlags_AlwaysAutoResize;

                    ImGui::SetNextWindowBgAlpha(0.30f);
                    ImGui::SetNextWindowPos(
                        ImVec2(kMfdW + (kMfdW - consW) * 0.5f, H),
                        ImGuiCond_Always, ImVec2(0.0f, 1.0f));  // anchor bottom-left
                    ImGui::SetNextWindowSize(ImVec2(consW, 0.0f));
                    ImGui::Begin("##NavConsole", nullptr, kConFlags);

                    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kGrn));

                    // Title
                    ImGui::SetCursorPosX((consW - ImGui::CalcTextSize("-- NAV CONSOLE --").x) * 0.5f);
                    ImGui::TextColored({0.0f, 0.82f, 0.30f, 0.8f}, "-- NAV CONSOLE --");
                    ImGui::Separator();

                    // Helper: two-column row.
                    auto rowf = [&](const char* lbl, const char* fmt, ...) {
                        char buf[64]; va_list ap; va_start(ap, fmt);
                        std::vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
                        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(kDim));
                        ImGui::Text("%s", lbl);
                        ImGui::PopStyleColor();
                        ImGui::SameLine(consW * 0.42f);
                        ImGui::Text("%s", buf);
                    };

                    // Rotational rates (body P/Y/R in °/s).
                    rowf("ω Pitch", "%+7.2f °/s", w_degs.y);
                    rowf("ω Yaw  ", "%+7.2f °/s", w_degs.z);
                    rowf("ω Roll ", "%+7.2f °/s", w_degs.x);
                    ImGui::Separator();

                    // Angular accelerations.
                    rowf("α Pitch", "%+7.2f °/s²", al_degs.y);
                    rowf("α Yaw  ", "%+7.2f °/s²", al_degs.z);
                    rowf("α Roll ", "%+7.2f °/s²", al_degs.x);
                    ImGui::Separator();

                    // Linear acceleration.
                    rowf("Acc X  ", "%+7.3f g", linAcc_g.x);
                    rowf("Acc Y  ", "%+7.3f g", linAcc_g.y);
                    rowf("Acc Z  ", "%+7.3f g", linAcc_g.z);
                    ImGui::Separator();

                    // Flight data.
                    rowf("Pro∠   ", "%7.1f °",  progradeErrDeg);
                    rowf("Vspd   ", "%+7.3f km/s", vsKms);
                    ImGui::Separator();

                    // Main engine indicator.
                    if (mainEngineOn)
                        ImGui::TextColored({1.0f, 0.50f, 0.10f, 1.0f}, "  *** MAIN ENGINE ***");
                    else
                        ImGui::TextColored({0.0f, 0.50f, 0.20f, 0.5f}, "  main engine off");
                    ImGui::Separator();

                    // Autopilot mode buttons (wired up later).
                    ImGui::TextColored({0.0f, 0.82f, 0.30f, 0.6f}, "Autopilot:");
                    ImGui::SameLine();
                    ImGui::TextColored({0.6f, 0.6f, 0.6f, 0.6f}, "(coming soon)");
                    if (ImGui::Button("Kill Rot")) { /* TODO */ }
                    ImGui::SameLine();
                    if (ImGui::Button("Prograde")) { /* TODO */ }
                    ImGui::SameLine();
                    if (ImGui::Button("Retro")) { /* TODO */ }
                    ImGui::SameLine();
                    if (ImGui::Button("Nrm+")) { /* TODO */ }
                    ImGui::SameLine();
                    if (ImGui::Button("Nrm-")) { /* TODO */ }

                    ImGui::PopStyleColor();
                    ImGui::End();
                }

                // ---- Helmet HUD overlay (drawn on top of everything) ----
                {
                    ImDrawList* dl = ImGui::GetForegroundDrawList();
                    const float cx = W * 0.5f;
                    const float cy = H * 0.5f;
                    const ImU32 kHudGreen  = IM_COL32(  0, 210,  75, 180);
                    const ImU32 kHudYellow = IM_COL32(220, 200,   0, 210);

                    // Center reticle — gap in the middle so it doesn't obscure target.
                    constexpr float kInner = 5.0f, kOuter = 14.0f;
                    dl->AddLine({cx - kOuter, cy}, {cx - kInner, cy}, kHudGreen, 1.5f);
                    dl->AddLine({cx + kInner, cy}, {cx + kOuter, cy}, kHudGreen, 1.5f);
                    dl->AddLine({cx, cy - kOuter}, {cx, cy - kInner}, kHudGreen, 1.5f);
                    dl->AddLine({cx, cy + kInner}, {cx, cy + kOuter}, kHudGreen, 1.5f);

                    // Project a normalised world-direction to ImGui screen coords.
                    // Vulkan NDC: y=-1 = top, y=+1 = bottom  →  sy = (ndc.y*0.5+0.5)*H
                    glm::vec3 shipRp = scene.origin().toRenderSpace(
                        earthWorld + ship.position());
                    glm::mat4 vp = camera.viewProjection();

                    auto projectDir = [&](glm::vec3 dir) -> std::optional<ImVec2> {
                        glm::vec4 clip = vp * glm::vec4(shipRp + dir * 1000.0f, 1.0f);
                        if (clip.w <= 0.0f) return std::nullopt; // behind camera
                        glm::vec3 ndc = glm::vec3(clip) / clip.w;
                        float sx = (ndc.x * 0.5f + 0.5f) * W;
                        float sy = (ndc.y * 0.5f + 0.5f) * H;
                        // Only draw if on-screen (with small margin).
                        if (sx < -50.0f || sx > W + 50.0f ||
                            sy < -50.0f || sy > H + 50.0f)
                            return std::nullopt;
                        return ImVec2{sx, sy};
                    };

                    // Prograde: circle with four external ticks.
                    auto drawPrograde = [&](ImVec2 p, ImU32 col) {
                        constexpr float r = 14.0f, tick = 8.0f;
                        dl->AddCircle(p, r, col, 0, 1.5f);
                        dl->AddLine({p.x - r - tick, p.y}, {p.x - r, p.y}, col, 1.5f);
                        dl->AddLine({p.x + r,        p.y}, {p.x + r + tick, p.y}, col, 1.5f);
                        dl->AddLine({p.x, p.y - r - tick}, {p.x, p.y - r}, col, 1.5f);
                        dl->AddLine({p.x, p.y + r},        {p.x, p.y + r + tick}, col, 1.5f);
                    };

                    // Retrograde: circle with X inside.
                    auto drawRetrograde = [&](ImVec2 p, ImU32 col) {
                        constexpr float r = 14.0f, d = 8.0f;
                        dl->AddCircle(p, r, col, 0, 1.5f);
                        dl->AddLine({p.x - d, p.y - d}, {p.x + d, p.y + d}, col, 1.5f);
                        dl->AddLine({p.x + d, p.y - d}, {p.x - d, p.y + d}, col, 1.5f);
                    };

                    glm::vec3 progradeDir = glm::normalize(glm::vec3(ship.velocity()));
                    if (auto p = projectDir( progradeDir)) drawPrograde (*p, kHudYellow);
                    if (auto p = projectDir(-progradeDir)) drawRetrograde(*p, kHudYellow);
                }
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

            if (!renderer.beginFrame()) continue;

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
            // Positions are in Earth-centred ECI (km); convert via earthWorld offset.
            {
                glm::vec3 scSunDir = (sunIndex >= 0)
                    ? glm::normalize(sunRenderPos - scene.origin().toRenderSpace(earthWorld))
                    : glm::vec3(0.0f, 1.0f, 0.0f);

                for (auto& sc : spacecraft) {
                    glm::dvec3 worldPos = earthWorld + sc->position();
                    glm::vec3  rp       = scene.origin().toRenderSpace(worldPos);

                    glm::mat3 attRot = glm::mat3_cast(glm::fquat(sc->attitude()));

                    // The Orion model is in metres; render space is in km → scale by 1e-3.
                    // Body +X is forward in both the physics model and glTF model space.
                    constexpr float kModelToKm = 1e-3f;
                    // 90° roll correction: Blender model has windows facing -Y,
                    // physics expects windows facing +Z (orbit normal).
                    glm::mat4 rollFix = glm::rotate(glm::mat4(1.0f),
                                                    glm::radians(90.0f),
                                                    glm::vec3(1.0f, 0.0f, 0.0f));
                    glm::mat4 shipModel = glm::translate(glm::mat4(1.0f), rp)
                                       * glm::mat4(glm::mat3(attRot))
                                       * rollFix
                                       * glm::scale(glm::mat4(1.0f), glm::vec3(kModelToKm));

                    if (orionGltf.isLoaded()) {
                        // Update plume node visibility and scale from throttles.
                        if (!orionModel.thrusters.empty()) {
                            for (const auto& t : orionModel.thrusters) {
                                if (t.exhaustNode.empty()) continue;
                                orionGltf.setNodeVisible(t.exhaustNode, t.firing);
                                if (t.firing)
                                    orionGltf.setNodeScale(t.exhaustNode, t.exhaustScale);
                            }
                        }
                        orionGltf.draw(renderer.currentCmd(), meshPipeline,
                                       vp, shipModel, scSunDir);
                    } else {
                        // Fallback: small emissive sphere.
                        constexpr float kShipRadiusKm = 0.01f;
                        glm::mat4 model = glm::translate(glm::mat4(1.0f), rp);
                        model = model * glm::mat4(glm::mat3(attRot));
                        model = glm::scale(model, glm::vec3(kShipRadiusKm));
                        glm::vec3 viewDir = glm::normalize(camera.position() - rp);
                        renderer.draw(vp * model, model, scSunDir, viewDir,
                                      /*emissive=*/true, 1.0f, 0.0f,
                                      descriptorSets[0], *meshes[0]);
                    }
                }
            }

            renderer.endFrame();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
