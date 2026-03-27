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
#include "apeiron/render/StarField.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Texture.h"
#include "apeiron/render/Vertex.h"

#include "apeiron/universe/BodyProperties.h"
#include "apeiron/universe/CelestialBody.h"
#include "apeiron/universe/KernelPool.h"
#include "apeiron/universe/Observer.h"
#include "apeiron/universe/Scene.h"

#include "astro/Time.h"
#include "ScenarioConfig.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cspice/SpiceUsr.h>

#include <cmath>
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
                                   bc.ringTexturePath, bc.ringInnerRadius, bc.ringOuterRadius });
            if (bc.naif == "SUN") sunIndex = i;
        }

        auto observerPos = observer.worldPosition(et);
        scene.update(et, observerPos);

        // Earth radius for camera setup.
        float earthRadius = 6371.0f;
        for (auto& bi : bodyInfos)
            if (bi.node->naifName() == "EARTH") { earthRadius = bi.radiusKm; break; }

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

        // Star field — loaded after SPICE kernels are in memory (pxform_c needs them).
        glfwSetWindowTitle(window, "Apeiron — Loading star catalog…");
        glfwPollEvents();
        auto starVertices = loadStars(APEIRON_STAR_CATALOG, 7.5f);
        apeiron::render::StarField starField(ctx, allocator,
                                             pipeline.renderPass(), swapchain,
                                             APEIRON_SHADER_DIR, std::move(starVertices));

        // Renderer last — its destructor calls waitIdle before anything else frees.
        apeiron::render::Renderer renderer(ctx, swapchain, pipeline, tonemap, bloom);
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

        // Window state shared between callbacks.
        struct WindowState {
            apeiron::render::Renderer* renderer;
            double scrollDelta = 0.0;
        } windowState{&renderer};

        bool isFullscreen = false;
        bool needsResize  = false;
        int  windowedX = 100, windowedY = 100;

        glfwSetWindowUserPointer(window, &windowState);
        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int) {
            if (action != GLFW_PRESS) return;
            auto* s = static_cast<WindowState*>(glfwGetWindowUserPointer(w));
            if (key == GLFW_KEY_ESCAPE)
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            else if (key == GLFW_KEY_W)
                s->renderer->setWireframe(!s->renderer->wireframe());
        });
        glfwSetScrollCallback(window, [](GLFWwindow* w, double, double yoff) {
            auto* s = static_cast<WindowState*>(glfwGetWindowUserPointer(w));
            s->scrollDelta += yoff;
        });

        // =================================================================
        // Main loop
        // =================================================================
        double simSecondsPerRealSecond = 3600.0; // adjustable via ImGui
        double simElapsed = 0.0; // accumulated simulation seconds

        auto prevFrameTime = std::chrono::steady_clock::now();
        double prevMouseX{}, prevMouseY{};
        glfwGetCursorPos(window, &prevMouseX, &prevMouseY);

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
            windowState.scrollDelta = 0.0;

            // Focus point: render-space position of the selected body.
            if (selectedBodyIndex >= 0 && selectedBodyIndex < (int)bodyInfos.size())
                focusRenderPos = scene.origin().toRenderSpace(
                    bodyInfos[selectedBodyIndex].node->worldPosition());

            camera.setPosition(focusRenderPos + orbit.offset());
            camera.setTarget  (focusRenderPos);

            simElapsed += frameDt * simSecondsPerRealSecond;
            auto currentEt = et + astro::TimeDelta(simElapsed);

            observerPos = observer.worldPosition(currentEt);
            scene.update(currentEt, observerPos);

            // Recenter the floating origin on the focused body so its render-space
            // position is (0,0,0).  This keeps all float32 values small and avoids
            // precision jitter when orbiting a body far from the observer.
            if (selectedBodyIndex >= 0 && selectedBodyIndex < (int)bodyInfos.size())
                scene.origin().recenter(
                    bodyInfos[selectedBodyIndex].node->worldPosition());

            // Sun's render-space position (used as light source for all bodies).
            glm::vec3 sunRenderPos{0.0f};
            if (sunIndex >= 0) {
                sunRenderPos = scene.origin().toRenderSpace(
                    bodyInfos[sunIndex].node->worldPosition());
            }

            // --- ImGui frame ---
            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGui::Begin("Dev View");
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
                                &simSecondsPerRealSecond,
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
