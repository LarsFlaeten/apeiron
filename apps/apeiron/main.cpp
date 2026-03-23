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
#include "apeiron/render/Renderer.h"
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

#include <cmath>
#include <iostream>
#include <memory>
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

        auto& pool = apeiron::universe::KernelPool::instance();
        for (auto& k : cfg.kernels)
            pool.load(k);

        auto et = astro::EphemerisTime::fromString(cfg.epoch);

        apeiron::universe::Observer observer(
            cfg.observerBody, cfg.observerTarget, cfg.frame);

        apeiron::universe::Scene scene;

        // Plain body info — no GPU resources, safe to declare here.
        struct BodyInfo {
            apeiron::universe::CelestialBody* node;
            float     radiusKm;
            glm::vec3 color;
            std::string diffusePath;
            std::string specularPath;
            std::string normalPath;
            std::string cloudsPath;
        };
        std::vector<BodyInfo> bodyInfos;

        // Index of the Sun body in bodyInfos (for computing light direction).
        int sunIndex = -1;

        for (int i = 0; i < static_cast<int>(cfg.bodies.size()); ++i) {
            auto& bc    = cfg.bodies[i];
            auto  props = apeiron::universe::BodyProperties::queryFromSpice(bc.naif);
            auto& node  = scene.addBody(bc.naif, bc.naif, props.radiusKm,
                                        "SOLAR SYSTEM BARYCENTER", cfg.frame);
            bodyInfos.push_back({ &node, static_cast<float>(props.radiusKm),
                                   bc.color, bc.diffusePath, bc.specularPath,
                                   bc.normalPath, bc.cloudsPath });
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
        apeiron::render::Context      ctx(window);
        apeiron::render::GpuAllocator allocator(ctx);
        apeiron::render::Swapchain    swapchain(ctx, allocator, kWidth, kHeight);
        apeiron::render::Pipeline     pipeline (ctx, swapchain, APEIRON_SHADER_DIR);

        // Meshes and textures declared after allocator — destroyed before allocator.
        // Textures declared before renderer so they outlive it (destroyed after renderer).
        using Tex = apeiron::render::Texture;
        struct BodyTextures { Tex diffuse, specular, normal, clouds; };

        auto load = [&](const std::string& path, Tex fallback) -> Tex {
            return path.empty() ? std::move(fallback)
                                : Tex(ctx, allocator, path);
        };

        std::vector<std::unique_ptr<apeiron::render::Mesh>> meshes;
        std::vector<BodyTextures> bodyTextures;
        bodyTextures.reserve(bodyInfos.size());
        for (auto& bi : bodyInfos) {
            auto [verts, idxs] = apeiron::render::Geometry::makeSphere(
                1.0f, 64, 64, bi.color);
            meshes.push_back(std::make_unique<apeiron::render::Mesh>(
                allocator, verts, idxs));
            bodyTextures.push_back({
                load(bi.diffusePath,  Tex::makeWhite        (ctx, allocator)),
                load(bi.specularPath, Tex::makeBlack        (ctx, allocator)),
                load(bi.normalPath,   Tex::makeNeutralNormal(ctx, allocator)),
                load(bi.cloudsPath,   Tex::makeBlack        (ctx, allocator))
            });
        }

        // Orbit camera: azimuth/elevation around scene origin, Z = ecliptic north.
        struct OrbitCamera {
            float azimuthDeg   =   0.0f;
            float elevationDeg =  60.0f;
            float distanceKm   = 1'000'000.0f;

            glm::vec3 position() const {
                float az = glm::radians(azimuthDeg);
                float el = glm::radians(elevationDeg);
                return distanceKm * glm::vec3(
                    std::cos(el) * std::cos(az),
                    std::cos(el) * std::sin(az),
                    std::sin(el));
            }
        } orbit;

        apeiron::render::Camera camera(
            orbit.position(),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 1.0f),   // Z = ecliptic north
            45.0f,
            static_cast<float>(kWidth) / static_cast<float>(kHeight),
            0.1f, 1.0e9f  // must match C_NEAR / C_FAR in triangle.frag
        );

        // Renderer last — its destructor calls waitIdle before anything else frees.
        apeiron::render::Renderer renderer(ctx, swapchain, pipeline);
        renderer.initImGui(window);

        // Descriptor sets — allocated from Renderer's pool, freed with it.
        std::vector<vk::DescriptorSet> descriptorSets;
        for (auto& bt : bodyTextures)
            descriptorSets.push_back(renderer.allocateDescriptorSet(
                {bt.diffuse.imageView(), bt.specular.imageView(),
                 bt.normal.imageView(),  bt.clouds.imageView()},
                {bt.diffuse.sampler(),   bt.specular.sampler(),
                 bt.normal.sampler(),    bt.clouds.sampler()}));

        // Window state shared between callbacks.
        struct WindowState {
            apeiron::render::Renderer* renderer;
            double scrollDelta = 0.0;
        } windowState{&renderer};

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
            camera.setPosition(orbit.position());

            simElapsed += frameDt * simSecondsPerRealSecond;
            auto currentEt = et + astro::TimeDelta(simElapsed);

            observerPos = observer.worldPosition(currentEt);
            scene.update(currentEt, observerPos);

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
            ImGui::Separator();

            ImGui::Text("Simulation");
            ImGui::SliderScalar("Speed (s/s)", ImGuiDataType_Double,
                                &simSecondsPerRealSecond,
                                (const double[]){1.0}, (const double[]){1e6},
                                "%.0f", ImGuiSliderFlags_Logarithmic);

            ImGui::Separator();
            ImGui::Text("Camera");
            ImGui::Text("  Frame    : %s", cfg.frame.c_str());
            ImGui::Text("  Focus    : %s", cfg.observerBody.c_str());
            ImGui::Text("  Distance : %.0f km  /  %.4f AU",
                        orbit.distanceKm, orbit.distanceKm / 149'597'870.7f);
            ImGui::Text("  Azimuth  : %.1f°", orbit.azimuthDeg);
            ImGui::Text("  Elevation: %.1f°", orbit.elevationDeg);

            ImGui::Separator();
            ImGui::Text("Bodies");
            if (ImGui::BeginTable("bodies", 3,
                                  ImGuiTableFlags_Borders |
                                  ImGuiTableFlags_RowBg   |
                                  ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Dist from cam (km)");
                ImGui::TableSetupColumn("Dist from cam (AU)");
                ImGui::TableHeadersRow();
                for (auto& bi : bodyInfos) {
                    glm::vec3 rp   = scene.origin().toRenderSpace(bi.node->worldPosition());
                    float     dist = glm::length(rp - camera.position());
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::Text("%s", bi.node->naifName().c_str());
                    ImGui::TableSetColumnIndex(1); ImGui::Text("%.0f", dist);
                    ImGui::TableSetColumnIndex(2); ImGui::Text("%.4f", dist / 149'597'870.7f);
                }
                ImGui::EndTable();
            }
            ImGui::End();

            ImGui::Render();

            // --- GPU frame ---
            const glm::mat4 vp = camera.viewProjection();

            if (!renderer.beginFrame()) continue;

            for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
                auto& bi = bodyInfos[i];

                glm::vec3 renderPos = scene.origin().toRenderSpace(
                    bi.node->worldPosition());

                // T * R * S: translate to body position, rotate by IAU orientation,
                // then scale to physical radius.
                glm::mat4 model = glm::translate(glm::mat4(1.0f), renderPos);
                model = model * glm::mat4(bi.node->orientation());
                model = glm::scale(model, glm::vec3(bi.radiusKm));

                // Per-body sun direction.
                glm::vec3 sunDir = (sunIndex >= 0 && static_cast<int>(i) != sunIndex)
                    ? glm::normalize(sunRenderPos - renderPos)
                    : glm::vec3(0.0f, 1.0f, 0.0f);

                glm::vec3 viewDir = glm::normalize(camera.position() - renderPos);
                renderer.draw(vp * model, model, sunDir, viewDir,
                              descriptorSets[i], *meshes[i]);
            }

            renderer.renderImGui();
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
