#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "apeiron/render/Camera.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Geometry.h"
#include "apeiron/render/GpuAllocator.h"
#include "apeiron/render/Mesh.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Renderer.h"
#include "apeiron/render/Swapchain.h"
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

#include <iostream>
#include <memory>
#include <stdexcept>
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
        };
        std::vector<BodyInfo> bodyInfos;

        // Index of the Sun body in bodyInfos (for computing light direction).
        int sunIndex = -1;

        for (int i = 0; i < static_cast<int>(cfg.bodies.size()); ++i) {
            auto& bc    = cfg.bodies[i];
            auto  props = apeiron::universe::BodyProperties::queryFromSpice(bc.naif);
            auto& node  = scene.addBody(bc.naif, bc.naif,
                                        props.radiusKm, "SOLAR SYSTEM BARYCENTER");
            bodyInfos.push_back({ &node, static_cast<float>(props.radiusKm), bc.color });
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

        // Meshes declared after allocator — destroyed before allocator.
        std::vector<std::unique_ptr<apeiron::render::Mesh>> meshes;
        for (auto& bi : bodyInfos) {
            auto [verts, idxs] = apeiron::render::Geometry::makeSphere(
                1.0f, 64, 64, bi.color);
            meshes.push_back(std::make_unique<apeiron::render::Mesh>(
                allocator, verts, idxs));
        }

        // Camera: 10× Earth radii back, looking at Earth (floating-origin centre).
        // At this distance Earth is well-framed and the Moon (~60 R_E away) is visible.
        apeiron::render::Camera camera(
            glm::vec3(0.0f, 0.0f, 10.0f * earthRadius),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f),
            45.0f,
            static_cast<float>(kWidth) / static_cast<float>(kHeight),
            0.1f, 1.0e9f  // must match C_NEAR / C_FAR in triangle.frag
        );

        // Renderer last — its destructor calls waitIdle before anything else frees.
        apeiron::render::Renderer renderer(ctx, swapchain, pipeline);

        glfwSetWindowUserPointer(window, &renderer);
        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int) {
            if (action != GLFW_PRESS) return;
            if (key == GLFW_KEY_ESCAPE)
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            else if (key == GLFW_KEY_W) {
                auto* r = static_cast<apeiron::render::Renderer*>(
                    glfwGetWindowUserPointer(w));
                r->setWireframe(!r->wireframe());
            }
        });

        // =================================================================
        // Main loop
        // =================================================================
        constexpr double kSecondsPerSimSecond = 3600.0; // 1 h per real second
        auto wallStart = std::chrono::steady_clock::now();

        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();

            double wallElapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - wallStart).count();
            auto currentEt = et + astro::TimeDelta(wallElapsed * kSecondsPerSimSecond);

            observerPos = observer.worldPosition(currentEt);
            scene.update(currentEt, observerPos);

            // Sun's render-space position (used as light source for all bodies).
            glm::vec3 sunRenderPos{0.0f};
            if (sunIndex >= 0) {
                sunRenderPos = scene.origin().toRenderSpace(
                    bodyInfos[sunIndex].node->worldPosition());
            }

            const glm::mat4 vp = camera.viewProjection();

            if (!renderer.beginFrame()) continue;

            for (std::size_t i = 0; i < bodyInfos.size(); ++i) {
                auto& bi = bodyInfos[i];

                glm::vec3 renderPos = scene.origin().toRenderSpace(
                    bi.node->worldPosition());

                glm::mat4 model = glm::translate(glm::mat4(1.0f), renderPos);
                model           = glm::scale(model, glm::vec3(bi.radiusKm));

                // Per-body sun direction: from this body toward the sun.
                glm::vec3 sunDir = (sunIndex >= 0 && static_cast<int>(i) != sunIndex)
                    ? glm::normalize(sunRenderPos - renderPos)
                    : glm::vec3(0.0f, 1.0f, 0.0f); // sun lights itself uniformly

                renderer.draw(vp * model, model, sunDir, *meshes[i]);
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
