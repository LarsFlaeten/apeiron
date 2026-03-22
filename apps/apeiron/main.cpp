#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "apeiron/render/Camera.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Geometry.h"
#include "apeiron/render/GpuAllocator.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Renderer.h"
#include "apeiron/render/Mesh.h"
#include "apeiron/render/Vertex.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

#ifndef APEIRON_SHADER_DIR
#  error "APEIRON_SHADER_DIR not defined — configure via CMake"
#endif

int main()
{
    // ----- Window -----
    if (!glfwInit()) {
        std::cerr << "GLFW init failed\n";
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE,  GLFW_FALSE);

    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;

    GLFWwindow* window = glfwCreateWindow(
        kWidth, kHeight, "Apeiron — lit sphere", nullptr, nullptr);
    if (!window) {
        std::cerr << "Window creation failed\n";
        glfwTerminate();
        return 1;
    }

    // ----- Render stack -----
    // Constructed in dependency order; destroyed in reverse order by RAII.
    try {
        apeiron::render::Context      ctx(window);
        apeiron::render::GpuAllocator allocator(ctx);
        apeiron::render::Swapchain    swapchain(ctx, allocator, kWidth, kHeight);
        apeiron::render::Pipeline     pipeline (ctx, swapchain, APEIRON_SHADER_DIR);

        // Camera: 3 units back along +Z, looking at the origin.
        // near/far must match C_NEAR/C_FAR constants in triangle.frag.
        apeiron::render::Camera camera(
            glm::vec3(0.0f, 0.0f, 3.0f),
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f),
            45.0f,
            static_cast<float>(kWidth) / static_cast<float>(kHeight),
            0.1f, 1.0e9f
        );

        // Earth-like blue-green sphere, 1 m radius, 64 rings × 64 sectors.
        auto [verts, idxs] = apeiron::render::Geometry::makeSphere(
            1.0f, 64, 64, glm::vec3(0.25f, 0.52f, 0.95f));
        apeiron::render::Mesh     mesh    (allocator, verts, idxs);
        apeiron::render::Renderer renderer(ctx, swapchain, pipeline);

        // ----- Main loop -----
        constexpr float kRadiansPerSecond = 0.4f;
        auto startTime = std::chrono::steady_clock::now();

        glfwSetWindowUserPointer(window, &renderer);
        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int) {
            if (action != GLFW_PRESS) return;
            if (key == GLFW_KEY_ESCAPE) {
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            } else if (key == GLFW_KEY_W) {
                auto* r = static_cast<apeiron::render::Renderer*>(glfwGetWindowUserPointer(w));
                r->setWireframe(!r->wireframe());
            }
        });

        const glm::mat4 vp = camera.viewProjection();

        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();

            float     elapsed = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - startTime).count();
            glm::mat4 model   = glm::rotate(glm::mat4(1.0f),
                                            elapsed * kRadiansPerSecond,
                                            glm::vec3(0.0f, 1.0f, 0.0f));
            renderer.drawFrame(vp * model, model, mesh);
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
