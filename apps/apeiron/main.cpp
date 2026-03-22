#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "apeiron/render/Context.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Renderer.h"

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
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API); // no OpenGL context
    glfwWindowHint(GLFW_RESIZABLE,  GLFW_FALSE);  // resize handling added later

    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;

    GLFWwindow* window = glfwCreateWindow(
        kWidth, kHeight, "Apeiron — spinning triangle", nullptr, nullptr);
    if (!window) {
        std::cerr << "Window creation failed\n";
        glfwTerminate();
        return 1;
    }

    // ----- Render stack -----
    // Constructed in dependency order; destroyed in reverse order by RAII.
    try {
        apeiron::render::Context   ctx(window);
        apeiron::render::Swapchain swapchain(ctx, kWidth, kHeight);
        apeiron::render::Pipeline  pipeline (ctx, swapchain, APEIRON_SHADER_DIR);
        apeiron::render::Renderer  renderer (ctx, swapchain, pipeline);

        // ----- Main loop -----
        constexpr float kRadiansPerSecond = 1.0f; // one full turn every ~6.3 s
        auto startTime = std::chrono::steady_clock::now();

        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int, int action, int) {
            if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
                glfwSetWindowShouldClose(w, GLFW_TRUE);
        });

        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();

            float elapsed = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - startTime).count();
            float angle = std::fmod(elapsed * kRadiansPerSecond,
                                    2.0f * 3.14159265f);

            renderer.drawFrame(angle);
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
