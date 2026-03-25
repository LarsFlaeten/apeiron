#include "apeiron/render/Context.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <stdexcept>

namespace apeiron::render {

#ifdef NDEBUG
constexpr bool kEnableValidation = false;
#else
constexpr bool kEnableValidation = true;
#endif

Context::Context(GLFWwindow* window)
{
    // --- Instance ---
    // Ask GLFW which surface extensions are needed (e.g. VK_KHR_xcb_surface).
    uint32_t    glfwExtCount = 0;
    const char** glfwExts    = glfwGetRequiredInstanceExtensions(&glfwExtCount);

    vkb::InstanceBuilder builder;
    builder.set_app_name("Apeiron")
           .require_api_version(1, 3, 0)
           .request_validation_layers(kEnableValidation)
           .use_default_debug_messenger();
    for (uint32_t i = 0; i < glfwExtCount; ++i)
        builder.enable_extension(glfwExts[i]);

    auto instResult = builder.build();
    if (!instResult)
        throw std::runtime_error("Vulkan instance creation failed: "
                                 + instResult.error().message());
    m_vkbInstance = instResult.value();
    m_instance    = vk::Instance(m_vkbInstance.instance);

    // --- Surface ---
    VkSurfaceKHR rawSurface{};
    if (glfwCreateWindowSurface(m_vkbInstance.instance, window, nullptr, &rawSurface) != VK_SUCCESS)
        throw std::runtime_error("Failed to create window surface");
    m_surface = vk::SurfaceKHR(rawSurface);

    // --- Physical device ---
    VkPhysicalDeviceFeatures requiredFeatures{};
    requiredFeatures.fillModeNonSolid   = VK_TRUE; // needed for wireframe rendering
    requiredFeatures.tessellationShader = VK_TRUE; // needed for displacement mapping

    vkb::PhysicalDeviceSelector selector(m_vkbInstance);
    auto physResult = selector
        .set_surface(rawSurface)
        .set_minimum_version(1, 3)
        .prefer_gpu_device_type(vkb::PreferredDeviceType::discrete)
        .set_required_features(requiredFeatures)
        .select();
    if (!physResult)
        throw std::runtime_error("Physical device selection failed: "
                                 + physResult.error().message());
    m_physDevice = vk::PhysicalDevice(physResult.value().physical_device);

    // --- Logical device ---
    vkb::DeviceBuilder deviceBuilder(physResult.value());
    auto devResult = deviceBuilder.build();
    if (!devResult)
        throw std::runtime_error("Logical device creation failed: "
                                 + devResult.error().message());
    m_vkbDevice = devResult.value();
    m_device    = vk::Device(m_vkbDevice.device);

    // --- Queues ---
    auto gfxQ = m_vkbDevice.get_queue(vkb::QueueType::graphics);
    if (!gfxQ)
        throw std::runtime_error("Failed to get graphics queue");
    m_graphicsQueue  = vk::Queue(gfxQ.value());
    m_graphicsFamily = m_vkbDevice.get_queue_index(vkb::QueueType::graphics).value();

    auto presQ = m_vkbDevice.get_queue(vkb::QueueType::present);
    if (!presQ)
        throw std::runtime_error("Failed to get present queue");
    m_presentQueue  = vk::Queue(presQ.value());
    m_presentFamily = m_vkbDevice.get_queue_index(vkb::QueueType::present).value();
}

Context::~Context()
{
    if (m_device)
        m_device.waitIdle();
    vkb::destroy_device(m_vkbDevice);
    m_instance.destroySurfaceKHR(m_surface);
    vkb::destroy_instance(m_vkbInstance);
}

} // namespace apeiron::render
