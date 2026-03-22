#pragma once

#include <vulkan/vulkan.hpp>
#include <VkBootstrap.h>

// Forward-declare so this header doesn't pull in all of GLFW.
struct GLFWwindow;

namespace apeiron::render {

// Owns the Vulkan instance, surface, physical device, logical device,
// and the graphics/present queues.  Built once at startup via vk-bootstrap.
class Context {
public:
    explicit Context(GLFWwindow* window);
    ~Context();

    Context(const Context&)            = delete;
    Context& operator=(const Context&) = delete;

    vk::Instance       instance()       const { return m_instance;       }
    vk::PhysicalDevice physicalDevice() const { return m_physDevice;     }
    vk::Device         device()         const { return m_device;         }
    vk::SurfaceKHR     surface()        const { return m_surface;        }
    vk::Queue          graphicsQueue()  const { return m_graphicsQueue;  }
    uint32_t           graphicsFamily() const { return m_graphicsFamily; }
    vk::Queue          presentQueue()   const { return m_presentQueue;   }
    uint32_t           presentFamily()  const { return m_presentFamily;  }

    // Exposed for vkb::SwapchainBuilder, which needs the vkb::Device struct.
    const vkb::Device& vkbDevice() const { return m_vkbDevice; }

private:
    vkb::Instance   m_vkbInstance{};
    vkb::Device     m_vkbDevice{};

    vk::Instance       m_instance;
    vk::PhysicalDevice m_physDevice;
    vk::Device         m_device;
    vk::SurfaceKHR     m_surface;

    vk::Queue  m_graphicsQueue;
    uint32_t   m_graphicsFamily{};
    vk::Queue  m_presentQueue;
    uint32_t   m_presentFamily{};
};

} // namespace apeiron::render
