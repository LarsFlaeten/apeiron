#pragma once

#include "apeiron/render/Image.h"

#include <vulkan/vulkan.hpp>
#include <filesystem>

namespace apeiron::render {

class Context;
class GpuAllocator;

// A 2-D GPU texture: owns a VMA-backed Image, an ImageView, and a Sampler.
// Movable but not copyable.
class Texture {
public:
    // Load from an image file (JPEG, PNG, etc.) via stb_image.
    // Pass linear=true for data textures (heightmaps, masks) that must not
    // have sRGB gamma applied by the sampler hardware.
    Texture(const Context& ctx, GpuAllocator& allocator,
            const std::filesystem::path& path, bool linear = false);

    static Texture makeWhite        (const Context& ctx, GpuAllocator& allocator); // diffuse fallback
    static Texture makeBlack        (const Context& ctx, GpuAllocator& allocator); // specular/clouds fallback
    static Texture makeNeutralNormal(const Context& ctx, GpuAllocator& allocator); // normal map fallback (flat)

    ~Texture();

    Texture(Texture&&) noexcept;
    Texture& operator=(Texture&&) noexcept;

    Texture(const Texture&)            = delete;
    Texture& operator=(const Texture&) = delete;

    vk::ImageView imageView() const { return m_view;    }
    vk::Sampler   sampler()   const { return m_sampler; }

private:
    Texture() = default;

    void upload(const Context& ctx, GpuAllocator& allocator,
                const uint8_t* pixels, int width, int height, bool linear = false);

    const Context* m_ctx{};
    Image          m_image;
    vk::ImageView  m_view;
    vk::Sampler    m_sampler;
};

} // namespace apeiron::render
