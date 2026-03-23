#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "apeiron/render/Texture.h"
#include "apeiron/render/Buffer.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/GpuAllocator.h"

#include <stdexcept>

namespace apeiron::render {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static void transitionImageLayout(vk::CommandBuffer cmd, vk::Image image,
                                   vk::ImageLayout oldLayout,
                                   vk::ImageLayout newLayout)
{
    vk::ImageMemoryBarrier barrier{};
    barrier.setOldLayout          (oldLayout)
           .setNewLayout          (newLayout)
           .setSrcQueueFamilyIndex(vk::QueueFamilyIgnored)
           .setDstQueueFamilyIndex(vk::QueueFamilyIgnored)
           .setImage              (image);
    barrier.subresourceRange
        .setAspectMask    (vk::ImageAspectFlagBits::eColor)
        .setBaseMipLevel  (0)
        .setLevelCount    (1)
        .setBaseArrayLayer(0)
        .setLayerCount    (1);

    vk::PipelineStageFlags srcStage, dstStage;
    if (oldLayout == vk::ImageLayout::eUndefined &&
        newLayout == vk::ImageLayout::eTransferDstOptimal)
    {
        barrier.setSrcAccessMask({})
               .setDstAccessMask(vk::AccessFlagBits::eTransferWrite);
        srcStage = vk::PipelineStageFlagBits::eTopOfPipe;
        dstStage = vk::PipelineStageFlagBits::eTransfer;
    }
    else if (oldLayout == vk::ImageLayout::eTransferDstOptimal &&
             newLayout == vk::ImageLayout::eShaderReadOnlyOptimal)
    {
        barrier.setSrcAccessMask(vk::AccessFlagBits::eTransferWrite)
               .setDstAccessMask(vk::AccessFlagBits::eShaderRead);
        srcStage = vk::PipelineStageFlagBits::eTransfer;
        dstStage = vk::PipelineStageFlagBits::eFragmentShader;
    }
    else {
        throw std::runtime_error("Unsupported image layout transition");
    }

    cmd.pipelineBarrier(srcStage, dstStage, {}, {}, {}, barrier);
}

// ---------------------------------------------------------------------------
// upload — create Image, stage pixel data, transition to shader-readable.
// ---------------------------------------------------------------------------

void Texture::upload(const Context& ctx, GpuAllocator& allocator,
                     const uint8_t* pixels, int width, int height)
{
    m_ctx = &ctx;

    vk::DeviceSize imageSize = static_cast<vk::DeviceSize>(width * height * 4);

    Buffer staging(allocator.handle(), imageSize,
                   vk::BufferUsageFlagBits::eTransferSrc,
                   VMA_MEMORY_USAGE_AUTO,
                   VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT);
    staging.upload(pixels, imageSize);

    m_image = Image(allocator.handle(),
                    vk::Extent2D{static_cast<uint32_t>(width),
                                 static_cast<uint32_t>(height)},
                    vk::Format::eR8G8B8A8Srgb,
                    vk::ImageUsageFlagBits::eTransferDst |
                    vk::ImageUsageFlagBits::eSampled);

    allocator.immediateSubmit([&](vk::CommandBuffer cmd) {
        transitionImageLayout(cmd, m_image.handle(),
                              vk::ImageLayout::eUndefined,
                              vk::ImageLayout::eTransferDstOptimal);

        vk::BufferImageCopy region{};
        region.setBufferOffset     (0)
              .setBufferRowLength  (0)
              .setBufferImageHeight(0);
        region.imageSubresource
            .setAspectMask    (vk::ImageAspectFlagBits::eColor)
            .setMipLevel      (0)
            .setBaseArrayLayer(0)
            .setLayerCount    (1);
        region.setImageOffset({0, 0, 0})
              .setImageExtent ({static_cast<uint32_t>(width),
                                static_cast<uint32_t>(height), 1});

        cmd.copyBufferToImage(staging.handle(), m_image.handle(),
                              vk::ImageLayout::eTransferDstOptimal, region);

        transitionImageLayout(cmd, m_image.handle(),
                              vk::ImageLayout::eTransferDstOptimal,
                              vk::ImageLayout::eShaderReadOnlyOptimal);
    });

    // ImageView
    vk::ImageViewCreateInfo viewInfo{};
    viewInfo.setImage   (m_image.handle())
            .setViewType(vk::ImageViewType::e2D)
            .setFormat  (vk::Format::eR8G8B8A8Srgb);
    viewInfo.subresourceRange
        .setAspectMask    (vk::ImageAspectFlagBits::eColor)
        .setBaseMipLevel  (0)
        .setLevelCount    (1)
        .setBaseArrayLayer(0)
        .setLayerCount    (1);
    m_view = ctx.device().createImageView(viewInfo);

    // Sampler (bilinear, clamp-to-edge)
    vk::SamplerCreateInfo samplerInfo{};
    samplerInfo.setMagFilter      (vk::Filter::eLinear)
               .setMinFilter      (vk::Filter::eLinear)
               .setAddressModeU   (vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeV   (vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeW   (vk::SamplerAddressMode::eClampToEdge)
               .setAnisotropyEnable(vk::False)
               .setBorderColor    (vk::BorderColor::eIntOpaqueBlack)
               .setUnnormalizedCoordinates(vk::False)
               .setCompareEnable  (vk::False)
               .setMipmapMode     (vk::SamplerMipmapMode::eLinear);
    m_sampler = ctx.device().createSampler(samplerInfo);
}

// ---------------------------------------------------------------------------
// Constructors / destructor
// ---------------------------------------------------------------------------

Texture::Texture(const Context& ctx, GpuAllocator& allocator,
                 const std::filesystem::path& path)
{
    int   width{}, height{}, channels{};
    auto* pixels = stbi_load(path.string().c_str(), &width, &height, &channels, STBI_rgb_alpha);
    if (!pixels)
        throw std::runtime_error("Failed to load texture: " + path.string()
                                 + " — " + stbi_failure_reason()
                                 + " (supported: JPEG, PNG, BMP, TGA, GIF, PSD, HDR)");

    upload(ctx, allocator, pixels, width, height);
    stbi_image_free(pixels);
}

Texture Texture::makeWhite(const Context& ctx, GpuAllocator& allocator)
{
    Texture t;
    uint8_t pixels[4] = {255, 255, 255, 255};
    t.upload(ctx, allocator, pixels, 1, 1);
    return t;
}

Texture Texture::makeBlack(const Context& ctx, GpuAllocator& allocator)
{
    Texture t;
    uint8_t pixels[4] = {0, 0, 0, 255};
    t.upload(ctx, allocator, pixels, 1, 1);
    return t;
}

Texture Texture::makeNeutralNormal(const Context& ctx, GpuAllocator& allocator)
{
    // (128, 128, 255) decodes to tangent-space (0, 0, 1) — no perturbation.
    Texture t;
    uint8_t pixels[4] = {128, 128, 255, 255};
    t.upload(ctx, allocator, pixels, 1, 1);
    return t;
}

Texture::~Texture()
{
    if (m_ctx) {
        m_ctx->device().destroySampler  (m_sampler);
        m_ctx->device().destroyImageView(m_view);
    }
}

Texture::Texture(Texture&& o) noexcept
    : m_ctx    (o.m_ctx)
    , m_image  (std::move(o.m_image))
    , m_view   (o.m_view)
    , m_sampler(o.m_sampler)
{
    o.m_ctx     = nullptr;
    o.m_view    = nullptr;
    o.m_sampler = nullptr;
}

Texture& Texture::operator=(Texture&& o) noexcept
{
    if (this != &o) {
        if (m_ctx) {
            m_ctx->device().destroySampler  (m_sampler);
            m_ctx->device().destroyImageView(m_view);
        }
        m_ctx     = o.m_ctx;
        m_image   = std::move(o.m_image);
        m_view    = o.m_view;
        m_sampler = o.m_sampler;
        o.m_ctx     = nullptr;
        o.m_view    = nullptr;
        o.m_sampler = nullptr;
    }
    return *this;
}

} // namespace apeiron::render
