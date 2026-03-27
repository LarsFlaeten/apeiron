#include "apeiron/render/AtmospherePrecompute.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/GpuAllocator.h"

#include <fstream>
#include <stdexcept>
#include <vector>

namespace apeiron::render {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::vector<uint32_t> readSpirv(const std::filesystem::path& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open())
        throw std::runtime_error("Cannot open shader: " + path.string());
    auto bytes = static_cast<std::streamsize>(f.tellg());
    if (bytes % 4 != 0)
        throw std::runtime_error("SPIR-V not aligned: " + path.string());
    f.seekg(0);
    std::vector<uint32_t> code(bytes / 4);
    f.read(reinterpret_cast<char*>(code.data()), bytes);
    return code;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

AtmospherePrecompute::AtmospherePrecompute(const Context&               ctx,
                                           GpuAllocator&                allocator,
                                           const std::filesystem::path& shaderDir,
                                           const Params&                params)
    : m_ctx(ctx), m_params(params)
{
    auto device = ctx.device();

    // Allocate the transmittance LUT image (storage + sampled).
    m_lut = Image(allocator.handle(),
                  {kLutW, kLutH},
                  vk::Format::eR32G32B32A32Sfloat,
                  vk::ImageUsageFlagBits::eStorage |
                  vk::ImageUsageFlagBits::eSampled);

    // Image view for both compute write and shader read.
    vk::ImageViewCreateInfo viewInfo{};
    viewInfo.setImage          (m_lut.handle())
            .setViewType       (vk::ImageViewType::e2D)
            .setFormat         (vk::Format::eR32G32B32A32Sfloat)
            .setSubresourceRange({vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1});
    m_lutView = device.createImageView(viewInfo);

    // Linear sampler with clamp-to-edge for LUT lookups.
    vk::SamplerCreateInfo sampInfo{};
    sampInfo.setMagFilter    (vk::Filter::eLinear)
            .setMinFilter    (vk::Filter::eLinear)
            .setAddressModeU (vk::SamplerAddressMode::eClampToEdge)
            .setAddressModeV (vk::SamplerAddressMode::eClampToEdge)
            .setAddressModeW (vk::SamplerAddressMode::eClampToEdge);
    m_sampler = device.createSampler(sampInfo);

    runPrecompute(shaderDir);
}

AtmospherePrecompute::~AtmospherePrecompute()
{
    auto device = m_ctx.device();
    device.destroySampler  (m_sampler);
    device.destroyImageView(m_lutView);
    // m_lut destroyed by Image RAII
}

// ---------------------------------------------------------------------------
// Precomputation pass
// ---------------------------------------------------------------------------

void AtmospherePrecompute::runPrecompute(const std::filesystem::path& shaderDir)
{
    auto device = m_ctx.device();

    // ---- Compute descriptor set layout (binding 0 = storage image) ----
    vk::DescriptorSetLayoutBinding binding{};
    binding.setBinding        (0)
           .setDescriptorType (vk::DescriptorType::eStorageImage)
           .setDescriptorCount(1)
           .setStageFlags     (vk::ShaderStageFlagBits::eCompute);
    vk::DescriptorSetLayoutCreateInfo dslInfo{};
    dslInfo.setBindings(binding);
    auto descSetLayout = device.createDescriptorSetLayout(dslInfo);

    // ---- Compute pipeline layout (with push constants) ----
    // PC layout (48 bytes):
    //   vec4 rayleighH  (16) — xyz=β_R (per unit), w=H_R (per unit)
    //   vec4 miePack    (16) — x=β_M scat, y=β_M ext, z=H_M (per unit), w=groundRatio
    //   int  numSamples  (4)
    //   int  _pad[3]    (12)
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eCompute)
           .setOffset(0)
           .setSize(48);

    vk::PipelineLayoutCreateInfo plInfo{};
    plInfo.setSetLayouts        (descSetLayout)
          .setPushConstantRanges(pcRange);
    auto pipelineLayout = device.createPipelineLayout(plInfo);

    // ---- Load compute shader ----
    auto code = readSpirv(shaderDir / "atmosphere_transmittance.comp.spv");
    vk::ShaderModuleCreateInfo smInfo{};
    smInfo.setCode(code);
    auto shaderModule = device.createShaderModule(smInfo);

    vk::PipelineShaderStageCreateInfo stageInfo{};
    stageInfo.setStage (vk::ShaderStageFlagBits::eCompute)
             .setModule(shaderModule)
             .setPName ("main");

    vk::ComputePipelineCreateInfo cpInfo{};
    cpInfo.setStage (stageInfo)
          .setLayout(pipelineLayout);
    auto [res, compPipeline] = device.createComputePipeline(nullptr, cpInfo);
    if (res != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create atmosphere compute pipeline");

    device.destroyShaderModule(shaderModule);

    // ---- Descriptor pool + set ----
    vk::DescriptorPoolSize poolSize{};
    poolSize.setType           (vk::DescriptorType::eStorageImage)
            .setDescriptorCount(1);
    vk::DescriptorPoolCreateInfo dpInfo{};
    dpInfo.setMaxSets  (1)
          .setPoolSizes(poolSize);
    auto descPool = device.createDescriptorPool(dpInfo);

    vk::DescriptorSetAllocateInfo dsAlloc{};
    dsAlloc.setDescriptorPool(descPool)
           .setSetLayouts    (descSetLayout);
    auto descSet = device.allocateDescriptorSets(dsAlloc)[0];

    vk::DescriptorImageInfo imgInfo{};
    imgInfo.setImageLayout(vk::ImageLayout::eGeneral)
           .setImageView  (m_lutView);
    vk::WriteDescriptorSet write{};
    write.setDstSet        (descSet)
         .setDstBinding    (0)
         .setDescriptorType(vk::DescriptorType::eStorageImage)
         .setImageInfo     (imgInfo);
    device.updateDescriptorSets(write, {});

    // ---- Build push constants (local space: Ra = 1.0) ----
    float Ra = m_params.atmosphereRadius;   // km
    struct ComputePC {
        glm::vec4 rayleighH;    // xyz=β_R·Ra, w=H_R/Ra
        glm::vec4 miePack;      // x=β_M·Ra, y=β_Mext·Ra, z=H_M/Ra, w=Rg/Ra
        int       numSamples;
        int       _pad[3];
    };
    static_assert(sizeof(ComputePC) == 48);
    ComputePC pc{};
    pc.rayleighH  = glm::vec4(m_params.rayleigh * Ra, m_params.rayleighScaleH / Ra);
    pc.miePack    = glm::vec4(m_params.mieScattering * Ra,
                              m_params.mieExtinction  * Ra,
                              m_params.mieScaleH      / Ra,
                              m_params.groundRadius   / Ra);
    pc.numSamples = 500;   // high quality: atmosphere is thin so 500 is fast

    // ---- Dispatch: one-shot command buffer from a temporary pool ----
    vk::CommandPoolCreateInfo tmpPoolInfo{};
    tmpPoolInfo.setQueueFamilyIndex(m_ctx.graphicsFamily())
               .setFlags(vk::CommandPoolCreateFlagBits::eTransient);
    auto tmpPool = device.createCommandPool(tmpPoolInfo);

    vk::CommandBufferAllocateInfo cbAlloc{};
    cbAlloc.setCommandPool       (tmpPool)
           .setLevel             (vk::CommandBufferLevel::ePrimary)
           .setCommandBufferCount(1);
    auto cmd = device.allocateCommandBuffers(cbAlloc)[0];

    cmd.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

    // Transition: undefined → general (compute write)
    vk::ImageMemoryBarrier toGeneral{};
    toGeneral.setOldLayout          (vk::ImageLayout::eUndefined)
             .setNewLayout          (vk::ImageLayout::eGeneral)
             .setSrcQueueFamilyIndex(vk::QueueFamilyIgnored)
             .setDstQueueFamilyIndex(vk::QueueFamilyIgnored)
             .setImage              (m_lut.handle())
             .setSubresourceRange   ({vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1})
             .setSrcAccessMask      ({})
             .setDstAccessMask      (vk::AccessFlagBits::eShaderWrite);
    cmd.pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe,
                        vk::PipelineStageFlagBits::eComputeShader,
                        {}, {}, {}, toGeneral);

    cmd.bindPipeline(vk::PipelineBindPoint::eCompute, compPipeline);
    cmd.bindDescriptorSets(vk::PipelineBindPoint::eCompute,
                           pipelineLayout, 0, descSet, {});
    cmd.pushConstants(pipelineLayout,
                      vk::ShaderStageFlagBits::eCompute,
                      0, sizeof(ComputePC), &pc);
    // Workgroups: 256/16 = 16, 64/16 = 4
    cmd.dispatch(kLutW / 16, kLutH / 16, 1);

    // Transition: general → shader-read-only-optimal
    vk::ImageMemoryBarrier toRead{};
    toRead.setOldLayout          (vk::ImageLayout::eGeneral)
          .setNewLayout          (vk::ImageLayout::eShaderReadOnlyOptimal)
          .setSrcQueueFamilyIndex(vk::QueueFamilyIgnored)
          .setDstQueueFamilyIndex(vk::QueueFamilyIgnored)
          .setImage              (m_lut.handle())
          .setSubresourceRange   ({vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1})
          .setSrcAccessMask      (vk::AccessFlagBits::eShaderWrite)
          .setDstAccessMask      (vk::AccessFlagBits::eShaderRead);
    cmd.pipelineBarrier(vk::PipelineStageFlagBits::eComputeShader,
                        vk::PipelineStageFlagBits::eFragmentShader,
                        {}, {}, {}, toRead);

    cmd.end();

    vk::SubmitInfo si{};
    si.setCommandBuffers(cmd);
    m_ctx.graphicsQueue().submit(si, nullptr);
    m_ctx.graphicsQueue().waitIdle();

    // ---- Cleanup compute resources ----
    device.destroyCommandPool  (tmpPool);
    device.destroyDescriptorPool(descPool);
    device.destroyPipeline     (compPipeline);
    device.destroyPipelineLayout(pipelineLayout);
    device.destroyDescriptorSetLayout(descSetLayout);
}

} // namespace apeiron::render
