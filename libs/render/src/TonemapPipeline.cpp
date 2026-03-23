#include "apeiron/render/TonemapPipeline.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Swapchain.h"

#include <fstream>
#include <stdexcept>
#include <vector>

namespace {

static std::vector<uint32_t> readSpirv(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open())
        throw std::runtime_error("Cannot open shader: " + path.string());
    auto bytes = static_cast<std::streamsize>(file.tellg());
    if (bytes % 4 != 0)
        throw std::runtime_error("SPIR-V size not a multiple of 4: " + path.string());
    file.seekg(0);
    std::vector<uint32_t> code(bytes / 4);
    file.read(reinterpret_cast<char*>(code.data()), bytes);
    return code;
}

} // namespace

namespace apeiron::render {

TonemapPipeline::TonemapPipeline(const Context&               ctx,
                                 const Swapchain&             swapchain,
                                 const std::filesystem::path& shaderDir)
    : m_ctx(ctx)
{
    auto device = ctx.device();

    // ----- Render pass — writes to the swapchain (display) format -----
    vk::AttachmentDescription colorAttach{};
    colorAttach.setFormat        (swapchain.imageFormat())
               .setSamples       (vk::SampleCountFlagBits::e1)
               .setLoadOp        (vk::AttachmentLoadOp::eDontCare)
               .setStoreOp       (vk::AttachmentStoreOp::eStore)
               .setInitialLayout (vk::ImageLayout::eUndefined)
               .setFinalLayout   (vk::ImageLayout::ePresentSrcKHR);

    vk::AttachmentReference colorRef{0, vk::ImageLayout::eColorAttachmentOptimal};

    vk::SubpassDescription subpass{};
    subpass.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
           .setColorAttachments (colorRef);

    vk::SubpassDependency dep{};
    dep.setSrcSubpass   (vk::SubpassExternal)
       .setDstSubpass   (0)
       .setSrcStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
       .setDstStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
       .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentWrite);

    vk::RenderPassCreateInfo rpInfo{};
    rpInfo.setAttachments (colorAttach)
          .setSubpasses   (subpass)
          .setDependencies(dep);
    m_renderPass = device.createRenderPass(rpInfo);

    // ----- Descriptor set layout: one sampler (HDR texture) -----
    vk::DescriptorSetLayoutBinding binding{};
    binding.setBinding        (0)
           .setDescriptorType (vk::DescriptorType::eCombinedImageSampler)
           .setDescriptorCount(1)
           .setStageFlags     (vk::ShaderStageFlagBits::eFragment);
    vk::DescriptorSetLayoutCreateInfo dslInfo{};
    dslInfo.setBindings(binding);
    m_descriptorSetLayout = device.createDescriptorSetLayout(dslInfo);

    // ----- Pipeline layout: set 0 = HDR sampler, push constant = exposure -----
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eFragment)
           .setOffset    (0)
           .setSize      (sizeof(float));
    vk::PipelineLayoutCreateInfo plInfo{};
    plInfo.setSetLayouts       (m_descriptorSetLayout)
          .setPushConstantRanges(pcRange);
    m_layout = device.createPipelineLayout(plInfo);

    // ----- Shaders -----
    auto makeModule = [&](const std::filesystem::path& spvPath) {
        auto code = readSpirv(spvPath);
        vk::ShaderModuleCreateInfo info{};
        info.setCode(code);
        return device.createShaderModule(info);
    };
    auto vertMod = makeModule(shaderDir / "tonemap.vert.spv");
    auto fragMod = makeModule(shaderDir / "tonemap.frag.spv");

    std::array<vk::PipelineShaderStageCreateInfo, 2> stages{};
    stages[0].setStage(vk::ShaderStageFlagBits::eVertex)  .setModule(vertMod).setPName("main");
    stages[1].setStage(vk::ShaderStageFlagBits::eFragment).setModule(fragMod).setPName("main");

    // Fullscreen triangle: no vertex buffer.
    vk::PipelineVertexInputStateCreateInfo   viState{};
    vk::PipelineInputAssemblyStateCreateInfo iaState{};
    iaState.setTopology(vk::PrimitiveTopology::eTriangleList);

    auto ext = swapchain.extent();
    vk::Viewport vp{0.0f, 0.0f,
                    static_cast<float>(ext.width), static_cast<float>(ext.height),
                    0.0f, 1.0f};
    vk::Rect2D scissor{{0, 0}, ext};
    vk::PipelineViewportStateCreateInfo vpState{};
    vpState.setViewports(vp).setScissors(scissor);

    vk::PipelineRasterizationStateCreateInfo rsState{};
    rsState.setPolygonMode(vk::PolygonMode::eFill)
           .setCullMode   (vk::CullModeFlagBits::eNone)
           .setLineWidth  (1.0f);

    vk::PipelineMultisampleStateCreateInfo msState{};
    msState.setRasterizationSamples(vk::SampleCountFlagBits::e1);

    vk::PipelineColorBlendAttachmentState blendAttach{};
    blendAttach.setColorWriteMask(
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
        vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);
    vk::PipelineColorBlendStateCreateInfo cbState{};
    cbState.setAttachments(blendAttach);

    vk::GraphicsPipelineCreateInfo gpInfo{};
    gpInfo.setStages            (stages)
          .setPVertexInputState  (&viState)
          .setPInputAssemblyState(&iaState)
          .setPViewportState     (&vpState)
          .setPRasterizationState(&rsState)
          .setPMultisampleState  (&msState)
          .setPColorBlendState   (&cbState)
          .setLayout             (m_layout)
          .setRenderPass         (m_renderPass);

    auto [result, pipeline] = device.createGraphicsPipeline(nullptr, gpInfo);
    if (result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create tonemap pipeline");
    m_pipeline = pipeline;

    device.destroyShaderModule(vertMod);
    device.destroyShaderModule(fragMod);
}

TonemapPipeline::~TonemapPipeline()
{
    auto device = m_ctx.device();
    device.destroyPipeline           (m_pipeline);
    device.destroyPipelineLayout     (m_layout);
    device.destroyDescriptorSetLayout(m_descriptorSetLayout);
    device.destroyRenderPass         (m_renderPass);
}

} // namespace apeiron::render
