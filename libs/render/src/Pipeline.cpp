#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Vertex.h"

#include <fstream>
#include <stdexcept>
#include <vector>

namespace apeiron::render {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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

vk::ShaderModule Pipeline::loadShader(const std::filesystem::path& spvPath)
{
    auto code = readSpirv(spvPath);
    vk::ShaderModuleCreateInfo info{};
    info.setCode(code);
    return m_ctx.device().createShaderModule(info);
}

// ---------------------------------------------------------------------------
// Pipeline
// ---------------------------------------------------------------------------

Pipeline::Pipeline(const Context&               ctx,
                   const Swapchain&             swapchain,
                   const std::filesystem::path& shaderDir)
    : m_ctx(ctx)
{
    auto device = ctx.device();

    // ----- Render pass -----
    // Single colour attachment, cleared on load, stored on store,
    // transitioning to present layout at the end of the pass.
    vk::AttachmentDescription colorAttachment{};
    colorAttachment
        .setFormat        (swapchain.imageFormat())
        .setSamples       (vk::SampleCountFlagBits::e1)
        .setLoadOp        (vk::AttachmentLoadOp::eClear)
        .setStoreOp       (vk::AttachmentStoreOp::eStore)
        .setStencilLoadOp (vk::AttachmentLoadOp::eDontCare)
        .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
        .setInitialLayout (vk::ImageLayout::eUndefined)
        .setFinalLayout   (vk::ImageLayout::ePresentSrcKHR);

    vk::AttachmentReference colorRef{};
    colorRef.setAttachment(0)
            .setLayout(vk::ImageLayout::eColorAttachmentOptimal);

    vk::SubpassDescription subpass{};
    subpass.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
           .setColorAttachments(colorRef);

    // Ensure the colour output stage has finished before the driver
    // hands the image to the presentation engine.
    vk::SubpassDependency dep{};
    dep.setSrcSubpass   (vk::SubpassExternal)
       .setDstSubpass   (0)
       .setSrcStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
       .setSrcAccessMask({})
       .setDstStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
       .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentWrite);

    vk::RenderPassCreateInfo rpInfo{};
    rpInfo.setAttachments(colorAttachment)
          .setSubpasses  (subpass)
          .setDependencies(dep);
    m_renderPass = device.createRenderPass(rpInfo);

    // ----- Shaders -----
    auto vertModule = loadShader(shaderDir / "triangle.vert.spv");
    auto fragModule = loadShader(shaderDir / "triangle.frag.spv");

    std::array<vk::PipelineShaderStageCreateInfo, 2> stages{};
    stages[0].setStage(vk::ShaderStageFlagBits::eVertex)
             .setModule(vertModule)
             .setPName("main");
    stages[1].setStage(vk::ShaderStageFlagBits::eFragment)
             .setModule(fragModule)
             .setPName("main");

    // ----- Fixed-function state -----

    auto binding    = Vertex::bindingDescription();
    auto attributes = Vertex::attributeDescriptions();
    vk::PipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.setVertexBindingDescriptions  (binding)
               .setVertexAttributeDescriptions(attributes);

    vk::PipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.setTopology(vk::PrimitiveTopology::eTriangleList);

    // Viewport and scissor set dynamically so we don't need to rebuild
    // the pipeline on window resize.
    std::array dynStates{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
    vk::PipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.setDynamicStates(dynStates);

    vk::PipelineViewportStateCreateInfo viewportState{};
    viewportState.setViewportCount(1).setScissorCount(1);

    vk::PipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.setPolygonMode(vk::PolygonMode::eFill)
              .setCullMode   (vk::CullModeFlagBits::eNone)
              .setFrontFace  (vk::FrontFace::eClockwise)
              .setLineWidth  (1.0f);

    vk::PipelineMultisampleStateCreateInfo multisampling{};
    multisampling.setRasterizationSamples(vk::SampleCountFlagBits::e1);

    vk::PipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.setColorWriteMask(
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
        vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);

    vk::PipelineColorBlendStateCreateInfo colorBlend{};
    colorBlend.setAttachments(blendAttachment);

    // ----- Push constants -----
    // One float (rotation angle, radians) visible in the vertex stage.
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eVertex)
           .setOffset(0)
           .setSize(sizeof(float));

    vk::PipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.setPushConstantRanges(pcRange);
    m_layout = device.createPipelineLayout(layoutInfo);

    // ----- Graphics pipeline -----
    vk::GraphicsPipelineCreateInfo pipelineInfo{};
    pipelineInfo
        .setStages             (stages)
        .setPVertexInputState  (&vertexInput)
        .setPInputAssemblyState(&inputAssembly)
        .setPViewportState     (&viewportState)
        .setPRasterizationState(&rasterizer)
        .setPMultisampleState  (&multisampling)
        .setPColorBlendState   (&colorBlend)
        .setPDynamicState      (&dynamicState)
        .setLayout             (m_layout)
        .setRenderPass         (m_renderPass)
        .setSubpass            (0);

    auto [result, pipeline] = device.createGraphicsPipeline(nullptr, pipelineInfo);
    if (result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create graphics pipeline");
    m_pipeline = pipeline;

    // Shader modules baked into the pipeline; no longer needed.
    device.destroyShaderModule(vertModule);
    device.destroyShaderModule(fragModule);
}

Pipeline::~Pipeline()
{
    auto device = m_ctx.device();
    device.destroyPipeline      (m_pipeline);
    device.destroyPipelineLayout(m_layout);
    device.destroyRenderPass    (m_renderPass);
}

} // namespace apeiron::render
