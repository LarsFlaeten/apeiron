#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Swapchain.h"
#include "apeiron/render/Vertex.h"

#include <glm/glm.hpp>
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
// buildPipeline — creates one vk::Pipeline variant, sharing the already-built
// render pass, layout, shaders, and all fixed-function state except the
// rasterizer polygon mode.
// ---------------------------------------------------------------------------

vk::Pipeline Pipeline::buildPipeline(vk::PolygonMode  polygonMode,
                                      vk::ShaderModule vert,
                                      vk::ShaderModule frag)
{
    std::array<vk::PipelineShaderStageCreateInfo, 2> stages{};
    stages[0].setStage(vk::ShaderStageFlagBits::eVertex)  .setModule(vert).setPName("main");
    stages[1].setStage(vk::ShaderStageFlagBits::eFragment).setModule(frag).setPName("main");

    auto binding    = Vertex::bindingDescription();
    auto attributes = Vertex::attributeDescriptions();
    vk::PipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.setVertexBindingDescriptions  (binding)
               .setVertexAttributeDescriptions(attributes);

    vk::PipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.setTopology(vk::PrimitiveTopology::eTriangleList);

    std::array dynStates{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
    vk::PipelineDynamicStateCreateInfo dynamicState{};
    dynamicState.setDynamicStates(dynStates);

    vk::PipelineViewportStateCreateInfo viewportState{};
    viewportState.setViewportCount(1).setScissorCount(1);

    vk::PipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.setPolygonMode(polygonMode)              // ← the only difference
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

    vk::PipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.setDepthTestEnable       (vk::True)
                .setDepthWriteEnable      (vk::True)
                .setDepthCompareOp        (vk::CompareOp::eLess)
                .setDepthBoundsTestEnable (vk::False)
                .setStencilTestEnable     (vk::False);

    vk::GraphicsPipelineCreateInfo info{};
    info.setStages             (stages)
        .setPVertexInputState  (&vertexInput)
        .setPInputAssemblyState(&inputAssembly)
        .setPViewportState     (&viewportState)
        .setPRasterizationState(&rasterizer)
        .setPMultisampleState  (&multisampling)
        .setPColorBlendState   (&colorBlend)
        .setPDepthStencilState (&depthStencil)
        .setPDynamicState      (&dynamicState)
        .setLayout             (m_layout)
        .setRenderPass         (m_renderPass)
        .setSubpass            (0);

    auto [result, pipeline] = m_ctx.device().createGraphicsPipeline(nullptr, info);
    if (result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create graphics pipeline");
    return pipeline;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

Pipeline::Pipeline(const Context&               ctx,
                   const Swapchain&             swapchain,
                   const std::filesystem::path& shaderDir)
    : m_ctx(ctx)
{
    auto device = ctx.device();

    // ----- Render pass -----
    // attachment 0 — HDR colour (RGBA16F); transitions to ShaderReadOnly so
    // the tonemap pass can sample it without an extra explicit barrier.
    vk::AttachmentDescription colorAttachment{};
    colorAttachment
        .setFormat        (vk::Format::eR16G16B16A16Sfloat)
        .setSamples       (vk::SampleCountFlagBits::e1)
        .setLoadOp        (vk::AttachmentLoadOp::eClear)
        .setStoreOp       (vk::AttachmentStoreOp::eStore)
        .setStencilLoadOp (vk::AttachmentLoadOp::eDontCare)
        .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
        .setInitialLayout (vk::ImageLayout::eUndefined)
        .setFinalLayout   (vk::ImageLayout::eShaderReadOnlyOptimal);

    // attachment 1 — depth (cleared to 1.0, not needed after the pass)
    vk::AttachmentDescription depthAttachment{};
    depthAttachment
        .setFormat        (swapchain.depthFormat())
        .setSamples       (vk::SampleCountFlagBits::e1)
        .setLoadOp        (vk::AttachmentLoadOp::eClear)
        .setStoreOp       (vk::AttachmentStoreOp::eDontCare)
        .setStencilLoadOp (vk::AttachmentLoadOp::eDontCare)
        .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
        .setInitialLayout (vk::ImageLayout::eUndefined)
        .setFinalLayout   (vk::ImageLayout::eDepthStencilAttachmentOptimal);

    vk::AttachmentReference colorRef{};
    colorRef.setAttachment(0).setLayout(vk::ImageLayout::eColorAttachmentOptimal);

    vk::AttachmentReference depthRef{};
    depthRef.setAttachment(1).setLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

    vk::SubpassDescription subpass{};
    subpass.setPipelineBindPoint      (vk::PipelineBindPoint::eGraphics)
           .setColorAttachments       (colorRef)
           .setPDepthStencilAttachment(&depthRef);

    // dep[0]: wait for prior colour/depth writes before this subpass starts.
    // dep[1]: after the subpass, make the HDR colour available for the
    //         tonemap fragment shader without a separate explicit barrier.
    std::array<vk::SubpassDependency, 2> deps{};
    deps[0].setSrcSubpass   (vk::SubpassExternal)
           .setDstSubpass   (0)
           .setSrcStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput |
                             vk::PipelineStageFlagBits::eEarlyFragmentTests)
           .setSrcAccessMask({})
           .setDstStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput |
                             vk::PipelineStageFlagBits::eEarlyFragmentTests)
           .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentWrite |
                             vk::AccessFlagBits::eDepthStencilAttachmentWrite);
    deps[1].setSrcSubpass   (0)
           .setDstSubpass   (vk::SubpassExternal)
           .setSrcStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
           .setSrcAccessMask(vk::AccessFlagBits::eColorAttachmentWrite)
           .setDstStageMask (vk::PipelineStageFlagBits::eFragmentShader)
           .setDstAccessMask(vk::AccessFlagBits::eShaderRead);

    std::array<vk::AttachmentDescription, 2> attachments{colorAttachment, depthAttachment};
    vk::RenderPassCreateInfo rpInfo{};
    rpInfo.setAttachments(attachments).setSubpasses(subpass).setDependencies(deps);
    m_renderPass = device.createRenderPass(rpInfo);

    // ----- Descriptor set layout (set 0): diffuse, specular, normals, clouds -----
    std::array<vk::DescriptorSetLayoutBinding, 4> samplerBindings{};
    for (uint32_t i = 0; i < 4; ++i) {
        samplerBindings[i]
            .setBinding        (i)
            .setDescriptorType (vk::DescriptorType::eCombinedImageSampler)
            .setDescriptorCount(1)
            .setStageFlags     (vk::ShaderStageFlagBits::eFragment);
    }
    vk::DescriptorSetLayoutCreateInfo dslInfo{};
    dslInfo.setBindings(samplerBindings);
    m_descriptorSetLayout = device.createDescriptorSetLayout(dslInfo);

    // ----- Pipeline layout (shared by both pipelines) -----
    // 128 bytes: mat4 mvp + mat3 normalMat (as 3×vec4) + vec3 sunDir (+ pad).
    // sunDir is read in the fragment shader, so both stages need access.
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eVertex |
                          vk::ShaderStageFlagBits::eFragment)
           .setOffset(0)
           .setSize(sizeof(glm::mat4) * 2);

    vk::PipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.setSetLayouts        (m_descriptorSetLayout)
              .setPushConstantRanges(pcRange);
    m_layout = device.createPipelineLayout(layoutInfo);

    // ----- Shaders (loaded once, shared by both pipelines) -----
    auto vertModule = loadShader(shaderDir / "triangle.vert.spv");
    auto fragModule = loadShader(shaderDir / "triangle.frag.spv");

    m_fillPipeline      = buildPipeline(vk::PolygonMode::eFill, vertModule, fragModule);
    m_wireframePipeline = buildPipeline(vk::PolygonMode::eLine, vertModule, fragModule);

    device.destroyShaderModule(vertModule);
    device.destroyShaderModule(fragModule);
}

Pipeline::~Pipeline()
{
    auto device = m_ctx.device();
    device.destroyPipeline           (m_wireframePipeline);
    device.destroyPipeline           (m_fillPipeline);
    device.destroyPipelineLayout     (m_layout);
    device.destroyRenderPass         (m_renderPass);
    device.destroyDescriptorSetLayout(m_descriptorSetLayout);
}

} // namespace apeiron::render
