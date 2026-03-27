#include "apeiron/render/AtmospherePipeline.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Vertex.h"

#include <glm/glm.hpp>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace apeiron::render {

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

AtmospherePipeline::AtmospherePipeline(const Context&               ctx,
                                        vk::RenderPass               renderPass,
                                        const std::filesystem::path& shaderDir)
    : m_ctx(ctx)
{
    auto device = ctx.device();

    // ---- Descriptor set layout: one transmittance LUT sampler ----
    vk::DescriptorSetLayoutBinding samplerBinding{};
    samplerBinding.setBinding        (0)
                  .setDescriptorType (vk::DescriptorType::eCombinedImageSampler)
                  .setDescriptorCount(1)
                  .setStageFlags     (vk::ShaderStageFlagBits::eFragment);
    vk::DescriptorSetLayoutCreateInfo dslInfo{};
    dslInfo.setBindings(samplerBinding);
    m_descSetLayout = device.createDescriptorSetLayout(dslInfo);

    // ---- Pipeline layout: 128-byte push constants (vert + frag) ----
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eVertex |
                          vk::ShaderStageFlagBits::eFragment)
           .setOffset(0)
           .setSize(sizeof(glm::mat4) * 2);   // 128 bytes

    vk::PipelineLayoutCreateInfo plInfo{};
    plInfo.setSetLayouts        (m_descSetLayout)
          .setPushConstantRanges(pcRange);
    m_layout = device.createPipelineLayout(plInfo);

    // ---- Load shaders ----
    auto vertCode = readSpirv(shaderDir / "atmosphere.vert.spv");
    auto fragCode = readSpirv(shaderDir / "atmosphere.frag.spv");

    vk::ShaderModuleCreateInfo smInfo{};
    smInfo.setCode(vertCode);
    auto vertMod = device.createShaderModule(smInfo);
    smInfo.setCode(fragCode);
    auto fragMod = device.createShaderModule(smInfo);

    std::array<vk::PipelineShaderStageCreateInfo, 2> stages{};
    stages[0].setStage(vk::ShaderStageFlagBits::eVertex)  .setModule(vertMod).setPName("main");
    stages[1].setStage(vk::ShaderStageFlagBits::eFragment).setModule(fragMod).setPName("main");

    // ---- Vertex input (same layout as body meshes) ----
    auto binding    = Vertex::bindingDescription();
    auto attributes = Vertex::attributeDescriptions();
    vk::PipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.setVertexBindingDescriptions  (binding)
               .setVertexAttributeDescriptions(attributes);

    vk::PipelineInputAssemblyStateCreateInfo ia{};
    ia.setTopology(vk::PrimitiveTopology::eTriangleList);

    std::array dynStates{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
    vk::PipelineDynamicStateCreateInfo dynState{};
    dynState.setDynamicStates(dynStates);

    vk::PipelineViewportStateCreateInfo vpState{};
    vpState.setViewportCount(1).setScissorCount(1);

    // Back-face culling: only the hemisphere facing the camera is rendered.
    // The fragment shader ray-marches through the full atmosphere from the
    // front face entry point, so only one face per ray is needed.
    vk::PipelineRasterizationStateCreateInfo raster{};
    raster.setPolygonMode(vk::PolygonMode::eFill)
          .setCullMode   (vk::CullModeFlagBits::eBack)
          .setFrontFace  (vk::FrontFace::eClockwise)
          .setLineWidth  (1.0f);

    vk::PipelineMultisampleStateCreateInfo ms{};
    ms.setRasterizationSamples(vk::SampleCountFlagBits::e1);

    // Blend: result = L_inscatter + background × transmittance
    //   src colour factor = One,      dst colour factor = SrcAlpha (= transmittance)
    vk::PipelineColorBlendAttachmentState blendAtt{};
    blendAtt.setBlendEnable        (vk::True)
            .setSrcColorBlendFactor(vk::BlendFactor::eOne)
            .setDstColorBlendFactor(vk::BlendFactor::eSrcAlpha)
            .setColorBlendOp       (vk::BlendOp::eAdd)
            .setSrcAlphaBlendFactor(vk::BlendFactor::eZero)
            .setDstAlphaBlendFactor(vk::BlendFactor::eOne)
            .setAlphaBlendOp       (vk::BlendOp::eAdd)
            .setColorWriteMask(vk::ColorComponentFlagBits::eR |
                               vk::ColorComponentFlagBits::eG |
                               vk::ColorComponentFlagBits::eB |
                               vk::ColorComponentFlagBits::eA);
    vk::PipelineColorBlendStateCreateInfo blend{};
    blend.setAttachments(blendAtt);

    // Depth test on (don't draw atmosphere behind the planet surface);
    // depth write off (atmosphere is transparent).
    vk::PipelineDepthStencilStateCreateInfo ds{};
    ds.setDepthTestEnable      (vk::True)
      .setDepthWriteEnable     (vk::False)
      .setDepthCompareOp       (vk::CompareOp::eLess)
      .setDepthBoundsTestEnable(vk::False)
      .setStencilTestEnable    (vk::False);

    vk::GraphicsPipelineCreateInfo gpInfo{};
    gpInfo.setStages              (stages)
          .setPVertexInputState   (&vertexInput)
          .setPInputAssemblyState (&ia)
          .setPViewportState      (&vpState)
          .setPRasterizationState (&raster)
          .setPMultisampleState   (&ms)
          .setPColorBlendState    (&blend)
          .setPDepthStencilState  (&ds)
          .setPDynamicState       (&dynState)
          .setLayout              (m_layout)
          .setRenderPass          (renderPass)
          .setSubpass             (0);

    auto [res, pl] = device.createGraphicsPipeline(nullptr, gpInfo);
    if (res != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create atmosphere graphics pipeline");
    m_pipeline = pl;

    device.destroyShaderModule(vertMod);
    device.destroyShaderModule(fragMod);
}

AtmospherePipeline::~AtmospherePipeline()
{
    auto device = m_ctx.device();
    device.destroyPipeline          (m_pipeline);
    device.destroyPipelineLayout    (m_layout);
    device.destroyDescriptorSetLayout(m_descSetLayout);
}

} // namespace apeiron::render
