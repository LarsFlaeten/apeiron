#include "apeiron/render/MeshPipeline.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Pipeline.h"
#include "apeiron/render/Vertex.h"

#include <fstream>
#include <stdexcept>
#include <vector>

namespace apeiron::render {

static std::vector<uint32_t> readSpirv(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open())
        throw std::runtime_error("Cannot open shader: " + path.string());
    auto bytes = static_cast<std::streamsize>(file.tellg());
    if (bytes % 4 != 0)
        throw std::runtime_error("SPIR-V not multiple of 4: " + path.string());
    file.seekg(0);
    std::vector<uint32_t> code(bytes / 4);
    file.read(reinterpret_cast<char*>(code.data()), bytes);
    return code;
}

vk::ShaderModule MeshPipeline::loadShader(const std::filesystem::path& spvPath)
{
    auto code = readSpirv(spvPath);
    vk::ShaderModuleCreateInfo info{};
    info.setCode(code);
    return m_ctx.device().createShaderModule(info);
}

void MeshPipeline::createMaterialDescLayout()
{
    // Binding 0: MaterialUBO (vertex + fragment access).
    vk::DescriptorSetLayoutBinding uboBinding{};
    uboBinding.setBinding        (0)
              .setDescriptorType (vk::DescriptorType::eUniformBuffer)
              .setDescriptorCount(1)
              .setStageFlags     (vk::ShaderStageFlagBits::eVertex |
                                  vk::ShaderStageFlagBits::eFragment);

    // Binding 1: albedo texture sampler.
    vk::DescriptorSetLayoutBinding samplerBinding{};
    samplerBinding.setBinding        (1)
                  .setDescriptorType (vk::DescriptorType::eCombinedImageSampler)
                  .setDescriptorCount(1)
                  .setStageFlags     (vk::ShaderStageFlagBits::eFragment);

    std::array bindings{uboBinding, samplerBinding};
    vk::DescriptorSetLayoutCreateInfo dslInfo{};
    dslInfo.setBindings(bindings);
    m_matDescSetLayout = m_ctx.device().createDescriptorSetLayout(dslInfo);

    // Descriptor pool — allow up to 256 material descriptor sets.
    // (Orion has ~10 materials; 256 leaves ample headroom for future models.)
    std::array<vk::DescriptorPoolSize, 2> poolSizes{};
    poolSizes[0].setType(vk::DescriptorType::eUniformBuffer)        .setDescriptorCount(256);
    poolSizes[1].setType(vk::DescriptorType::eCombinedImageSampler) .setDescriptorCount(256);
    vk::DescriptorPoolCreateInfo poolInfo{};
    poolInfo.setMaxSets   (256)
            .setPoolSizes (poolSizes);
    m_matDescPool = m_ctx.device().createDescriptorPool(poolInfo);
}

vk::DescriptorSet MeshPipeline::allocateMaterialDescSet(vk::Buffer     ubo,
                                                         vk::DeviceSize uboSize,
                                                         vk::ImageView  imageView,
                                                         vk::Sampler    sampler) const
{
    vk::DescriptorSetAllocateInfo allocInfo{};
    allocInfo.setDescriptorPool    (m_matDescPool)
             .setSetLayouts        (m_matDescSetLayout);
    auto sets = m_ctx.device().allocateDescriptorSets(allocInfo);
    vk::DescriptorSet ds = sets[0];

    vk::DescriptorBufferInfo bufInfo{};
    bufInfo.setBuffer(ubo).setOffset(0).setRange(uboSize);

    vk::DescriptorImageInfo imgInfo{};
    imgInfo.setImageLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
           .setImageView  (imageView)
           .setSampler    (sampler);

    std::array<vk::WriteDescriptorSet, 2> writes{};
    writes[0].setDstSet         (ds)
             .setDstBinding     (0)
             .setDescriptorType (vk::DescriptorType::eUniformBuffer)
             .setBufferInfo     (bufInfo);
    writes[1].setDstSet         (ds)
             .setDstBinding     (1)
             .setDescriptorType (vk::DescriptorType::eCombinedImageSampler)
             .setImageInfo      (imgInfo);

    m_ctx.device().updateDescriptorSets(writes, {});
    return ds;
}

MeshPipeline::MeshPipeline(const Context&               ctx,
                             const Pipeline&              mainPipeline,
                             const std::filesystem::path& shaderDir)
    : m_ctx(ctx)
    , m_renderPass(mainPipeline.renderPass())  // borrow — not owned
{
    // Material descriptor set layout + pool.
    createMaterialDescLayout();

    // Push-constant range: MeshPushConstants = 160 bytes.
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eVertex |
                          vk::ShaderStageFlagBits::eFragment)
           .setOffset(0)
           .setSize(sizeof(MeshPushConstants));

    // Set 0 = material descriptor (UBO + albedo sampler).
    vk::PipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.setSetLayouts        (m_matDescSetLayout)
              .setPushConstantRanges(pcRange);
    m_layout = m_ctx.device().createPipelineLayout(layoutInfo);

    // Shaders.
    auto vert = loadShader(shaderDir / "mesh.vert.spv");
    auto frag = loadShader(shaderDir / "mesh.frag.spv");

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
    rasterizer.setPolygonMode(vk::PolygonMode::eFill)
              .setCullMode   (vk::CullModeFlagBits::eFront)
              .setFrontFace  (vk::FrontFace::eClockwise)
              .setLineWidth  (1.0f);

    vk::PipelineMultisampleStateCreateInfo multisampling{};
    multisampling.setRasterizationSamples(vk::SampleCountFlagBits::e1);

    // Additive alpha blend for emissive plumes; opaque for hull.
    // We use a single pipeline with additive blending — hull meshes have alpha=1
    // so the blend is effectively opaque; emissive plumes add to HDR.
    vk::PipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment
        .setBlendEnable        (vk::True)
        .setSrcColorBlendFactor(vk::BlendFactor::eSrcAlpha)
        .setDstColorBlendFactor(vk::BlendFactor::eOneMinusSrcAlpha)
        .setColorBlendOp       (vk::BlendOp::eAdd)
        .setSrcAlphaBlendFactor(vk::BlendFactor::eOne)
        .setDstAlphaBlendFactor(vk::BlendFactor::eOneMinusSrcAlpha)
        .setAlphaBlendOp       (vk::BlendOp::eAdd)
        .setColorWriteMask(
            vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
            vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);

    vk::PipelineColorBlendStateCreateInfo colorBlend{};
    colorBlend.setAttachments(blendAttachment);

    vk::PipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.setDepthTestEnable      (vk::True)
                .setDepthWriteEnable     (vk::True)
                .setDepthCompareOp       (vk::CompareOp::eLess)
                .setDepthBoundsTestEnable(vk::False)
                .setStencilTestEnable    (vk::False);

    vk::GraphicsPipelineCreateInfo info{};
    info.setStages            (stages)
        .setPVertexInputState (&vertexInput)
        .setPInputAssemblyState(&inputAssembly)
        .setPViewportState    (&viewportState)
        .setPRasterizationState(&rasterizer)
        .setPMultisampleState (&multisampling)
        .setPColorBlendState  (&colorBlend)
        .setPDepthStencilState(&depthStencil)
        .setPDynamicState     (&dynamicState)
        .setLayout            (m_layout)
        .setRenderPass        (m_renderPass)
        .setSubpass           (0);

    auto [result, pipeline] = m_ctx.device().createGraphicsPipeline(nullptr, info);
    if (result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create mesh pipeline");
    m_pipeline = pipeline;

    // Double-sided variant: identical but with no culling.
    vk::PipelineRasterizationStateCreateInfo rasterizerDS = rasterizer;
    rasterizerDS.setCullMode(vk::CullModeFlagBits::eNone);
    info.setPRasterizationState(&rasterizerDS);
    auto [resultDS, pipelineDS] = m_ctx.device().createGraphicsPipeline(nullptr, info);
    if (resultDS != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create double-sided mesh pipeline");
    m_pipelineDS = pipelineDS;

    m_ctx.device().destroyShaderModule(vert);
    m_ctx.device().destroyShaderModule(frag);

    // Plume pipeline: dedicated shaders, additive blend, no culling.
    auto plumeVert = loadShader(shaderDir / "plume.vert.spv");
    auto plumeFrag = loadShader(shaderDir / "plume.frag.spv");

    stages[0].setModule(plumeVert);
    stages[1].setModule(plumeFrag);

    vk::PipelineRasterizationStateCreateInfo rasterizerPlume = rasterizer;
    rasterizerPlume.setCullMode(vk::CullModeFlagBits::eNone);

    vk::PipelineColorBlendAttachmentState additiveBlend{};
    additiveBlend
        .setBlendEnable        (vk::True)
        .setSrcColorBlendFactor(vk::BlendFactor::eSrcAlpha)
        .setDstColorBlendFactor(vk::BlendFactor::eOne)       // additive
        .setColorBlendOp       (vk::BlendOp::eAdd)
        .setSrcAlphaBlendFactor(vk::BlendFactor::eOne)
        .setDstAlphaBlendFactor(vk::BlendFactor::eOne)
        .setAlphaBlendOp       (vk::BlendOp::eAdd)
        .setColorWriteMask(
            vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
            vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);

    vk::PipelineColorBlendStateCreateInfo additiveColorBlend{};
    additiveColorBlend.setAttachments(additiveBlend);

    // Plumes don't write depth (additive transparent — depth read only).
    vk::PipelineDepthStencilStateCreateInfo plumeDepth = depthStencil;
    plumeDepth.setDepthWriteEnable(vk::False);

    info.setPRasterizationState(&rasterizerPlume)
        .setPColorBlendState   (&additiveColorBlend)
        .setPDepthStencilState (&plumeDepth);

    auto [resultP, pipelinePlume] = m_ctx.device().createGraphicsPipeline(nullptr, info);
    if (resultP != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create plume pipeline");
    m_pipelinePlume = pipelinePlume;

    m_ctx.device().destroyShaderModule(plumeVert);
    m_ctx.device().destroyShaderModule(plumeFrag);
}

MeshPipeline::~MeshPipeline()
{
    m_ctx.device().destroyPipeline         (m_pipeline);
    m_ctx.device().destroyPipeline         (m_pipelineDS);
    m_ctx.device().destroyPipeline         (m_pipelinePlume);
    m_ctx.device().destroyPipelineLayout   (m_layout);
    m_ctx.device().destroyDescriptorPool   (m_matDescPool);
    m_ctx.device().destroyDescriptorSetLayout(m_matDescSetLayout);
    // m_renderPass is borrowed from Pipeline — not destroyed here
}

} // namespace apeiron::render
