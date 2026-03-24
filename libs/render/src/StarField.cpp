#include "apeiron/render/StarField.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/GpuAllocator.h"
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

StarField::StarField(const Context&               ctx,
                     GpuAllocator&                allocator,
                     vk::RenderPass               hdrRenderPass,
                     const Swapchain&             swapchain,
                     const std::filesystem::path& shaderDir,
                     std::vector<StarVertex>       stars)
    : m_ctx(ctx)
    , m_starCount(static_cast<uint32_t>(stars.size()))
{
    auto device = ctx.device();

    // ----- Vertex buffer -----
    vk::DeviceSize vbSize = stars.size() * sizeof(StarVertex);

    Buffer staging(allocator.handle(), vbSize,
                   vk::BufferUsageFlagBits::eTransferSrc,
                   VMA_MEMORY_USAGE_AUTO,
                   VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT);
    staging.upload(stars.data(), vbSize);

    m_vertexBuffer = Buffer(allocator.handle(), vbSize,
                            vk::BufferUsageFlagBits::eVertexBuffer |
                            vk::BufferUsageFlagBits::eTransferDst,
                            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);

    allocator.immediateSubmit([&](vk::CommandBuffer cmd) {
        cmd.copyBuffer(staging.handle(), m_vertexBuffer.handle(),
                       vk::BufferCopy{0, 0, vbSize});
    });

    // ----- Pipeline layout (push constant: mat4 vpRot) -----
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eVertex)
           .setOffset    (0)
           .setSize      (sizeof(glm::mat4));
    vk::PipelineLayoutCreateInfo plInfo{};
    plInfo.setPushConstantRanges(pcRange);
    m_layout = device.createPipelineLayout(plInfo);

    // ----- Shaders -----
    auto makeModule = [&](const std::filesystem::path& spv) {
        auto code = readSpirv(spv);
        vk::ShaderModuleCreateInfo info{};
        info.setCode(code);
        return device.createShaderModule(info);
    };
    auto vertMod = makeModule(shaderDir / "star.vert.spv");
    auto fragMod = makeModule(shaderDir / "star.frag.spv");

    std::array<vk::PipelineShaderStageCreateInfo, 2> stages{};
    stages[0].setStage(vk::ShaderStageFlagBits::eVertex)  .setModule(vertMod).setPName("main");
    stages[1].setStage(vk::ShaderStageFlagBits::eFragment).setModule(fragMod).setPName("main");

    // Vertex input: binding 0 — StarVertex (vec3 direction, vec3 color)
    vk::VertexInputBindingDescription binding{};
    binding.setBinding  (0)
           .setStride   (sizeof(StarVertex))
           .setInputRate(vk::VertexInputRate::eVertex);

    std::array<vk::VertexInputAttributeDescription, 2> attrs{};
    attrs[0].setLocation(0).setBinding(0).setFormat(vk::Format::eR32G32B32Sfloat).setOffset(0);
    attrs[1].setLocation(1).setBinding(0).setFormat(vk::Format::eR32G32B32Sfloat).setOffset(12);

    vk::PipelineVertexInputStateCreateInfo viState{};
    viState.setVertexBindingDescriptions  (binding)
           .setVertexAttributeDescriptions(attrs);

    vk::PipelineInputAssemblyStateCreateInfo iaState{};
    iaState.setTopology(vk::PrimitiveTopology::ePointList);

    std::array dynStates{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
    vk::PipelineDynamicStateCreateInfo dynState{};
    dynState.setDynamicStates(dynStates);

    vk::PipelineViewportStateCreateInfo vpState{};
    vpState.setViewportCount(1).setScissorCount(1);

    vk::PipelineRasterizationStateCreateInfo rsState{};
    rsState.setPolygonMode(vk::PolygonMode::eFill)
           .setCullMode   (vk::CullModeFlagBits::eNone)
           .setLineWidth  (1.0f);

    vk::PipelineMultisampleStateCreateInfo msState{};
    msState.setRasterizationSamples(vk::SampleCountFlagBits::e1);

    // No depth test, no depth write — stars are always in the background.
    vk::PipelineDepthStencilStateCreateInfo dsState{};
    dsState.setDepthTestEnable (vk::False)
           .setDepthWriteEnable(vk::False);

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
          .setPDepthStencilState (&dsState)
          .setPColorBlendState   (&cbState)
          .setPDynamicState      (&dynState)
          .setLayout             (m_layout)
          .setRenderPass         (hdrRenderPass);

    auto [result, pipeline] = device.createGraphicsPipeline(nullptr, gpInfo);
    if (result != vk::Result::eSuccess)
        throw std::runtime_error("Failed to create star pipeline");
    m_pipeline = pipeline;

    device.destroyShaderModule(vertMod);
    device.destroyShaderModule(fragMod);
}

StarField::~StarField()
{
    auto device = m_ctx.device();
    device.destroyPipeline      (m_pipeline);
    device.destroyPipelineLayout(m_layout);
}

void StarField::draw(vk::CommandBuffer cmd, const glm::mat4& vpRot) const
{
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, m_pipeline);
    cmd.pushConstants(m_layout, vk::ShaderStageFlagBits::eVertex,
                      0, sizeof(glm::mat4), &vpRot);
    vk::DeviceSize offset = 0;
    cmd.bindVertexBuffers(0, m_vertexBuffer.handle(), offset);
    cmd.draw(m_starCount, 1, 0, 0);
}

} // namespace apeiron::render
