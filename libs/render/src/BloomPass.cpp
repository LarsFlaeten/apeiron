#include "apeiron/render/BloomPass.h"
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
        throw std::runtime_error("SPIR-V not a multiple of 4: " + path.string());
    file.seekg(0);
    std::vector<uint32_t> code(bytes / 4);
    file.read(reinterpret_cast<char*>(code.data()), bytes);
    return code;
}

} // namespace

namespace apeiron::render {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

BloomPass::BloomPass(const Context&               ctx,
                     GpuAllocator&                allocator,
                     const Swapchain&             swapchain,
                     const std::filesystem::path& shaderDir)
    : m_ctx(ctx), m_allocator(&allocator)
{
    auto device = ctx.device();

    // ----- Render pass (shared by all bloom sub-passes) -----
    // Single RGBA16F colour attachment.
    // finalLayout = ShaderReadOnlyOptimal so the next pass can sample it
    // without an explicit barrier.
    vk::AttachmentDescription colorAttach{};
    colorAttach.setFormat        (Swapchain::kHdrFormat)
               .setSamples       (vk::SampleCountFlagBits::e1)
               .setLoadOp        (vk::AttachmentLoadOp::eDontCare)
               .setStoreOp       (vk::AttachmentStoreOp::eStore)
               .setInitialLayout (vk::ImageLayout::eUndefined)
               .setFinalLayout   (vk::ImageLayout::eShaderReadOnlyOptimal);

    vk::AttachmentReference colorRef{0, vk::ImageLayout::eColorAttachmentOptimal};
    vk::SubpassDescription  subpass{};
    subpass.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
           .setColorAttachments (colorRef);

    // deps[0]: wait for the previous render pass's colour write before we start ours.
    // deps[1]: make our colour write visible as a fragment-shader read afterwards.
    std::array<vk::SubpassDependency, 2> deps{};
    deps[0].setSrcSubpass   (vk::SubpassExternal)
           .setDstSubpass   (0)
           .setSrcStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
           .setSrcAccessMask(vk::AccessFlagBits::eColorAttachmentWrite)
           .setDstStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
           .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentWrite);
    deps[1].setSrcSubpass   (0)
           .setDstSubpass   (vk::SubpassExternal)
           .setSrcStageMask (vk::PipelineStageFlagBits::eColorAttachmentOutput)
           .setSrcAccessMask(vk::AccessFlagBits::eColorAttachmentWrite)
           .setDstStageMask (vk::PipelineStageFlagBits::eFragmentShader)
           .setDstAccessMask(vk::AccessFlagBits::eShaderRead);

    vk::RenderPassCreateInfo rpInfo{};
    rpInfo.setAttachments(colorAttach).setSubpasses(subpass).setDependencies(deps);
    m_renderPass = device.createRenderPass(rpInfo);

    // ----- Descriptor set layout: binding 0 = one combined image sampler -----
    vk::DescriptorSetLayoutBinding samplerBinding{};
    samplerBinding.setBinding        (0)
                  .setDescriptorType (vk::DescriptorType::eCombinedImageSampler)
                  .setDescriptorCount(1)
                  .setStageFlags     (vk::ShaderStageFlagBits::eFragment);
    vk::DescriptorSetLayoutCreateInfo dslInfo{};
    dslInfo.setBindings(samplerBinding);
    m_descSetLayout = device.createDescriptorSetLayout(dslInfo);

    // ----- Pipeline layout: set 0 + 8-byte push constant (fragment) -----
    // 8 bytes covers both float threshold (4 B) and vec2 texelDir (8 B).
    vk::PushConstantRange pcRange{};
    pcRange.setStageFlags(vk::ShaderStageFlagBits::eFragment)
           .setOffset    (0)
           .setSize      (8);
    vk::PipelineLayoutCreateInfo plInfo{};
    plInfo.setSetLayouts       (m_descSetLayout)
          .setPushConstantRanges(pcRange);
    m_pipelineLayout = device.createPipelineLayout(plInfo);

    // ----- Sampler (linear, clamp-to-edge) -----
    vk::SamplerCreateInfo samplerInfo{};
    samplerInfo.setMagFilter    (vk::Filter::eLinear)
               .setMinFilter    (vk::Filter::eLinear)
               .setAddressModeU (vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeV (vk::SamplerAddressMode::eClampToEdge)
               .setAddressModeW (vk::SamplerAddressMode::eClampToEdge);
    m_sampler = device.createSampler(samplerInfo);

    // ----- Descriptor pool: 6 sets × 1 sampler each -----
    vk::DescriptorPoolSize poolSize{};
    poolSize.setType           (vk::DescriptorType::eCombinedImageSampler)
            .setDescriptorCount(6);
    vk::DescriptorPoolCreateInfo dpInfo{};
    dpInfo.setMaxSets  (6)
          .setPoolSizes(poolSize);
    m_descPool = device.createDescriptorPool(dpInfo);

    // Allocate all 6 sets in one call.
    std::array<vk::DescriptorSetLayout, 6> layouts;
    layouts.fill(m_descSetLayout);
    vk::DescriptorSetAllocateInfo dsAlloc{};
    dsAlloc.setDescriptorPool(m_descPool).setSetLayouts(layouts);
    auto sets = device.allocateDescriptorSets(dsAlloc);
    for (int i = 0; i < 6; ++i) m_sets[i] = sets[i];

    // ----- Pipelines -----
    // Both share the fullscreen-triangle vertex shader from the tonemap pass.
    auto makeModule = [&](const std::filesystem::path& spv) {
        auto code = readSpirv(spv);
        vk::ShaderModuleCreateInfo info{};
        info.setCode(code);
        return device.createShaderModule(info);
    };
    auto vertMod    = makeModule(shaderDir / "tonemap.vert.spv");
    auto threshFrag = makeModule(shaderDir / "bloom_threshold.frag.spv");
    auto blurFrag   = makeModule(shaderDir / "bloom_blur.frag.spv");

    vk::PipelineVertexInputStateCreateInfo   viState{};
    vk::PipelineInputAssemblyStateCreateInfo iaState{};
    iaState.setTopology(vk::PrimitiveTopology::eTriangleList);

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

    vk::PipelineColorBlendAttachmentState blendAttach{};
    blendAttach.setColorWriteMask(
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
        vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);
    vk::PipelineColorBlendStateCreateInfo cbState{};
    cbState.setAttachments(blendAttach);

    vk::PipelineDepthStencilStateCreateInfo dsState{};  // no depth test/write

    auto buildPipeline = [&](vk::ShaderModule fragMod) -> vk::Pipeline {
        std::array<vk::PipelineShaderStageCreateInfo, 2> stages{};
        stages[0].setStage(vk::ShaderStageFlagBits::eVertex)  .setModule(vertMod) .setPName("main");
        stages[1].setStage(vk::ShaderStageFlagBits::eFragment).setModule(fragMod).setPName("main");

        vk::GraphicsPipelineCreateInfo gpInfo{};
        gpInfo.setStages             (stages)
              .setPVertexInputState  (&viState)
              .setPInputAssemblyState(&iaState)
              .setPViewportState     (&vpState)
              .setPRasterizationState(&rsState)
              .setPMultisampleState  (&msState)
              .setPColorBlendState   (&cbState)
              .setPDepthStencilState (&dsState)
              .setPDynamicState      (&dynState)
              .setLayout             (m_pipelineLayout)
              .setRenderPass         (m_renderPass);

        auto [result, pipeline] = device.createGraphicsPipeline(nullptr, gpInfo);
        if (result != vk::Result::eSuccess)
            throw std::runtime_error("BloomPass: failed to create pipeline");
        return pipeline;
    };

    m_thresholdPipeline = buildPipeline(threshFrag);
    m_blurPipeline      = buildPipeline(blurFrag);

    device.destroyShaderModule(vertMod);
    device.destroyShaderModule(threshFrag);
    device.destroyShaderModule(blurFrag);

    // ----- Size-dependent resources -----
    createImages      (swapchain);
    createFramebuffers();
    updateDescriptors (swapchain);
}

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------

BloomPass::~BloomPass()
{
    auto device = m_ctx.device();
    destroyFramebuffers();
    destroyImages();
    device.destroyDescriptorPool    (m_descPool);
    device.destroySampler           (m_sampler);
    device.destroyPipeline          (m_blurPipeline);
    device.destroyPipeline          (m_thresholdPipeline);
    device.destroyPipelineLayout    (m_pipelineLayout);
    device.destroyDescriptorSetLayout(m_descSetLayout);
    device.destroyRenderPass        (m_renderPass);
}

// ---------------------------------------------------------------------------
// Size-dependent helpers
// ---------------------------------------------------------------------------

void BloomPass::createImages(const Swapchain& swapchain)
{
    auto ext = swapchain.extent();
    m_extent = vk::Extent2D{std::max(1u, ext.width  / 2),
                             std::max(1u, ext.height / 2)};

    for (int f = 0; f < 2; ++f) {
        for (int p = 0; p < 2; ++p) {
            m_bloomImages[f][p] = Image(m_allocator->handle(),
                                        m_extent,
                                        Swapchain::kHdrFormat,
                                        vk::ImageUsageFlagBits::eColorAttachment |
                                        vk::ImageUsageFlagBits::eSampled);

            vk::ImageViewCreateInfo viewInfo{};
            viewInfo.setImage           (m_bloomImages[f][p].handle())
                    .setViewType        (vk::ImageViewType::e2D)
                    .setFormat          (Swapchain::kHdrFormat)
                    .setSubresourceRange({vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1});
            m_bloomViews[f][p] = m_ctx.device().createImageView(viewInfo);
        }
    }
}

void BloomPass::destroyImages()
{
    auto device = m_ctx.device();
    for (auto& row : m_bloomViews)
        for (auto v : row)
            device.destroyImageView(v);
    // m_bloomImages destroyed by Image RAII when overwritten
}

void BloomPass::createFramebuffers()
{
    auto device = m_ctx.device();
    for (int f = 0; f < 2; ++f) {
        for (int p = 0; p < 2; ++p) {
            vk::FramebufferCreateInfo fbInfo{};
            fbInfo.setRenderPass(m_renderPass)
                  .setAttachments(m_bloomViews[f][p])
                  .setWidth (m_extent.width)
                  .setHeight(m_extent.height)
                  .setLayers(1);
            m_framebuffers[f][p] = device.createFramebuffer(fbInfo);
        }
    }
}

void BloomPass::destroyFramebuffers()
{
    auto device = m_ctx.device();
    for (auto& row : m_framebuffers)
        for (auto fb : row)
            device.destroyFramebuffer(fb);
}

void BloomPass::updateDescriptors(const Swapchain& swapchain)
{
    auto writeSet = [&](vk::DescriptorSet set,
                        vk::ImageView     view,
                        vk::Sampler       samp) {
        vk::DescriptorImageInfo imgInfo{};
        imgInfo.setImageLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
               .setImageView  (view)
               .setSampler    (samp);
        vk::WriteDescriptorSet write{};
        write.setDstSet        (set)
             .setDstBinding    (0)
             .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
             .setImageInfo     (imgInfo);
        m_ctx.device().updateDescriptorSets(write, {});
    };

    // sets[0,1]: threshold reads HDR image 0 or 1
    for (int i = 0; i < 2; ++i)
        writeSet(m_sets[i], swapchain.hdrImageView(i), m_sampler);

    // sets[2,3]: blur for frame 0, reads bloom[0][0] or bloom[0][1]
    // sets[4,5]: blur for frame 1, reads bloom[1][0] or bloom[1][1]
    for (int f = 0; f < 2; ++f)
        for (int p = 0; p < 2; ++p)
            writeSet(m_sets[2 + f * 2 + p], m_bloomViews[f][p], m_sampler);
}

void BloomPass::recreate(const Swapchain& swapchain)
{
    destroyFramebuffers();
    destroyImages();
    createImages      (swapchain);
    createFramebuffers();
    updateDescriptors (swapchain);
}

// ---------------------------------------------------------------------------
// render
// ---------------------------------------------------------------------------

void BloomPass::render(vk::CommandBuffer cmd, int frameIndex)
{
    const auto w = static_cast<float>(m_extent.width);
    const auto h = static_cast<float>(m_extent.height);

    const vk::Viewport vp{0.f, 0.f, w, h, 0.f, 1.f};
    const vk::Rect2D   scissor{{0, 0}, m_extent};

    // Helper: run one fullscreen render pass into bloom[frameIndex][dst].
    auto pass = [&](vk::Pipeline      pipeline,
                    vk::DescriptorSet set,
                    int               dst,
                    const void*       pushData,
                    uint32_t          pushSize)
    {
        vk::RenderPassBeginInfo rpBegin{};
        rpBegin.setRenderPass (m_renderPass)
               .setFramebuffer(m_framebuffers[frameIndex][dst])
               .setRenderArea ({vk::Offset2D{0, 0}, m_extent});
        cmd.beginRenderPass(rpBegin, vk::SubpassContents::eInline);
        cmd.setViewport(0, vp);
        cmd.setScissor (0, scissor);
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline);
        cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                               m_pipelineLayout, 0, set, {});
        cmd.pushConstants(m_pipelineLayout, vk::ShaderStageFlagBits::eFragment,
                          0, pushSize, pushData);
        cmd.draw(3, 1, 0, 0);
        cmd.endRenderPass();
    };

    // 1. Threshold: HDR[frameIndex] → bloom[frame][0]
    pass(m_thresholdPipeline, m_sets[frameIndex], 0, &m_threshold, sizeof(float));

    // 2-5. Two iterations of separable Gaussian (H → V → H → V).
    // Ping-pong within the frame's own image pair.
    const int blurBase = 2 + frameIndex * 2;  // first blur set for this frame
    float texH[2] = {1.0f / w, 0.0f};
    float texV[2] = {0.0f,     1.0f / h};

    pass(m_blurPipeline, m_sets[blurBase + 0], 1, texH, sizeof(texH));  // bloom[f][0] → [1]
    pass(m_blurPipeline, m_sets[blurBase + 1], 0, texV, sizeof(texV));  // bloom[f][1] → [0]
    pass(m_blurPipeline, m_sets[blurBase + 0], 1, texH, sizeof(texH));  // bloom[f][0] → [1]
    pass(m_blurPipeline, m_sets[blurBase + 1], 0, texV, sizeof(texV));  // bloom[f][1] → [0]
    // Output: bloom[frameIndex][0]  (= outputImageView(frameIndex))
}

} // namespace apeiron::render
