#include "apeiron/render/GltfModel.h"
#include "apeiron/render/GpuAllocator.h"
#include "apeiron/render/Vertex.h"

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/glm_element_traits.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <stdexcept>
#include <cstring>
#include <algorithm>
#include <numeric>

namespace apeiron::render {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static glm::mat4 nodeLocalTransform(const GltfNode& n)
{
    glm::mat4 t = glm::translate(glm::mat4(1.0f), n.translation);
    glm::mat4 r = glm::mat4_cast(n.rotation);
    glm::mat4 s = glm::scale(glm::mat4(1.0f), n.scale);

    float eff = (n.scaleOverride > 0.0f) ? n.scaleOverride : 1.0f;
    if (eff != 1.0f)
        s = glm::scale(s, glm::vec3(eff));

    return t * r * s;
}

// ---------------------------------------------------------------------------
// load()
// ---------------------------------------------------------------------------

void GltfModel::load(GpuAllocator& allocator, const std::filesystem::path& path)
{
    // Parse.
    fastgltf::Parser parser;
    auto data = fastgltf::GltfDataBuffer::FromPath(path);
    if (data.error() != fastgltf::Error::None)
        throw std::runtime_error("fastgltf: cannot read " + path.string());

    constexpr auto kOpts =
        fastgltf::Options::DecomposeNodeMatrices |
        fastgltf::Options::LoadExternalBuffers;

    auto asset = parser.loadGltf(data.get(), path.parent_path(), kOpts);
    if (asset.error() != fastgltf::Error::None)
        throw std::runtime_error("fastgltf: parse error in " + path.string());

    auto& a = asset.get();

    // ---- Build mesh list ----
    m_meshes.reserve(a.meshes.size());

    for (const auto& gMesh : a.meshes) {
        for (const auto& prim : gMesh.primitives) {
            // Require POSITION.
            auto posIt = prim.findAttribute("POSITION");
            if (posIt == prim.attributes.end()) continue;

            // Fetch base colour from material.
            glm::vec3 baseColor{1.0f};
            bool      isEmissive   = false;
            float     emissiveScale = 1.0f;

            if (prim.materialIndex.has_value()) {
                const auto& mat = a.materials[prim.materialIndex.value()];
                auto& bc = mat.pbrData.baseColorFactor;
                baseColor = { static_cast<float>(bc[0]),
                              static_cast<float>(bc[1]),
                              static_cast<float>(bc[2]) };

                auto& ef = mat.emissiveFactor;
                float eLen = glm::length(glm::vec3(static_cast<float>(ef[0]),
                                                    static_cast<float>(ef[1]),
                                                    static_cast<float>(ef[2])));
                if (eLen > 0.01f) {
                    isEmissive = true;
                    baseColor  = glm::vec3(static_cast<float>(ef[0]),
                                           static_cast<float>(ef[1]),
                                           static_cast<float>(ef[2]));
                    // emissive strength extension
                    emissiveScale = mat.emissiveStrength > 0.0f
                                    ? mat.emissiveStrength : eLen;
                }
            }

            // Positions.
            std::vector<glm::vec3> positions;
            {
                auto& acc = a.accessors[posIt->accessorIndex];
                positions.resize(acc.count);
                fastgltf::copyFromAccessor<glm::vec3>(a, acc, positions.data());
            }

            // Normals (optional).
            std::vector<glm::vec3> normals(positions.size(), glm::vec3(0, 1, 0));
            if (auto nrmIt = prim.findAttribute("NORMAL");
                nrmIt != prim.attributes.end()) {
                auto& acc = a.accessors[nrmIt->accessorIndex];
                fastgltf::copyFromAccessor<glm::vec3>(a, acc, normals.data());
            }

            // UVs (optional).
            std::vector<glm::vec2> uvs(positions.size(), glm::vec2(0.0f));
            if (auto uvIt = prim.findAttribute("TEXCOORD_0");
                uvIt != prim.attributes.end()) {
                auto& acc = a.accessors[uvIt->accessorIndex];
                fastgltf::copyFromAccessor<glm::vec2>(a, acc, uvs.data());
            }

            // Build vertex array.
            std::vector<Vertex> verts(positions.size());
            for (std::size_t i = 0; i < positions.size(); ++i) {
                verts[i].position = positions[i];
                verts[i].normal   = normals[i];
                verts[i].color    = baseColor;
                verts[i].uv       = uvs[i];
            }

            // Indices.
            std::vector<uint32_t> indices;
            if (prim.indicesAccessor.has_value()) {
                auto& acc = a.accessors[prim.indicesAccessor.value()];
                indices.resize(acc.count);
                fastgltf::copyFromAccessor<uint32_t>(a, acc, indices.data());
            } else {
                indices.resize(verts.size());
                std::iota(indices.begin(), indices.end(), 0u);
            }

            bool doubleSided = false;
            if (prim.materialIndex.has_value())
                doubleSided = a.materials[prim.materialIndex.value()].doubleSided;

            m_meshes.emplace_back(allocator, verts, indices);
            m_meshDoubleSided.push_back(doubleSided);
        }
    }

    // ---- Build node tree ----
    // We need a mapping from glTF mesh index → our mesh list index.
    // Our mesh list is a flat list of primitives (one per primitive across all meshes).
    // For simplicity, map glTF mesh index → first primitive index in our list.
    std::vector<int> gltfMeshToOurIdx(a.meshes.size(), -1);
    {
        int ourIdx = 0;
        for (std::size_t mi = 0; mi < a.meshes.size(); ++mi) {
            if (!a.meshes[mi].primitives.empty()) {
                gltfMeshToOurIdx[mi] = ourIdx;
                ourIdx += static_cast<int>(a.meshes[mi].primitives.size());
            }
        }
    }

    // Also track per-mesh emissive info for nodes.
    std::vector<bool>  meshEmissive(m_meshes.size(), false);
    std::vector<float> meshEmissiveScale(m_meshes.size(), 1.0f);
    {
        int idx = 0;
        for (const auto& gMesh : a.meshes) {
            for (const auto& prim : gMesh.primitives) {
                if (prim.materialIndex.has_value()) {
                    const auto& mat = a.materials[prim.materialIndex.value()];
                    auto& ef = mat.emissiveFactor;
                    float eLen = glm::length(glm::vec3(static_cast<float>(ef[0]),
                                                        static_cast<float>(ef[1]),
                                                        static_cast<float>(ef[2])));
                    if (eLen > 0.01f) {
                        meshEmissive[idx]      = true;
                        meshEmissiveScale[idx] = mat.emissiveStrength > 0.0f
                                                 ? mat.emissiveStrength : eLen;
                    }
                }
                ++idx;
            }
        }
    }

    m_nodes.resize(a.nodes.size());
    for (std::size_t ni = 0; ni < a.nodes.size(); ++ni) {
        const auto& gNode = a.nodes[ni];
        auto& n = m_nodes[ni];

        n.name = gNode.name;

        // Transform (fastgltf decomposes matrices into TRS via DecomposeNodeMatrices).
        if (auto* trs = std::get_if<fastgltf::TRS>(&gNode.transform)) {
            n.translation = { static_cast<float>(trs->translation[0]),
                              static_cast<float>(trs->translation[1]),
                              static_cast<float>(trs->translation[2]) };
            n.rotation    = glm::quat(static_cast<float>(trs->rotation[3]),
                                      static_cast<float>(trs->rotation[0]),
                                      static_cast<float>(trs->rotation[1]),
                                      static_cast<float>(trs->rotation[2]));
            n.scale       = { static_cast<float>(trs->scale[0]),
                              static_cast<float>(trs->scale[1]),
                              static_cast<float>(trs->scale[2]) };
        }

        // Mesh — record start index and primitive count so all material slots render.
        if (gNode.meshIndex.has_value()) {
            std::size_t mi = gNode.meshIndex.value();
            n.meshIdx   = (mi < gltfMeshToOurIdx.size()) ? gltfMeshToOurIdx[mi] : -1;
            n.meshCount = (mi < a.meshes.size())
                          ? static_cast<int>(a.meshes[mi].primitives.size()) : 0;
            if (n.meshIdx >= 0 && n.meshIdx < static_cast<int>(meshEmissive.size())) {
                n.isEmissive    = meshEmissive[n.meshIdx];
                n.emissiveScale = meshEmissiveScale[n.meshIdx];
            }
        }

        // Children — set children list; parent filled in pass 2.
        for (auto childIdx : gNode.children)
            n.children.push_back(static_cast<int>(childIdx));
    }

    // Pass 2: set parent indices.
    for (int ni = 0; ni < static_cast<int>(m_nodes.size()); ++ni)
        for (int ci : m_nodes[ni].children)
            m_nodes[ci].parentIdx = ni;

    // Root nodes (no parent).
    for (int ni = 0; ni < static_cast<int>(m_nodes.size()); ++ni)
        if (m_nodes[ni].parentIdx == -1)
            m_rootNodes.push_back(ni);

    // Build name→index map.
    for (int ni = 0; ni < static_cast<int>(m_nodes.size()); ++ni)
        if (!m_nodes[ni].name.empty())
            m_nameIndex[m_nodes[ni].name] = ni;

    // Bounding radius — scan all positions loaded into m_meshes' vertices.
    // For now use a conservative fallback; a proper scan would require Mesh to
    // expose its vertex buffer, which we avoid for simplicity.
    m_boundingRadius = 5.0f;
}

// ---------------------------------------------------------------------------
// nodeWorldTransform
// ---------------------------------------------------------------------------

glm::mat4 GltfModel::nodeWorldTransform(std::string_view name) const
{
    auto it = m_nameIndex.find(std::string(name));
    if (it == m_nameIndex.end()) return glm::mat4(1.0f);

    // Walk up the parent chain accumulating transforms.
    glm::mat4 t(1.0f);
    int idx = it->second;
    while (idx >= 0) {
        t   = nodeLocalTransform(m_nodes[idx]) * t;
        idx = m_nodes[idx].parentIdx;
    }
    return t;
}

// ---------------------------------------------------------------------------
// setNodeVisible / setNodeScale
// ---------------------------------------------------------------------------

void GltfModel::setNodeVisible(std::string_view name, bool visible)
{
    auto it = m_nameIndex.find(std::string(name));
    if (it != m_nameIndex.end())
        m_nodes[it->second].visible = visible;
}

void GltfModel::setNodeScale(std::string_view name, float scale)
{
    auto it = m_nameIndex.find(std::string(name));
    if (it != m_nameIndex.end())
        m_nodes[it->second].scaleOverride = scale;
}

// ---------------------------------------------------------------------------
// draw()
// ---------------------------------------------------------------------------

void GltfModel::draw(vk::CommandBuffer       cmd,
                      const MeshPipeline&     pipeline,
                      const glm::mat4&        vp,
                      const glm::mat4&        rootModel,
                      const glm::vec3&        sunDirWorld) const
{
    if (m_meshes.empty()) return;

    // Start with the back-face-culled pipeline; drawNode rebinds as needed.
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.handle(false));
    bool boundDS = false;

    for (int ri : m_rootNodes)
        drawNode(cmd, pipeline, ri, vp * rootModel, rootModel, sunDirWorld, boundDS);
}

void GltfModel::drawNode(vk::CommandBuffer  cmd,
                          const MeshPipeline& pipeline,
                          int                nodeIdx,
                          const glm::mat4&   parentMvp,
                          const glm::mat4&   parentModel,
                          const glm::vec3&   sunDir,
                          bool&              boundDS) const
{
    const auto& n = m_nodes[nodeIdx];
    if (!n.visible) return;

    glm::mat4 nodeLocal = nodeLocalTransform(n);
    glm::mat4 model     = parentModel * nodeLocal;
    glm::mat4 mvp       = parentMvp   * nodeLocal;

    if (n.meshIdx >= 0 && n.meshCount > 0) {
        // Plume quads are double-sided; everything else uses back-face culling.
        bool ds = (n.name.find("plume") != std::string::npos);
        if (ds != boundDS) {
            cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.handle(ds));
            boundDS = ds;
        }

        MeshPushConstants pc{};
        pc.mvp       = mvp;
        pc.modelMat  = model;
        pc.sunDir    = glm::vec4(sunDir, n.isEmissive ? 1.0f : 0.0f);
        pc.baseColor = glm::vec4(1.0f, 1.0f, 1.0f, n.emissiveScale);

        cmd.pushConstants(pipeline.layout(),
                          vk::ShaderStageFlagBits::eVertex |
                          vk::ShaderStageFlagBits::eFragment,
                          0, sizeof(MeshPushConstants),
                          &pc);

        // Draw all primitives (one per material slot).
        for (int p = 0; p < n.meshCount; ++p) {
            int idx = n.meshIdx + p;
            if (idx < static_cast<int>(m_meshes.size()))
                m_meshes[idx].draw(cmd);
        }
    }

    for (int ci : n.children)
        drawNode(cmd, pipeline, ci, mvp, model, sunDir, boundDS);
}

} // namespace apeiron::render
