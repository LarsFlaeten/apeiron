#include "apeiron/render/GltfModel.h"
#include "apeiron/render/GpuAllocator.h"
#include "apeiron/render/Context.h"
#include "apeiron/render/Vertex.h"
#include "apeiron/render/Buffer.h"
#include "apeiron/render/Texture.h"

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>
#include <fastgltf/glm_element_traits.hpp>

#include <draco/compression/decode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/mesh/mesh.h>

#include <meshoptimizer.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <stdexcept>
#include <cstring>
#include <algorithm>
#include <numeric>
#include <iostream>

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

// Extract raw image bytes from a fastgltf image data source.
// Returns nullptr + 0 if the source is not byte-accessible.
static const uint8_t* imageBytes(const fastgltf::Asset& a,
                                  const fastgltf::Image& img,
                                  std::size_t& outLen)
{
    outLen = 0;
    if (auto* bv = std::get_if<fastgltf::sources::BufferView>(&img.data)) {
        const auto& view   = a.bufferViews[bv->bufferViewIndex];
        const auto& buffer = a.buffers[view.bufferIndex];
        if (auto* arr = std::get_if<fastgltf::sources::Array>(&buffer.data)) {
            outLen = view.byteLength;
            return reinterpret_cast<const uint8_t*>(arr->bytes.data()) + view.byteOffset;
        }
        if (auto* vec = std::get_if<fastgltf::sources::Vector>(&buffer.data)) {
            outLen = view.byteLength;
            return reinterpret_cast<const uint8_t*>(vec->bytes.data()) + view.byteOffset;
        }
    }
    if (auto* arr = std::get_if<fastgltf::sources::Array>(&img.data)) {
        outLen = arr->bytes.size();
        return reinterpret_cast<const uint8_t*>(arr->bytes.data());
    }
    if (auto* vec = std::get_if<fastgltf::sources::Vector>(&img.data)) {
        outLen = vec->bytes.size();
        return reinterpret_cast<const uint8_t*>(vec->bytes.data());
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// load()
// ---------------------------------------------------------------------------

void GltfModel::load(const Context& ctx, GpuAllocator& allocator,
                     MeshPipeline& pipeline, const std::filesystem::path& path)
{
    // Parse.
    // Enable all extensions we can handle. Extensions requiring external decoders
    // (Draco, meshopt) are included — their compressed data is decoded below.
    // Metadata-only extensions (materials, quantization, etc.) are parsed natively.
    fastgltf::Parser parser(
        fastgltf::Extensions::KHR_draco_mesh_compression |
        fastgltf::Extensions::EXT_meshopt_compression    |
        fastgltf::Extensions::EXT_texture_webp           |
        fastgltf::Extensions::KHR_mesh_quantization      |
        fastgltf::Extensions::KHR_texture_transform      |
        fastgltf::Extensions::KHR_texture_basisu         |
        fastgltf::Extensions::KHR_lights_punctual        |
        fastgltf::Extensions::KHR_materials_unlit        |
        fastgltf::Extensions::KHR_materials_emissive_strength |
        fastgltf::Extensions::KHR_materials_specular     |
        fastgltf::Extensions::KHR_materials_ior          |
        fastgltf::Extensions::KHR_materials_clearcoat    |
        fastgltf::Extensions::KHR_materials_transmission |
        fastgltf::Extensions::KHR_materials_volume       |
        fastgltf::Extensions::KHR_materials_sheen        |
        fastgltf::Extensions::KHR_materials_variants     |
        fastgltf::Extensions::EXT_mesh_gpu_instancing);

    auto data = fastgltf::GltfDataBuffer::FromPath(path);
    if (data.error() != fastgltf::Error::None)
        throw std::runtime_error("fastgltf: cannot read " + path.string());

    constexpr auto kOpts =
        fastgltf::Options::DecomposeNodeMatrices |
        fastgltf::Options::LoadExternalBuffers;

    auto asset = parser.loadGltf(data.get(), path.parent_path(), kOpts);
    if (asset.error() != fastgltf::Error::None)
        throw std::runtime_error("fastgltf: parse error in " + path.string()
            + " — " + std::string(fastgltf::getErrorMessage(asset.error())));

    auto& a = asset.get();

    // ---- Decompress EXT_meshopt_compression buffer views ----
    // For each buffer view that carries meshopt compression metadata, decode it
    // into a fresh fastgltf buffer so that all subsequent iterateAccessor calls
    // work transparently on the uncompressed data.
    {
        // Helper: get raw bytes of a fastgltf buffer.
        auto bufferBytes = [&](std::size_t bufIdx) -> const uint8_t* {
            const auto& buf = a.buffers[bufIdx];
            if (auto* arr = std::get_if<fastgltf::sources::Array>(&buf.data))
                return reinterpret_cast<const uint8_t*>(arr->bytes.data());
            if (auto* vec = std::get_if<fastgltf::sources::Vector>(&buf.data))
                return reinterpret_cast<const uint8_t*>(vec->bytes.data());
            return nullptr;
        };

        for (auto& bv : a.bufferViews) {
            if (!bv.meshoptCompression) continue;
            const auto& mc = *bv.meshoptCompression;

            const uint8_t* src = bufferBytes(mc.bufferIndex);
            if (!src) {
                std::cerr << "[GltfModel] meshopt: cannot access source buffer\n";
                continue;
            }
            src += mc.byteOffset;

            const std::size_t decodedSize = mc.count * mc.byteStride;
            fastgltf::sources::Vector decoded;
            decoded.bytes.resize(decodedSize);

            int rc = -1;
            using Mode = fastgltf::MeshoptCompressionMode;
            if (mc.mode == Mode::Attributes)
                rc = meshopt_decodeVertexBuffer(decoded.bytes.data(), mc.count,
                                                mc.byteStride, src, mc.byteLength);
            else  // Triangles or Indices
                rc = meshopt_decodeIndexBuffer(decoded.bytes.data(), mc.count,
                                               mc.byteStride, src, mc.byteLength);

            if (rc != 0) {
                std::cerr << "[GltfModel] meshopt: decode failed (rc=" << rc << ")\n";
                continue;
            }

            // Apply filter if present.
            using Filter = fastgltf::MeshoptCompressionFilter;
            switch (mc.filter) {
                case Filter::Octahedral:
                    meshopt_decodeFilterOct(decoded.bytes.data(), mc.count, mc.byteStride);
                    break;
                case Filter::Quaternion:
                    meshopt_decodeFilterQuat(decoded.bytes.data(), mc.count, mc.byteStride);
                    break;
                case Filter::Exponential:
                    meshopt_decodeFilterExp(decoded.bytes.data(), mc.count, mc.byteStride);
                    break;
                default: break;
            }

            // Add decoded bytes as a new buffer and redirect this buffer view to it.
            fastgltf::Buffer newBuf;
            newBuf.byteLength = decodedSize;
            newBuf.data       = std::move(decoded);
            a.buffers.push_back(std::move(newBuf));

            bv.bufferIndex      = a.buffers.size() - 1;
            bv.byteOffset       = 0;
            bv.byteLength       = decodedSize;
            bv.byteStride       = mc.byteStride;
            bv.meshoptCompression.reset();
        }
    }

    // ---- Pre-load all images into m_textures ----
    // m_textures[0]   = white 1×1 fallback
    // m_textures[i+1] = a.images[i] (or white fallback if decode fails)
    m_textures.reserve(a.images.size() + 1);
    m_textures.push_back(Texture::makeWhite(ctx, allocator));  // index 0 = fallback

    for (std::size_t i = 0; i < a.images.size(); ++i) {
        const auto& img = a.images[i];
        std::size_t byteLen = 0;
        const uint8_t* bytes = imageBytes(a, img, byteLen);
        if (bytes && byteLen > 0) {
            try {
                m_textures.push_back(
                    Texture::fromMemory(ctx, allocator,
                                        bytes, static_cast<int>(byteLen)));
                std::cout << "[GltfModel] image[" << i << "] \""
                          << img.name << "\" loaded (" << byteLen << " bytes)\n";
                continue;
            } catch (const std::exception& e) {
                std::cout << "[GltfModel] image[" << i << "] \""
                          << img.name << "\" decode failed: " << e.what() << "\n";
            }
        } else {
            // Diagnose why bytes are null: print which source variant is active.
            std::string srcType = "unknown";
            std::visit([&](const auto& s) {
                using T = std::decay_t<decltype(s)>;
                if      constexpr (std::is_same_v<T, fastgltf::sources::BufferView>) srcType = "BufferView";
                else if constexpr (std::is_same_v<T, fastgltf::sources::URI>)        srcType = "URI";
                else if constexpr (std::is_same_v<T, fastgltf::sources::Array>)      srcType = "Array";
                else if constexpr (std::is_same_v<T, fastgltf::sources::Vector>)     srcType = "Vector";
                else if constexpr (std::is_same_v<T, std::monostate>)                srcType = "monostate(empty)";
            }, img.data);
            std::cout << "[GltfModel] image[" << i << "] \""
                      << img.name << "\" no bytes (source=" << srcType << ")\n";
        }
        m_textures.push_back(Texture::makeWhite(ctx, allocator));
    }

    // Diagnostic: dump texture index structure for the first few textures.
    {
        std::size_t nShow = std::min(a.textures.size(), std::size_t(5));
        std::cout << "[GltfModel] " << a.textures.size() << " textures, "
                  << a.images.size() << " images (showing first " << nShow << "):\n";
        for (std::size_t ti = 0; ti < nShow; ++ti) {
            const auto& tex = a.textures[ti];
            std::cout << "  tex[" << ti << "] imageIndex=";
            if (tex.imageIndex.has_value()) std::cout << tex.imageIndex.value();
            else                             std::cout << "none";
            std::cout << " webpImageIndex=";
            if (tex.webpImageIndex.has_value()) std::cout << tex.webpImageIndex.value();
            else                                 std::cout << "none";
            std::cout << "\n";
        }
    }

    // Helper: resolve TextureInfo → m_textures index (0 = white fallback).
    // glTF image index i maps to m_textures[i+1].
    // For EXT_texture_webp textures, imageIndex may be absent; use webpImageIndex.
    auto resolveTexIdx = [&](std::size_t texIdx) -> int {
        if (texIdx >= a.textures.size()) return 0;
        const auto& tex = a.textures[texIdx];
        // Prefer standard imageIndex; fall back to WebP extension image index.
        std::optional<std::size_t> imgIdxOpt = tex.imageIndex;
        if (!imgIdxOpt.has_value() && tex.webpImageIndex.has_value())
            imgIdxOpt = tex.webpImageIndex;
        if (!imgIdxOpt.has_value()) return 0;
        int imgIdx = static_cast<int>(imgIdxOpt.value());
        int mIdx   = imgIdx + 1;
        return (mIdx < static_cast<int>(m_textures.size())) ? mIdx : 0;
    };

    // ---- Draco decode helper ----
    // Returns decoded positions/normals/uvs/indices for a Draco-compressed primitive.
    // Leaves vectors unchanged if the primitive is not Draco-compressed.
    auto decodeDraco = [&](const fastgltf::Primitive& prim,
                            std::vector<glm::vec3>& positions,
                            std::vector<glm::vec3>& normals,
                            std::vector<glm::vec2>& uvs,
                            std::vector<uint32_t>&  indices) -> bool
    {
        if (!prim.dracoCompression) return false;

        const auto& dc  = *prim.dracoCompression;
        const auto& bv  = a.bufferViews[dc.bufferView];
        const auto& buf = a.buffers[bv.bufferIndex];

        const uint8_t* compressedData = nullptr;
        std::size_t    compressedLen  = 0;

        std::visit([&](const auto& src) {
            using T = std::decay_t<decltype(src)>;
            if constexpr (std::is_same_v<T, fastgltf::sources::Array>) {
                compressedData = reinterpret_cast<const uint8_t*>(src.bytes.data()) + bv.byteOffset;
                compressedLen  = bv.byteLength;
            } else if constexpr (std::is_same_v<T, fastgltf::sources::Vector>) {
                compressedData = reinterpret_cast<const uint8_t*>(src.bytes.data()) + bv.byteOffset;
                compressedLen  = bv.byteLength;
            }
        }, buf.data);

        if (!compressedData || compressedLen == 0) {
            std::cerr << "[GltfModel] Draco: no compressed data in buffer\n";
            return false;
        }

        draco::DecoderBuffer decBuf;
        decBuf.Init(reinterpret_cast<const char*>(compressedData),
                    static_cast<size_t>(compressedLen));

        draco::Decoder decoder;
        auto result = decoder.DecodeMeshFromBuffer(&decBuf);
        if (!result.ok()) {
            std::cerr << "[GltfModel] Draco decode failed: "
                      << result.status().error_msg_string() << "\n";
            return false;
        }
        const draco::Mesh* mesh = result.value().get();

        // Positions.
        const draco::PointAttribute* posAttr =
            mesh->GetNamedAttribute(draco::GeometryAttribute::POSITION);
        if (posAttr) {
            positions.resize(mesh->num_points());
            for (draco::PointIndex pi(0);
                 pi < static_cast<uint32_t>(mesh->num_points()); ++pi) {
                std::array<float, 3> v{};
                posAttr->GetMappedValue(pi, v.data());
                positions[pi.value()] = { v[0], v[1], v[2] };
            }
        }

        // Normals.
        const draco::PointAttribute* nrmAttr =
            mesh->GetNamedAttribute(draco::GeometryAttribute::NORMAL);
        normals.assign(positions.size(), glm::vec3(0, 1, 0));
        if (nrmAttr) {
            for (draco::PointIndex pi(0);
                 pi < static_cast<uint32_t>(mesh->num_points()); ++pi) {
                std::array<float, 3> v{};
                nrmAttr->GetMappedValue(pi, v.data());
                normals[pi.value()] = { v[0], v[1], v[2] };
            }
        }

        // UVs.
        const draco::PointAttribute* uvAttr =
            mesh->GetNamedAttribute(draco::GeometryAttribute::TEX_COORD);
        uvs.assign(positions.size(), glm::vec2(0.0f));
        if (uvAttr) {
            for (draco::PointIndex pi(0);
                 pi < static_cast<uint32_t>(mesh->num_points()); ++pi) {
                std::array<float, 2> v{};
                uvAttr->GetMappedValue(pi, v.data());
                uvs[pi.value()] = { v[0], v[1] };
            }
        }

        // Indices from faces.
        indices.clear();
        indices.reserve(mesh->num_faces() * 3);
        for (draco::FaceIndex fi(0);
             fi < static_cast<uint32_t>(mesh->num_faces()); ++fi) {
            const auto& face = mesh->face(fi);
            indices.push_back(face[0].value());
            indices.push_back(face[1].value());
            indices.push_back(face[2].value());
        }

        return true;
    };

    // ---- Build mesh list ----
    m_meshes.reserve(a.meshes.size());
    m_materials.reserve(a.meshes.size());
    bool uvDiagDone = false;  // fires once per load() call

    for (const auto& gMesh : a.meshes) {
        for (const auto& prim : gMesh.primitives) {
            // Require POSITION.
            auto posIt = prim.findAttribute("POSITION");
            if (posIt == prim.attributes.end()) continue;

            // ---- Material ----
            GltfMaterial mat;
            glm::vec3 baseColor{1.0f};
            float metallic  = 0.0f;
            float roughness = 1.0f;
            bool  isEmissive = false;
            float emissiveScale = 1.0f;
            int   texIdx = 0;  // index into m_textures (0 = white fallback)

            if (prim.materialIndex.has_value()) {
                const auto& m = a.materials[prim.materialIndex.value()];
                mat.doubleSided = m.doubleSided;

                auto& bc = m.pbrData.baseColorFactor;
                baseColor = { static_cast<float>(bc[0]),
                              static_cast<float>(bc[1]),
                              static_cast<float>(bc[2]) };
                metallic  = static_cast<float>(m.pbrData.metallicFactor);
                roughness = static_cast<float>(m.pbrData.roughnessFactor);

                // Emissive materials (plumes etc.) keep their old path.
                auto& ef = m.emissiveFactor;
                float eLen = glm::length(glm::vec3(static_cast<float>(ef[0]),
                                                    static_cast<float>(ef[1]),
                                                    static_cast<float>(ef[2])));
                if (eLen > 0.01f) {
                    isEmissive = true;
                    baseColor  = glm::vec3(static_cast<float>(ef[0]),
                                           static_cast<float>(ef[1]),
                                           static_cast<float>(ef[2]));
                    emissiveScale = m.emissiveStrength > 0.0f
                                    ? m.emissiveStrength : eLen;
                }

                // Base-colour texture.
                if (m.pbrData.baseColorTexture.has_value()) {
                    texIdx = resolveTexIdx(m.pbrData.baseColorTexture->textureIndex);
                    std::cout << "[GltfModel] mat \"" << m.name
                              << "\" → texIdx=" << texIdx
                              << " (gltfTex=" << m.pbrData.baseColorTexture->textureIndex << ")\n";
                } else {
                    std::cout << "[GltfModel] mat \"" << m.name << "\" → no texture\n";
                }
            }

            // Upload MaterialUBO.
            // baseColor goes only into the UBO — NOT into vertex color (avoids double-apply).
            MaterialUBO uboData{};
            uboData.baseColor = glm::vec4(baseColor, 1.0f);
            uboData.metallic  = metallic;
            uboData.roughness = roughness;

            mat.ubo = Buffer(allocator.handle(),
                             sizeof(MaterialUBO),
                             vk::BufferUsageFlagBits::eUniformBuffer,
                             VMA_MEMORY_USAGE_AUTO,
                             VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT);
            mat.ubo.upload(&uboData, sizeof(MaterialUBO));

            // Bind texture from shared m_textures (no ownership transfer).
            mat.descSet = pipeline.allocateMaterialDescSet(
                mat.ubo.handle(), sizeof(MaterialUBO),
                m_textures[texIdx].imageView(),
                m_textures[texIdx].sampler());

            // Positions / normals / UVs / indices.
            // For Draco-compressed primitives the geometry lives in a compressed
            // buffer view; decodeDraco() fills all four vectors directly.
            // For uncompressed primitives we read from accessors as normal.
            std::vector<glm::vec3> positions;
            std::vector<glm::vec3> normals;
            std::vector<glm::vec2> uvs;
            std::vector<uint32_t>  indices;

            if (!decodeDraco(prim, positions, normals, uvs, indices)) {
                // Uncompressed path.
                {
                    auto& acc = a.accessors[posIt->accessorIndex];
                    positions.resize(acc.count);
                    fastgltf::copyFromAccessor<glm::vec3>(a, acc, positions.data());
                }
                normals.assign(positions.size(), glm::vec3(0, 1, 0));
                if (auto nrmIt = prim.findAttribute("NORMAL");
                    nrmIt != prim.attributes.end()) {
                    auto& acc = a.accessors[nrmIt->accessorIndex];
                    fastgltf::copyFromAccessor<glm::vec3>(a, acc, normals.data());
                }
                uvs.assign(positions.size(), glm::vec2(0.0f));
                if (auto uvIt = prim.findAttribute("TEXCOORD_0");
                    uvIt != prim.attributes.end()) {
                    auto& acc = a.accessors[uvIt->accessorIndex];
                    fastgltf::copyFromAccessor<glm::vec2>(a, acc, uvs.data());
                    // KHR_mesh_quantization stores UVs as normalized unsigned short.
                    // fastgltf's glm::vec2 traits expect float components, so the
                    // raw integer is cast without normalization — fix it here.
                    if (acc.normalized && acc.componentType == fastgltf::ComponentType::UnsignedShort) {
                        constexpr float kInv = 1.0f / 65535.0f;
                        for (auto& uv : uvs) uv *= kInv;
                    } else if (acc.normalized && acc.componentType == fastgltf::ComponentType::UnsignedByte) {
                        constexpr float kInv = 1.0f / 255.0f;
                        for (auto& uv : uvs) uv *= kInv;
                    }
                }
                if (prim.indicesAccessor.has_value()) {
                    auto& acc = a.accessors[prim.indicesAccessor.value()];
                    indices.resize(acc.count);
                    fastgltf::copyFromAccessor<uint32_t>(a, acc, indices.data());
                } else {
                    indices.resize(positions.size());
                    std::iota(indices.begin(), indices.end(), 0u);
                }
            }

            if (positions.empty()) continue;

            // UV sanity check on the first textured primitive of each model load.
            // (Not static — fires once per GltfModel::load() call.)
            if (!uvDiagDone && texIdx > 0 && !uvs.empty()) {
                uvDiagDone = true;
                std::cout << "[GltfModel] UV sanity (first textured prim, "
                          << uvs.size() << " verts): ";
                for (std::size_t di = 0; di < std::min(uvs.size(), std::size_t(4)); ++di)
                    std::cout << "(" << uvs[di].x << "," << uvs[di].y << ") ";
                std::cout << "\n";
                // Also report UV accessor component type to detect quantization.
                if (auto uvIt2 = prim.findAttribute("TEXCOORD_0");
                    uvIt2 != prim.attributes.end()) {
                    const auto& acc2 = a.accessors[uvIt2->accessorIndex];
                    std::cout << "[GltfModel] UV accessor: componentType="
                              << static_cast<int>(acc2.componentType)
                              << " normalized=" << acc2.normalized
                              << " byteOffset=" << acc2.byteOffset << "\n";
                }
            }

            // Vertex color is white — base colour lives in the MaterialUBO.
            std::vector<Vertex> verts(positions.size());
            for (std::size_t i = 0; i < positions.size(); ++i) {
                verts[i].position = positions[i];
                verts[i].normal   = normals[i];
                verts[i].color    = glm::vec3(1.0f);
                verts[i].uv       = uvs[i];
            }

            m_meshes.emplace_back(allocator, verts, indices);
            m_materials.push_back(std::move(mat));

            // Stash emissive info for the per-primitive emissive array (built below).
            (void)isEmissive;
            (void)emissiveScale;
        }
    }

    // ---- Track emissive per-primitive (parallel to m_meshes) ----
    // We need to rebuild this now that we've restructured the loop.
    std::vector<bool>  meshEmissive(m_meshes.size(), false);
    std::vector<float> meshEmissiveScale(m_meshes.size(), 1.0f);
    {
        int idx = 0;
        for (const auto& gMesh : a.meshes) {
            for (const auto& prim : gMesh.primitives) {
                if (idx >= static_cast<int>(m_meshes.size())) break;
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

    // ---- Build node tree ----
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

    m_nodes.resize(a.nodes.size());
    for (std::size_t ni = 0; ni < a.nodes.size(); ++ni) {
        const auto& gNode = a.nodes[ni];
        auto& n = m_nodes[ni];

        n.name = gNode.name;

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

    m_boundingRadius = 5.0f;
}

// ---------------------------------------------------------------------------
// nodeWorldTransform
// ---------------------------------------------------------------------------

glm::mat4 GltfModel::nodeWorldTransform(std::string_view name) const
{
    auto it = m_nameIndex.find(std::string(name));
    if (it == m_nameIndex.end()) return glm::mat4(1.0f);

    glm::mat4 t(1.0f);
    int idx = it->second;
    while (idx >= 0) {
        t   = nodeLocalTransform(m_nodes[idx]) * t;
        idx = m_nodes[idx].parentIdx;
    }
    return t;
}

// ---------------------------------------------------------------------------
// setNodeVisible / setNodeScale / setNodeColor
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

void GltfModel::setNodeColor(std::string_view name, glm::vec3 color, float intensity)
{
    auto it = m_nameIndex.find(std::string(name));
    if (it != m_nameIndex.end()) {
        m_nodes[it->second].colorOverride     = color;
        m_nodes[it->second].intensityOverride = intensity;
    }
}

// ---------------------------------------------------------------------------
// draw()
// ---------------------------------------------------------------------------

void GltfModel::draw(vk::CommandBuffer       cmd,
                      const MeshPipeline&     pipeline,
                      const glm::mat4&        vp,
                      const glm::mat4&        rootModel,
                      const glm::vec3&        sunDirWorld,
                      const glm::vec3&        camPosWorld) const
{
    if (m_meshes.empty()) return;

    glm::mat4 mvp = vp * rootModel;

    // Pass 1: opaque geometry — writes depth.
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.handle(false));
    bool boundDS = false;
    for (int ri : m_rootNodes)
        drawNode(cmd, pipeline, ri, mvp, rootModel, sunDirWorld, camPosWorld, false, boundDS);

    // Pass 2: plumes — reads depth only, additive blend.
    cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.plumeHandle());
    boundDS = true;
    for (int ri : m_rootNodes)
        drawNode(cmd, pipeline, ri, mvp, rootModel, sunDirWorld, camPosWorld, true, boundDS);
}

void GltfModel::drawNode(vk::CommandBuffer  cmd,
                          const MeshPipeline& pipeline,
                          int                nodeIdx,
                          const glm::mat4&   parentMvp,
                          const glm::mat4&   parentModel,
                          const glm::vec3&   sunDir,
                          const glm::vec3&   camPos,
                          bool               plumePass,
                          bool&              boundDS) const
{
    const auto& n = m_nodes[nodeIdx];
    if (!n.visible) return;

    glm::mat4 nodeLocal = nodeLocalTransform(n);
    glm::mat4 model     = parentModel * nodeLocal;
    glm::mat4 mvp       = parentMvp   * nodeLocal;

    if (n.meshIdx >= 0 && n.meshCount > 0) {
        bool isPlume = (n.name.find("plume") != std::string::npos);

        if (isPlume == plumePass) {
            if (!plumePass && isPlume != boundDS) {
                cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline.handle(false));
                boundDS = false;
            }

            MeshPushConstants pc{};
            pc.mvp      = mvp;
            pc.modelMat = model;
            pc.sunDir   = glm::vec4(sunDir, n.isEmissive ? 1.0f : 0.0f);
            pc.camPos   = glm::vec4(camPos, 0.0f);
            if (plumePass)
                pc.baseColor = glm::vec4(n.colorOverride, n.intensityOverride);
            else
                pc.baseColor = glm::vec4(1.0f, 1.0f, 1.0f, n.emissiveScale);

            cmd.pushConstants(pipeline.layout(),
                              vk::ShaderStageFlagBits::eVertex |
                              vk::ShaderStageFlagBits::eFragment,
                              0, sizeof(MeshPushConstants),
                              &pc);

            for (int p = 0; p < n.meshCount; ++p) {
                int idx = n.meshIdx + p;
                if (idx >= static_cast<int>(m_meshes.size())) break;

                // Bind material descriptor set (UBO + albedo) for this primitive.
                // Plume pass uses push-constant colour only — still bind a valid set
                // so the shader can sample (result ignored for plumes).
                if (idx < static_cast<int>(m_materials.size())
                    && m_materials[idx].descSet)
                {
                    // Switch to DS pipeline if this primitive is double-sided.
                    if (!plumePass) {
                        bool wantDS = m_materials[idx].doubleSided;
                        if (wantDS != boundDS) {
                            cmd.bindPipeline(vk::PipelineBindPoint::eGraphics,
                                             pipeline.handle(wantDS));
                            boundDS = wantDS;
                        }
                    }

                    cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                                           pipeline.layout(), 0,
                                           m_materials[idx].descSet, {});
                }

                m_meshes[idx].draw(cmd);
            }
        }
    }

    for (int ci : n.children)
        drawNode(cmd, pipeline, ci, mvp, model, sunDir, camPos, plumePass, boundDS);
}

} // namespace apeiron::render
