#pragma once

#include "apeiron/render/Mesh.h"
#include "apeiron/render/MeshPipeline.h"
#include "apeiron/render/Texture.h"
#include "apeiron/render/Buffer.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <filesystem>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>

namespace apeiron::render {

class Context;
class GpuAllocator;

// Per-primitive GPU material: UBO + albedo texture + descriptor set.
struct GltfMaterial {
    Buffer            ubo;
    Texture           albedo;
    vk::DescriptorSet descSet{};
    bool              doubleSided = false;
};

// ---------------------------------------------------------------------------
// GltfModel
//
// Loads a glTF/GLB file and uploads all mesh primitives to the GPU.
// Maintains the full node hierarchy so nodes can be:
//   - hidden (visible = false)      → exhaust plumes off
//   - scale-overridden              → plume size by throttle
//
// Positions and directions are in the file's model space.
// The caller supplies the root world transform to draw().
// ---------------------------------------------------------------------------

struct GltfNode {
    std::string             name;
    int                     parentIdx  = -1;   // -1 = root
    std::vector<int>        children;
    glm::vec3               translation{0.0f};
    glm::quat               rotation   {1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3               scale      {1.0f};
    int                     meshIdx    = -1;   // first index into m_meshes; -1 = no mesh
    int                     meshCount  =  0;   // number of primitives (usually 1)

    // Runtime state (toggled by caller each frame).
    bool      visible       = true;
    float     scaleOverride = 0.0f;    // 0 = use node scale; >0 = override
    glm::vec3 colorOverride {1.0f};    // plume tint (RGB)
    float     intensityOverride = 1.0f; // plume emissive intensity

    // Material baked into vertex colors at load time.
    // Stored here for reference; not used at runtime.
    bool  isEmissive     = false;
    float emissiveScale  = 1.0f;
};

class GltfModel {
public:
    GltfModel() = default;

    // Load from a .glb or .gltf file.  Uploads all geometry to GPU immediately.
    // Throws std::runtime_error on failure.
    void load(const Context& ctx, GpuAllocator& allocator,
              MeshPipeline& pipeline, const std::filesystem::path& path);

    bool isLoaded() const { return !m_meshes.empty(); }

    // Per-node control.  No-ops if the name is not found.
    void setNodeVisible  (std::string_view name, bool visible);
    void setNodeScale    (std::string_view name, float scale);
    void setNodeColor    (std::string_view name, glm::vec3 color, float intensity);

    // Draw the full model with the given root model matrix.
    // Must be called inside an active render pass that is compatible with MeshPipeline.
    // vp: view-projection matrix.  rootModel: world transform for the model root.
    // sunDirWorld: unit vector toward the sun in world space.
    void draw(vk::CommandBuffer       cmd,
              const MeshPipeline&     pipeline,
              const glm::mat4&        vp,
              const glm::mat4&        rootModel,
              const glm::vec3&        sunDirWorld) const;

    // Bounding sphere radius in model space (useful for camera framing).
    float boundingRadius() const { return m_boundingRadius; }

    const std::vector<GltfNode>& nodes() const { return m_nodes; }

    // Returns the local-to-model-root transform for a named node,
    // or identity if not found.
    glm::mat4 nodeWorldTransform(std::string_view name) const;

private:
    // Recursively draw a node and its children.
    // plumePass: false = opaque geometry only, true = plumes only.
    // boundPipeline tracks the currently bound pipeline to avoid redundant rebinds.
    void drawNode(vk::CommandBuffer       cmd,
                  const MeshPipeline&     pipeline,
                  int                     nodeIdx,
                  const glm::mat4&        parentMvp,
                  const glm::mat4&        parentModel,
                  const glm::vec3&        sunDir,
                  bool                    plumePass,
                  bool&                   boundDoubleSided) const;

    std::vector<Mesh>         m_meshes;
    std::vector<GltfMaterial> m_materials;  // parallel to m_meshes (one per primitive)
    std::vector<GltfNode>     m_nodes;
    std::vector<int>          m_rootNodes;  // top-level node indices

    // name → node index for fast lookup.
    std::unordered_map<std::string, int> m_nameIndex;

    float m_boundingRadius = 1.0f;
};

} // namespace apeiron::render
