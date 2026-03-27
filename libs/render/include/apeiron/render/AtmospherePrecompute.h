#pragma once

#include "apeiron/render/Image.h"

#include <vulkan/vulkan.hpp>
#include <glm/glm.hpp>
#include <filesystem>

namespace apeiron::render {

class Context;
class GpuAllocator;

// Precomputes the transmittance LUT for one planetary atmosphere and owns
// the resulting GPU texture.
//
// All parameters are in km.  On construction the compute shader is dispatched
// synchronously via GpuAllocator::immediateSubmit; the LUT is ready to sample
// by the time the constructor returns.
//
// The LUT is a 256×64 RGBA32F 2-D texture (R/G/B = transmittance per
// wavelength band, A = 1).  The mapping is:
//   u = normalised altitude  [0 = ground, 1 = top of atmosphere]
//   v = (cos zenith + 1) / 2 [0 = straight down, 1 = straight up]
class AtmospherePrecompute {
public:
    struct Params {
        float     groundRadius;    // km  (planet surface)
        float     atmosphereRadius; // km  (= groundRadius + thickness)
        glm::vec3 rayleigh;        // β_R  km⁻¹, RGB wavelength channels
        float     rayleighScaleH;  // H_R  km
        float     mieScattering;   // β_M  km⁻¹
        float     mieExtinction;   // β_Mext km⁻¹ (>= mieScattering)
        float     mieScaleH;       // H_M  km
        float     mieG;            // Henyey-Greenstein asymmetry [0,1)
    };

    AtmospherePrecompute(const Context&               ctx,
                         GpuAllocator&                allocator,
                         const std::filesystem::path& shaderDir,
                         const Params&                params);
    ~AtmospherePrecompute();

    AtmospherePrecompute(const AtmospherePrecompute&)            = delete;
    AtmospherePrecompute& operator=(const AtmospherePrecompute&) = delete;

    const Params&  params()      const { return m_params;   }
    vk::ImageView  lutView()     const { return m_lutView;  }
    vk::Sampler    lutSampler()  const { return m_sampler;  }

    // Width / height of the transmittance LUT.
    static constexpr uint32_t kLutW = 256;
    static constexpr uint32_t kLutH =  64;

private:
    void runPrecompute(const std::filesystem::path& shaderDir);

    const Context& m_ctx;
    Params         m_params;

    Image         m_lut;
    vk::ImageView m_lutView;
    vk::Sampler   m_sampler;
};

} // namespace apeiron::render
