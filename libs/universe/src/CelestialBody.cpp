#include "apeiron/universe/CelestialBody.h"
#include "apeiron/universe/Ephemeris.h"
#include "astro/SpiceCore.h"

#include <cspice/SpiceUsr.h>

namespace apeiron::universe {

CelestialBody::CelestialBody(std::string name, std::string naifName,
                              double radiusKm, std::string observer,
                              std::string frame)
    : SceneNode(std::move(name))
    , m_naifName (std::move(naifName))
    , m_observer (std::move(observer))
    , m_frame    (std::move(frame))
    , m_radiusKm (radiusKm)
{}

const std::string& CelestialBody::naifName() const { return m_naifName; }
const std::string& CelestialBody::observer() const { return m_observer; }
double             CelestialBody::radiusKm()  const { return m_radiusKm; }

void CelestialBody::update(const astro::EphemerisTime& et)
{
    // Position from ephemeris.
    auto state = getState(m_naifName, et, m_observer, m_frame);
    setPosition(state.position);

    // Orientation: rotate body-fixed frame (IAU_<name>) into the display frame.
    // pxform_c returns a row-major 3×3 matrix M such that v_display = M * v_body.
    // Most PCK bodies have an IAU frame; if not (e.g. barycenters), use identity.
    std::string iauFrame = "IAU_" + m_naifName;
    SpiceDouble m[3][3];
    pxform_c(iauFrame.c_str(), m_frame.c_str(), et.getETValue(), m);

    if (!failed_c()) {
        // SPICE is row-major; glm is column-major — transpose on load.
        glm::mat3 rot;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                rot[col][row] = static_cast<float>(m[row][col]);
        setOrientation(rot);
    } else {
        reset_c(); // clear SPICE error so subsequent calls are unaffected
        setOrientation(glm::mat3(1.0f));
    }
}

} // namespace apeiron::universe
