#include "apeiron/universe/BodyProperties.h"
#include "astro/SpiceCore.h"

#include <cspice/SpiceUsr.h>
#include <stdexcept>

namespace apeiron::universe {

BodyProperties BodyProperties::queryFromSpice(const std::string& naifName)
{
    SpiceInt    dim    = 0;
    SpiceDouble radii[3] = {};
    bodvrd_c(naifName.c_str(), "RADII", 3, &dim, radii);
    astro::Spice().checkError();
    return { radii[0], radii[2] }; // equatorial (a-axis), polar (c-axis), km
}

} // namespace apeiron::universe
