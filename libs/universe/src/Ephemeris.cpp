#include "apeiron/universe/Ephemeris.h"
#include "astro/SpiceCore.h"

#include <cspice/SpiceUsr.h>

namespace apeiron::universe {

BodyState getState(
    const std::string&          target,
    const astro::EphemerisTime& et,
    const std::string&          observer,
    const std::string&          frame,
    const std::string&          abcorr)
{
    SpiceDouble starg[6];
    SpiceDouble lt;

    spkezr_c(
        target.c_str(),
        et.getETValue(),
        frame.c_str(),
        abcorr.c_str(),
        observer.c_str(),
        starg,
        &lt
    );

    astro::Spice().checkError();

    return BodyState{
        .position  = glm::dvec3(starg[0], starg[1], starg[2]),
        .velocity  = glm::dvec3(starg[3], starg[4], starg[5]),
        .lightTime = lt
    };
}

} // namespace apeiron::universe
