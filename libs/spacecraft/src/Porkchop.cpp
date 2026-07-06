#include "apeiron/spacecraft/Porkchop.h"
#include "apeiron/spacecraft/Lambert.h"

#include <astro/SpiceCore.h>
#include <astro/State.h>
#include <astro/Time.h>
#include <astro/ReferenceFrame.h>

#include <cmath>
#include <algorithm>

namespace spacecraft {

// ---------------------------------------------------------------------------
double PorkchopData::depET(int iDep) const
{
    if (params.nDep <= 1) return params.t0;
    double dt = (params.t1 - params.t0) / (params.nDep - 1);
    return params.t0 + iDep * dt;
}

double PorkchopData::tofS(int iTof) const
{
    if (params.nTof <= 1) return params.tofMin;
    double dt = (params.tofMax - params.tofMin) / (params.nTof - 1);
    return params.tofMin + iTof * dt;
}

// ---------------------------------------------------------------------------
PorkchopData computePorkchop(const PorkchopParams& paramsIn,
                              std::function<void(float)> progress)
{
    PorkchopData out;
    out.params = paramsIn;

    // Query Sun GM from SPICE if not provided.
    if (out.params.muCentral <= 0.0) {
        double mu = 0.0;
        astro::Spice().getPlanetaryConstants(out.params.centralBody, "GM", mu);
        out.params.muCentral = mu;
    }
    const double mu = out.params.muCentral;

    const int nDep = out.params.nDep;
    const int nTof = out.params.nTof;
    const int total = nDep * nTof;

    out.dv1    .assign(total, PorkchopData::kNoSolution);
    out.dv2    .assign(total, PorkchopData::kNoSolution);
    out.dvTotal.assign(total, PorkchopData::kNoSolution);

    // All Lambert arcs are computed in ECLIPJ2000 to match the spacecraft
    // integrator frame (which is ECLIPJ2000).
    static const astro::ReferenceFrame kEclipJ2000 =
        astro::ReferenceFrame::createEclipJ2000();

    int done = 0;
    for (int iDep = 0; iDep < nDep; ++iDep) {
        const double etDep = out.depET(iDep);
        const astro::EphemerisTime etDepT(etDep);

        // Position + velocity at departure.
        // Override: use the caller-supplied state (e.g. ship parking orbit for
        // cislunar TLI) instead of the departure body centre from SPICE.
        astro::PosState dep;
        if (out.params.useDepartureOverride) {
            dep.r = out.params.departureR;
            dep.v = out.params.departureV;
        } else {
            astro::Spice().getRelativeGeometricState(
                out.params.departureBody, out.params.centralBody, etDepT, dep,
                kEclipJ2000);
        }

        for (int iTof = 0; iTof < nTof; ++iTof) {
            const double tofSec = out.tofS(iTof);
            const double etArr  = etDep + tofSec;
            const astro::EphemerisTime etArrT(etArr);

            // Position + velocity of arrival body at arrival ET.
            astro::PosState arr;
            if (out.params.arrivalOverrideFn) {
                arr = out.params.arrivalOverrideFn(etArr);
            } else {
                astro::Spice().getRelativeGeometricState(
                    out.params.arrivalBody, out.params.centralBody, etArrT, arr,
                    kEclipJ2000);
            }

            // Lambert solve.
            glm::dvec3 vDep, vArr;
            bool ok = solveLambert(mu,
                                   dep.r, arr.r,
                                   tofSec,
                                   /*prograde=*/true,
                                   vDep, vArr);

            if (ok) {
                const float d1 = static_cast<float>(glm::length(vDep - dep.v));
                const float d2 = static_cast<float>(glm::length(vArr - arr.v));
                const int idx = iTof * nDep + iDep;
                out.dv1    [idx] = d1;
                out.dv2    [idx] = d2;
                out.dvTotal[idx] = d1 + d2;

                out.dvMin = std::min(out.dvMin, d1 + d2);
                out.dvMax = std::max(out.dvMax, d1 + d2);
            }

            ++done;
        }

        if (progress)
            progress(static_cast<float>(done) / static_cast<float>(total));
    }

    return out;
}

} // namespace spacecraft
