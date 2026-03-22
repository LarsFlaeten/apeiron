#include <iostream>
#include <iomanip>
#include <string>
#include <filesystem>
#include <cstdlib>
#include <cmath>

#include "apeiron/universe/KernelPool.h"
#include "apeiron/universe/Ephemeris.h"
#include "astro/Time.h"
#include "astro/Exceptions.h"

namespace fs = std::filesystem;
using apeiron::universe::KernelPool;
using apeiron::universe::getState;

static void printUsage(const char* prog)
{
    std::cerr <<
        "Usage: " << prog << " --body TARGET --time TIMESTR [OPTIONS]\n"
        "\n"
        "Options:\n"
        "  --body     NAME    NAIF body name (e.g. MARS, EARTH, SUN)\n"
        "  --time     STRING  UTC time string (e.g. \"2000-01-01T12:00:00\")\n"
        "  --observer NAME    Observer body (default: SOLAR SYSTEM BARYCENTER)\n"
        "  --frame    NAME    Reference frame (default: J2000)\n"
        "  --abcorr   CODE    Aberration correction (default: NONE)\n"
        "  --kernels  DIR     Kernel directory (default: $APEIRON_DATA or ./data/kernels)\n"
        "\n"
        "Kernel subdirectories loaded: lsk/, spk/, pck/\n";
}

static fs::path resolveKernelDir(const std::string& override)
{
    if (!override.empty())
        return override;

    const char* env = std::getenv("APEIRON_DATA");
    if (env && *env)
        return fs::path(env) / "kernels";

    return fs::path("data/kernels");
}

static void loadKernels(const fs::path& base)
{
    auto& pool = KernelPool::instance();
    for (const char* sub : { "lsk", "spk", "pck" })
    {
        fs::path dir = base / sub;
        if (fs::is_directory(dir))
            pool.loadDirectory(dir);
    }
}

int main(int argc, char* argv[])
{
    std::string body, timestr, observer = "SOLAR SYSTEM BARYCENTER";
    std::string frame = "J2000", abcorr = "NONE", kernelDir;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << arg << "\n";
                std::exit(1);
            }
            return argv[++i];
        };

        if      (arg == "--body")     body      = next();
        else if (arg == "--time")     timestr   = next();
        else if (arg == "--observer") observer  = next();
        else if (arg == "--frame")    frame     = next();
        else if (arg == "--abcorr")   abcorr    = next();
        else if (arg == "--kernels")  kernelDir = next();
        else if (arg == "--help" || arg == "-h") { printUsage(argv[0]); return 0; }
        else { std::cerr << "Unknown option: " << arg << "\n"; printUsage(argv[0]); return 1; }
    }

    if (body.empty() || timestr.empty())
    {
        std::cerr << "Error: --body and --time are required.\n\n";
        printUsage(argv[0]);
        return 1;
    }

    try
    {
        fs::path kernels = resolveKernelDir(kernelDir);
        if (!fs::is_directory(kernels))
        {
            std::cerr << "Kernel directory not found: " << kernels
                      << "\nSet APEIRON_DATA or use --kernels.\n";
            return 1;
        }
        loadKernels(kernels);

        astro::EphemerisTime et = astro::EphemerisTime::fromString(timestr);
        auto state = getState(body, et, observer, frame, abcorr);

        double range = glm::length(state.position);
        double speed = glm::length(state.velocity);

        std::cout << std::scientific << std::setprecision(9);
        std::cout
            << "Body:       " << body           << "\n"
            << "Observer:   " << observer        << "\n"
            << "Frame:      " << frame           << "\n"
            << "Time (UTC): " << et.toISOUTCString() << "\n"
            << "Time (ET):  " << et.getETValue() << " s (seconds past J2000)\n"
            << "\n"
            << "Position [km]\n"
            << "  X = " << state.position.x << "\n"
            << "  Y = " << state.position.y << "\n"
            << "  Z = " << state.position.z << "\n"
            << "  |r| = " << range          << "\n"
            << "\n"
            << "Velocity [km/s]\n"
            << "  VX = " << state.velocity.x << "\n"
            << "  VY = " << state.velocity.y << "\n"
            << "  VZ = " << state.velocity.z << "\n"
            << "  |v| = " << speed            << "\n"
            << "\n"
            << "Light time [s]: " << state.lightTime << "\n";
    }
    catch (const astro::SpiceException& e)
    {
        std::cerr << "SPICE error: " << e.what() << "\n";
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
