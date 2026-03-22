#pragma once

#include <filesystem>
#include <vector>
#include <string>

namespace apeiron::universe {

/// Manages loading of SPICE kernels into the global SPICE pool.
/// Load all required kernels (LSK, SPK, PCK) before any ephemeris queries.
class KernelPool
{
public:
    static KernelPool& instance();

    /// Load a single kernel file into the SPICE pool.
    void load(const std::filesystem::path& path);

    /// Load all kernels under a directory tree (non-recursive, alphabetical order).
    void loadDirectory(const std::filesystem::path& dir);

    const std::vector<std::string>& loaded() const;

private:
    KernelPool() = default;
    std::vector<std::string> m_loaded;
};

} // namespace apeiron::universe
