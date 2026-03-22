#include "apeiron/universe/KernelPool.h"
#include "astro/SpiceCore.h"

#include <algorithm>
#include <stdexcept>

namespace apeiron::universe {

KernelPool& KernelPool::instance()
{
    static KernelPool pool;
    return pool;
}

void KernelPool::load(const std::filesystem::path& path)
{
    astro::Spice().loadKernel(path.string());
    m_loaded.push_back(path.string());
}

void KernelPool::loadDirectory(const std::filesystem::path& dir)
{
    if (!std::filesystem::is_directory(dir))
        throw std::runtime_error("KernelPool: not a directory: " + dir.string());

    std::vector<std::filesystem::path> entries;
    for (const auto& e : std::filesystem::directory_iterator(dir))
        if (e.is_regular_file())
            entries.push_back(e.path());

    std::sort(entries.begin(), entries.end());

    for (const auto& p : entries)
        load(p);
}

const std::vector<std::string>& KernelPool::loaded() const
{
    return m_loaded;
}

} // namespace apeiron::universe
