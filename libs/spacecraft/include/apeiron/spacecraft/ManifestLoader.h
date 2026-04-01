#pragma once

#include "SpacecraftModel.h"
#include <filesystem>

namespace spacecraft {

// Load a SpacecraftModel from a TOML manifest file.
// Throws std::runtime_error on parse errors or missing required fields.
// Calls buildEffectivenessMatrix() before returning.
SpacecraftModel loadManifest(const std::filesystem::path& tomlPath);

} // namespace spacecraft
