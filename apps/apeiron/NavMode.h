#pragma once

// Navigation modes — governs HUD layout, autopilot availability, and nav-console contents.
enum class NavMode {
    Orbit,    // orbital mechanics: prograde/retrograde markers, orbit autopilots available
    Docking,  // proximity/docking:  relative-velocity markers, target box, only killrot AP
};
