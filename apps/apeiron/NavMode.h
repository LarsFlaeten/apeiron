#pragma once

// Navigation modes — governs HUD layout, autopilot availability, and nav-console contents.
enum class NavMode {
    Orbit,    // orbital mechanics: prograde/retrograde markers, orbit autopilots available
    Docking,  // proximity/docking:  relative-velocity markers, target box, only killrot AP
    Mcc,      // mid-course correction: MCC direction marker, killrot + MCC DIR autopilots
};

// Translation authority — selectable from the nav console.
// Fine:       rcsThrust × 1.0  (precise proximity maneuvering)
// Aggressive: rcsThrust × dockingTransBoost  (faster translations, higher thruster throttle)
enum class TransMode { Fine, Aggressive };
