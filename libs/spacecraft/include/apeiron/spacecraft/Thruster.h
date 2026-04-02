#pragma once

#include <glm/glm.hpp>
#include <string>

namespace spacecraft {

enum class ThrusterType { MainEngine, Auxiliary, RCS };

struct Thruster {
    std::string   id;
    ThrusterType  type;
    int           quad        = 0;      // RCS quad (1-based); 0 = not applicable
    float         thrustN     = 0.0f;   // max thrust, Newtons
    float         ispS        = 0.0f;   // specific impulse, seconds
    glm::vec3     position    {};       // model space, metres from origin
    glm::vec3     direction   {};       // unit thrust vector in model space
    std::string   refNode;              // glTF node name of the ref empty
    std::string   exhaustNode;             // glTF node name of the plume mesh
    float         exhaustScale  = 1.0f;  // plume scale at full throttle
    glm::vec3     plumeColor    {1.0f};  // RGB tint for the plume shader
    float         plumeIntensity = 1.0f; // emissive brightness multiplier

    // Runtime state — set each tick by the control allocator.
    float throttle  = 0.0f;  // [0, 1] duty cycle from pseudoinverse
    float pwmPhase  = 0.0f;  // [0, pwmPeriod) phase accumulator
    bool  firing    = false; // true when valve is open this tick
};

} // namespace spacecraft
