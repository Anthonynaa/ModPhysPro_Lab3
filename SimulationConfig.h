#ifndef SIMULATION_CONFIG_H
#define SIMULATION_CONFIG_H

namespace SimulationConfig {

constexpr float kWindowWidth = 800.0f;
constexpr float kWindowHeight = 600.0f;

constexpr float kOrdinaryFrequency = 60.0f;
constexpr float kFastFrequency = 240.0f;

constexpr float kSimulationScale = 1.0f;

constexpr float kProjectileSpeed = 1200.0f;
constexpr float kProjectileSize = 8.0f;

constexpr float kRestitution = 0.5f;

constexpr float kWallThickness = 10.0f;
constexpr float kBorderThickness = 10.0f;

constexpr int kOrdinaryBodyCount = 19;

}  // namespace SimulationConfig

#endif
