#pragma once

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace aimer_node
{
enum class AirframeModel {four_independent, diagonal_coupled, fixed_movable};

inline AirframeModel parse_airframe_model(const std::string & value)
{
  if (value == "four_independent") {return AirframeModel::four_independent;}
  if (value == "diagonal_coupled") {return AirframeModel::diagonal_coupled;}
  if (value == "fixed_movable") {return AirframeModel::fixed_movable;}
  throw std::invalid_argument("unsupported airframe model: " + value);
}

struct FinAllocatorConfig
{
  AirframeModel model{AirframeModel::four_independent};
  std::array<double, 8> mixing_matrix{1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0, -1.0};
  std::array<double, 4> neutral_rad{};
  std::array<double, 4> direction{1.0, 1.0, 1.0, 1.0};
  std::array<double, 4> min_rad{-0.35, -0.35, -0.35, -0.35};
  std::array<double, 4> max_rad{0.35, 0.35, 0.35, 0.35};
  std::array<bool, 4> movable{true, true, true, true};
};

class FinAllocator
{
public:
  explicit FinAllocator(FinAllocatorConfig config) : config_(std::move(config))
  {
    for (std::size_t index = 0; index < config_.min_rad.size(); ++index) {
      if (config_.min_rad[index] > config_.neutral_rad[index] ||
        config_.neutral_rad[index] > config_.max_rad[index])
      {
        throw std::invalid_argument("neutral fin deflection is outside its limits");
      }
    }
  }

  std::array<double, 4> allocate(const double attack, const double roll) const
  {
    std::array<double, 4> output{};
    for (std::size_t fin = 0; fin < output.size(); ++fin) {
      if (!config_.movable[fin]) {
        output[fin] = config_.neutral_rad[fin];
        continue;
      }
      const auto control = config_.mixing_matrix[2 * fin] * attack +
        config_.mixing_matrix[2 * fin + 1] * roll;
      const auto value = config_.neutral_rad[fin] + config_.direction[fin] * control;
      output[fin] = std::clamp(value, config_.min_rad[fin], config_.max_rad[fin]);
    }
    return output;
  }

private:
  FinAllocatorConfig config_;
};
}  // namespace aimer_node
