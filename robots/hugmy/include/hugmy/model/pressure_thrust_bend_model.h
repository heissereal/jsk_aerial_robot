#ifndef HUGMY_PRESSURE_THRUST_BEND_MODEL_H
#define HUGMY_PRESSURE_THRUST_BEND_MODEL_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

#include <hugmy/model/pressure_thrust_bend_model_data.h>

namespace hugmy
{
class PressureThrustBendModel
{
public:
  static constexpr size_t PRESSURE_COUNT = bend_model_data::PRESSURE_COUNT;
  static constexpr size_t THRUST_COUNT = bend_model_data::THRUST_COUNT;

  enum class Trend : int8_t
  {
    DECREASING = -1,
    NEUTRAL = 0,
    INCREASING = 1,
  };

  double angleDeg(double pressure_kpa, double thrust_n) const
  {
    // When command history is unavailable, use the center of the measured
    // hysteresis loop rather than arbitrarily selecting one sweep direction.
    return angleDeg(pressure_kpa, thrust_n, Trend::NEUTRAL, Trend::NEUTRAL);
  }

  double angleDeg(double pressure_kpa, double thrust_n,
                  Trend pressure_trend, Trend thrust_trend) const
  {
    pressure_kpa = clamp(pressure_kpa, bend_model_data::pressure_grid.front(),
                        bend_model_data::pressure_grid.back());
    thrust_n = clamp(thrust_n, bend_model_data::thrust_grid.front(),
                     bend_model_data::thrust_grid.back());
    const size_t p1 = upperIndex(bend_model_data::pressure_grid, pressure_kpa);
    const size_t t1 = upperIndex(bend_model_data::thrust_grid, thrust_n);
    const size_t p0 = p1 == 0 ? 0 : p1 - 1;
    const size_t t0 = t1 == 0 ? 0 : t1 - 1;
    const double p_ratio = bend_model_data::pressure_grid[p1] > bend_model_data::pressure_grid[p0]
      ? (pressure_kpa - bend_model_data::pressure_grid[p0]) /
        (bend_model_data::pressure_grid[p1] - bend_model_data::pressure_grid[p0]) : 0.0;
    const double t_ratio = bend_model_data::thrust_grid[t1] > bend_model_data::thrust_grid[t0]
      ? (thrust_n - bend_model_data::thrust_grid[t0]) /
        (bend_model_data::thrust_grid[t1] - bend_model_data::thrust_grid[t0]) : 0.0;

    double sum = 0.0;
    size_t count = 0;
    for (size_t pressure_direction = 0; pressure_direction < 2; ++pressure_direction)
      {
        if (!matchesTrend(pressure_direction, pressure_trend)) continue;
        for (size_t thrust_direction = 0; thrust_direction < 2; ++thrust_direction)
          {
            if (!matchesTrend(thrust_direction, thrust_trend)) continue;
            const auto& table = bend_model_data::angle_grid_deg[pressure_direction][thrust_direction];
            const double lower = (1.0 - p_ratio) * table[p0][t0] +
                                 p_ratio * table[p1][t0];
            const double upper = (1.0 - p_ratio) * table[p0][t1] +
                                 p_ratio * table[p1][t1];
            sum += (1.0 - t_ratio) * lower + t_ratio * upper;
            ++count;
          }
      }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
  }

  double angleRad(double pressure_kpa, double thrust_n) const
  {
    return angleDeg(pressure_kpa, thrust_n) * 3.14159265358979323846 / 180.0;
  }

  double angleRad(double pressure_kpa, double thrust_n,
                  Trend pressure_trend, Trend thrust_trend) const
  {
    return angleDeg(pressure_kpa, thrust_n, pressure_trend, thrust_trend) *
           3.14159265358979323846 / 180.0;
  }

  // Compatibility with the original joint_model API.
  double f_theta(double pressure_kpa, double thrust_n) const
  {
    return angleDeg(pressure_kpa, thrust_n);
  }

  std::pair<std::pair<double, double>, std::pair<double, double>> ranges() const
  {
    return {{bend_model_data::pressure_grid.front(), bend_model_data::pressure_grid.back()},
            {bend_model_data::thrust_grid.front(), bend_model_data::thrust_grid.back()}};
  }

private:
  template <size_t N>
  static size_t upperIndex(const std::array<double, N>& grid, double value)
  {
    const auto it = std::lower_bound(grid.begin(), grid.end(), value);
    return it == grid.end() ? N - 1 : static_cast<size_t>(it - grid.begin());
  }

  static double clamp(double value, double lower, double upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  static bool matchesTrend(size_t direction_index, Trend trend)
  {
    if (trend == Trend::NEUTRAL) return true;
    return direction_index == (trend == Trend::INCREASING ? 1u : 0u);
  }
};
}

#endif
