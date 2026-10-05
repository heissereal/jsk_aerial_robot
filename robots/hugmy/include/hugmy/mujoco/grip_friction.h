#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace hugmy {
// Compliant tangential contact, limited by the existing Coulomb bound.
// Not a world pose constraint: below the bound a small displacement stores
// shear, above it the elastic displacement is returned to the yield surface.
struct GripFriction {
  using Vec = std::array<double, 3>;
  Vec displacement{};
  void reset() { displacement.fill(0.0); }
  Vec force(const Vec& tangent_velocity, const Vec& normal, double dt,
            double stiffness, double damping, double limit,
            double* requested_magnitude = nullptr) {
    Vec result{};
    if (!(limit > 0.0) || !std::isfinite(limit)) { reset(); return result; }
    // Remove the normal component when the contact frame rotates.
    double along_normal = 0.0;
    for (size_t i=0;i<3;++i) along_normal += displacement[i]*normal[i];
    double squared = 0.0;
    for (size_t i=0;i<3;++i) {
      displacement[i] -= along_normal*normal[i];
      displacement[i] += tangent_velocity[i]*std::max(0.0,std::min(dt,0.01));
      result[i] = -std::max(0.0,stiffness)*displacement[i] - damping*tangent_velocity[i];
      squared += result[i]*result[i];
    }
    const double requested = std::sqrt(squared);
    if (requested_magnitude) *requested_magnitude = requested;
    if (requested > limit) {
      for (size_t i=0;i<3;++i) {
        result[i] *= limit/requested;
        if (stiffness > 0.0)
          displacement[i] = -(result[i]+damping*tangent_velocity[i])/stiffness;
      }
    }
    if (!(stiffness > 0.0)) reset(); // k=0 reproduces the legacy viscous model
    return result;
  }
};
} // namespace hugmy
