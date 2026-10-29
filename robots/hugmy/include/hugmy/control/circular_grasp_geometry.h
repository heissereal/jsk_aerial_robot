#ifndef HUGMY_CIRCULAR_GRASP_GEOMETRY_H
#define HUGMY_CIRCULAR_GRASP_GEOMETRY_H

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace hugmy {

// Cross-section geometry in the body y-z plane.  The model is intentionally
// constrained rather than a free three-point circle fit: Hugmy's front/rear
// arms are separated mainly along the cylinder axis, so they can provide only
// two independent points in the cross-section.  The existing bottom-contact
// point supplies the missing constraint and is treated as the top of the
// grasped circle along the measured gravity direction.
struct CircularGraspEstimate {
  bool valid = false;
  double radius = std::numeric_limits<double>::quiet_NaN();
  double center_y = std::numeric_limits<double>::quiet_NaN();
  double center_z = std::numeric_limits<double>::quiet_NaN();
  double rms = std::numeric_limits<double>::infinity();
  double confidence = 0.0;
  unsigned samples = 0;
  std::array<double, 4> depth_angle{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
};

// Geometric choice of the arm pair which is allowed to generate the recovery
// moment.  A valid pair depth requires both arms in that pair; one available
// arm is deliberately not treated as representative of its partner.
struct CircularRecoveryPairChoice {
  bool valid = false;
  size_t pair_index = 0;
  std::array<double, 2> pair_depth{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  bool used_gravity_or_roll_fallback = false;
};

inline double circularClamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

// Select the pair to actuate before evaluating rotor moments.  Depth is zero
// at the object top, pi/2 at its side and pi at its underside.
//
//  * If only one pair is below the side, actuate that pair.
//  * If both are above the side, actuate the deeper pair.
//  * If both are below the side, retain the deeper pair as the anchor and
//    actuate the shallower pair.
//
// When a complete depth is unavailable, downward_projection (larger means
// lower in gravity) is used only if both arms of both pairs are available.
// Otherwise the roll-derived lower pair is used.  Thus a single IMU angle can
// never decide the pair on its own.
inline CircularRecoveryPairChoice chooseCircularRecoveryPair(
    const std::array<double, 4>& depth_angle,
    const std::array<double, 4>& downward_projection,
    int roll_lower_pair,
    double side_angle = 0.5 * 3.14159265358979323846,
    double pair_depth_ambiguity = 0.0) {
  CircularRecoveryPairChoice choice;
  const std::array<std::array<size_t, 2>, 2> pairs{{
      std::array<size_t, 2>{{0, 1}},
      std::array<size_t, 2>{{2, 3}}}};
  std::array<double, 2> pair_projection{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  for (size_t pair = 0; pair < pairs.size(); ++pair) {
    const size_t a = pairs[pair][0], b = pairs[pair][1];
    if (std::isfinite(depth_angle[a]) && std::isfinite(depth_angle[b]))
      choice.pair_depth[pair] =
          0.5 * (depth_angle[a] + depth_angle[b]);
    if (std::isfinite(downward_projection[a]) &&
        std::isfinite(downward_projection[b]))
      pair_projection[pair] =
          0.5 * (downward_projection[a] + downward_projection[b]);
  }

  const bool depth0 = std::isfinite(choice.pair_depth[0]);
  const bool depth1 = std::isfinite(choice.pair_depth[1]);
  if (depth0 && depth1) {
    // A sub-degree depth difference is smaller than the uncertainty of the
    // Neuron-angle/circle reconstruction on the real arm.  Treating it as a
    // hard topological decision selected pair 0+1 at +21-deg roll although
    // both the measured roll and the signed moment required pair 2+3.  In an
    // ambiguous geometry, the measured body roll is the independent and
    // better-conditioned observation of which side is lower.
    const bool ambiguous = std::isfinite(pair_depth_ambiguity) &&
        pair_depth_ambiguity > 0.0 &&
        std::abs(choice.pair_depth[0] - choice.pair_depth[1]) <
            pair_depth_ambiguity;
    if (ambiguous && (roll_lower_pair == 0 || roll_lower_pair == 1)) {
      choice.pair_index = static_cast<size_t>(roll_lower_pair);
      choice.valid = true;
      choice.used_gravity_or_roll_fallback = true;
      return choice;
    }
    const bool below0 = choice.pair_depth[0] >= side_angle;
    const bool below1 = choice.pair_depth[1] >= side_angle;
    if (below0 != below1)
      choice.pair_index = below0 ? 0 : 1;
    else if (!below0)
      choice.pair_index = choice.pair_depth[0] >= choice.pair_depth[1] ? 0 : 1;
    else
      choice.pair_index = choice.pair_depth[0] <= choice.pair_depth[1] ? 0 : 1;
    choice.valid = true;
    return choice;
  }

  // Incomplete pair angles must not be averaged with a single sample.  Use
  // independent contact-point/gravity evidence if it is complete.
  if (std::isfinite(pair_projection[0]) &&
      std::isfinite(pair_projection[1])) {
    choice.pair_index = pair_projection[0] >= pair_projection[1] ? 0 : 1;
    choice.valid = true;
    choice.used_gravity_or_roll_fallback = true;
    return choice;
  }
  if (roll_lower_pair == 0 || roll_lower_pair == 1) {
    choice.pair_index = static_cast<size_t>(roll_lower_pair);
    choice.valid = true;
    choice.used_gravity_or_roll_fallback = true;
  }
  return choice;
}

// depth_angle is zero at the circle top, pi/2 at its side and pi at its
// underside.  Consequently depth_angle > pi/2 means that the contact lies
// below the side of the object, without overloading the word "bend angle".
inline CircularGraspEstimate estimateCircularGrasp(
    const std::array<std::array<double, 2>, 4>& contact_yz,
    const std::array<bool, 4>& contact_available,
    const std::array<double, 2>& surface_yz,
    const std::array<double, 2>& gravity_yz,
    double minimum_radius, double maximum_radius, double maximum_rms) {
  CircularGraspEstimate result;
  if (!std::isfinite(minimum_radius) || !std::isfinite(maximum_radius) ||
      !std::isfinite(maximum_rms) || minimum_radius <= 0.0 ||
      maximum_radius <= minimum_radius || maximum_rms <= 0.0)
    return result;
  const double gravity_norm = std::hypot(gravity_yz[0], gravity_yz[1]);
  if (!std::isfinite(gravity_norm) || gravity_norm < 1.0e-6) return result;
  const std::array<double, 2> down{{gravity_yz[0] / gravity_norm,
                                    gravity_yz[1] / gravity_norm}};
  std::vector<double> radii;
  radii.reserve(4);
  bool positive_side = false, negative_side = false;
  for (size_t arm = 0; arm < contact_yz.size(); ++arm) {
    if (!contact_available[arm] || !std::isfinite(contact_yz[arm][0]) ||
        !std::isfinite(contact_yz[arm][1])) continue;
    const double dy = contact_yz[arm][0] - surface_yz[0];
    const double dz = contact_yz[arm][1] - surface_yz[1];
    const double inward = dy * down[0] + dz * down[1];
    // A contact at or above the assumed top tangent does not constrain a
    // circle below the body and is rejected instead of producing a negative
    // or arbitrarily large radius.
    if (!std::isfinite(inward) || inward <= 1.0e-4) continue;
    const double radius = (dy * dy + dz * dz) / (2.0 * inward);
    if (!std::isfinite(radius) || radius < minimum_radius ||
        radius > maximum_radius) continue;
    radii.push_back(radius);
    const double lateral = dy * (-down[1]) + dz * down[0];
    positive_side = positive_side || lateral > 1.0e-4;
    negative_side = negative_side || lateral < -1.0e-4;
  }
  // Samples must cover both sides of the object.  Two front/rear samples on
  // only one side are not independent evidence of diameter.
  if (radii.size() < 2 || !positive_side || !negative_side) return result;
  std::sort(radii.begin(), radii.end());
  const size_t middle = radii.size() / 2;
  result.radius = radii.size() % 2 ? radii[middle]
      : 0.5 * (radii[middle - 1] + radii[middle]);
  result.center_y = surface_yz[0] + result.radius * down[0];
  result.center_z = surface_yz[1] + result.radius * down[1];
  result.samples = static_cast<unsigned>(radii.size());

  double squared_error = 0.0;
  unsigned residual_count = 0;
  for (size_t arm = 0; arm < contact_yz.size(); ++arm) {
    if (!contact_available[arm] || !std::isfinite(contact_yz[arm][0]) ||
        !std::isfinite(contact_yz[arm][1])) continue;
    const double radial_y = contact_yz[arm][0] - result.center_y;
    const double radial_z = contact_yz[arm][1] - result.center_z;
    const double distance = std::hypot(radial_y, radial_z);
    if (!std::isfinite(distance) || distance < 1.0e-6) continue;
    const double residual = distance - result.radius;
    squared_error += residual * residual;
    ++residual_count;
    const double top_projection =
        (radial_y * -down[0] + radial_z * -down[1]) / distance;
    result.depth_angle[arm] = std::acos(circularClamp(
        top_projection, -1.0, 1.0));
  }
  if (residual_count < 2) return result;
  result.rms = std::sqrt(squared_error / residual_count);
  const double coverage = circularClamp((result.samples - 1.0) / 3.0, 0.0, 1.0);
  const double residual_quality = circularClamp(
      1.0 - result.rms / maximum_rms, 0.0, 1.0);
  result.confidence = coverage * residual_quality;
  result.valid = result.radius >= minimum_radius &&
      result.radius <= maximum_radius && result.rms <= maximum_rms;
  return result;
}

}  // namespace hugmy

#endif
