#ifndef HUGMY_ROLL_RECOVERY_PLANNER_H
#define HUGMY_ROLL_RECOVERY_PLANNER_H

// Contact-conditioned roll recovery planning, independent of ROS and of the
// pressure/thrust actuator loops.  This deliberately does NOT estimate contact
// forces from pneumatic pressure.  A candidate supplies the vertices of its
// geometrically plausible support region.  Selection uses the least favourable
// signed roll lever over those vertices, rather than a pressure-weighted or
// averaged support point.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace hugmy {
namespace recovery {

using Vec3 = std::array<double, 3>;

inline Vec3 subtract(const Vec3& a, const Vec3& b) {
  return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {{a[1] * b[2] - a[2] * b[1],
           a[2] * b[0] - a[0] * b[2],
           a[0] * b[1] - a[1] * b[0]}};
}

inline double dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline bool finite(const Vec3& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// J_roll^T * F: the one-DOF roll Jacobian is e_x x (rotor - support).
// The axis, position, force direction and support must share one frame.
inline double rollLever(const Vec3& rotor, const Vec3& unit_force,
                        const Vec3& support) {
  return cross(subtract(rotor, support), unit_force)[0];
}

struct Quaternion {
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
};

struct Pose {
  Vec3 position{{0.0, 0.0, 0.0}};
  Quaternion rotation;
};

inline bool valid(const Quaternion& q) {
  const double n = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  return std::isfinite(n) && n > 1.0e-12;
}

inline Vec3 rotate(const Quaternion& q, const Vec3& p, bool inverse = false) {
  const double n = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  if (!std::isfinite(n) || n <= 1.0e-12)
    return {{NAN, NAN, NAN}};
  const double w = q.w / std::sqrt(n);
  const Vec3 v{{(inverse ? -q.x : q.x) / std::sqrt(n),
                (inverse ? -q.y : q.y) / std::sqrt(n),
                (inverse ? -q.z : q.z) / std::sqrt(n)}};
  const Vec3 t = cross(v, p);
  const Vec3 u = cross(v, t);
  return {{p[0] + 2.0 * (w*t[0] + u[0]),
           p[1] + 2.0 * (w*t[1] + u[1]),
           p[2] + 2.0 * (w*t[2] + u[2])}};
}

// Lock the object's centre in an inertial frame, never as a constant in the
// moving body frame.  Only usable if body pose and object share a frame.
inline Vec3 objectCenterWorld(const Vec3& initial_center_body,
                              const Pose& initial_body_pose) {
  const Vec3 offset = rotate(initial_body_pose.rotation, initial_center_body);
  return {{initial_body_pose.position[0] + offset[0],
           initial_body_pose.position[1] + offset[1],
           initial_body_pose.position[2] + offset[2]}};
}

inline Vec3 objectCenterBody(const Vec3& center_world,
                             const Pose& body_pose) {
  return rotate(body_pose.rotation,
                subtract(center_world, body_pose.position), true);
}

// The 2D instantaneous rotation centre is observable only when roll speed is
// appreciable.  Use this for offline motion validation, not near rest.
inline Vec3 instantaneousCenter(const Vec3& body_position,
                                const Vec3& body_velocity,
                                double roll_rate, double min_rate = 0.05) {
  if (!finite(body_position) || !finite(body_velocity) ||
      !std::isfinite(roll_rate) || std::abs(roll_rate) < min_rate)
    return {{NAN, NAN, NAN}};
  return {{body_position[0],
           body_position[1] - body_velocity[2] / roll_rate,
           body_position[2] + body_velocity[1] / roll_rate}};
}

struct Rotor {
  Vec3 position{{0.0, 0.0, 0.0}};
  Vec3 unit_force{{0.0, 0.0, 1.0}};
};

// One of {0+1 grasp, 0+1 extend, 2+3 grasp, 2+3 extend}.
// The actuator/shape estimator supplies a physically reachable rotor geometry
// and the thrust needed for extension.  Do not substitute alpha=0 instantly.
struct Candidate {
  std::size_t pair = 0;
  bool release = false;
  bool contact_feasible = false;
  // Diagnostic context only.  Candidate selection still uses the same
  // steady-state geometry and limits; these flags reveal when that geometry
  // was evaluated while pneumatic state was still moving toward its command.
  bool support_pressure_transitioning = false;
  bool actuated_pressure_transitioning = false;
  // Vertices of the possible support region in the body frame.  Since the
  // roll lever is affine in the support position, its minimum over the convex
  // hull occurs at one of these vertices; no interior sampling is necessary.
  // A zero count retains support_body as a backwards-compatible single-point
  // candidate for standalone callers.
  std::array<Vec3, 4> support_vertices{};
  std::size_t support_vertex_count = 0;
  // Centroid for diagnostics only.  It is not used for robust selection when
  // support_vertex_count is non-zero.
  Vec3 support_body{{0.0, 0.0, 0.0}};
  std::array<Rotor, 2> rotors;
  double extension_min_thrust = 0.0;  // Per rotor, N; 0 for grasped candidates.
  double transition_seconds = 0.0;
  double release_fraction = 0.0;
  // The circle/gravity adapter may admit a below-side grasp whose worst-case
  // support-point lever is uncertain.  It is then used only for a bounded
  // measured-response probe, never directly at full PI authority.
  bool conditional_grasp_probe = false;
  // Diagnostics for object-relative actuation. The first flag describes the
  // rotor application points; the signed centre lever also includes each
  // rotor's actual thrust direction about the inferred object centre.
  bool below_object_side = false;
  double signed_center_lever = NAN;
};

enum class EvaluationFailureReason {
  NONE = 0,
  SUPPORT_UNAVAILABLE,
  INVALID_INPUT,
  INVALID_ROTOR_GEOMETRY,
  EXTENSION_UNREACHABLE,
  WRONG_MOMENT_SIGN,
  LEVER_TOO_SMALL,
  THRUST_LIMIT
};

inline const char* evaluationFailureReasonName(
    EvaluationFailureReason reason) {
  switch (reason) {
    case EvaluationFailureReason::NONE: return "feasible";
    case EvaluationFailureReason::SUPPORT_UNAVAILABLE:
      return "support_unavailable";
    case EvaluationFailureReason::INVALID_INPUT: return "invalid_input";
    case EvaluationFailureReason::INVALID_ROTOR_GEOMETRY:
      return "invalid_rotor_geometry";
    case EvaluationFailureReason::EXTENSION_UNREACHABLE:
      return "extension_unreachable";
    case EvaluationFailureReason::WRONG_MOMENT_SIGN:
      return "wrong_moment_sign";
    case EvaluationFailureReason::LEVER_TOO_SMALL:
      return "lever_too_small";
    case EvaluationFailureReason::THRUST_LIMIT: return "thrust_limit";
  }
  return "unknown";
}

struct Config {
  double min_signed_lever = 0.005;       // m per 1 N commanded to each rotor
  double thrust_limit = 4.0;           // N per rotor
  // A support hypothesis farther from every physical rotor than the complete
  // mechanism can span is a frame/estimation fault, not a large useful
  // moment arm. Infinity preserves the standalone planner default; the
  // Hugmy adapter supplies a bound derived from the robot envelope.
  double max_support_rotor_distance = INFINITY;
  double minimum_command_thrust = 0.0; // N per rotor; calibrated actuator floor
  double grasp_probe_thrust = 0.8;     // N per rotor; response check only
  double release_penalty = 0.0;        // N equivalent, not an estimated force
  double transition_penalty = 0.0;     // N equivalent per second
  double switch_improvement = 0.1;     // N equivalent, hysteresis
};

struct Evaluation {
  bool feasible = false;
  EvaluationFailureReason failure_reason =
      EvaluationFailureReason::INVALID_INPUT;
  double signed_lever = NAN;
  double maximum_signed_lever = NAN;
  Vec3 worst_support_body{{NAN, NAN, NAN}};
  std::size_t support_vertex_count = 0;
  double roll_thrust = INFINITY;
  double command_thrust = INFINITY;
  double score = INFINITY;
  bool response_probe_required = false;
};

inline std::size_t supportVertexCount(const Candidate& c) {
  return c.support_vertex_count == 0 ? 1 :
      std::min(c.support_vertex_count, c.support_vertices.size());
}

inline Vec3 supportVertex(const Candidate& c, std::size_t index) {
  return c.support_vertex_count == 0 ? c.support_body :
      c.support_vertices[index];
}

template <typename RotorArray>
inline bool supportRotorGeometryPlausible(const Candidate& c,
                                          const RotorArray& rotors,
                                          double maximum_distance) {
  if (!std::isfinite(maximum_distance)) return true;
  if (maximum_distance <= 0.0) return false;
  const double maximum_distance_sq = maximum_distance * maximum_distance;
  for (std::size_t support_index = 0;
       support_index < supportVertexCount(c); ++support_index) {
    const Vec3 support = supportVertex(c, support_index);
    if (!finite(support)) return false;
    for (const auto& rotor : rotors) {
      if (!finite(rotor.position)) return false;
      const Vec3 span = subtract(rotor.position, support);
      if (dot(span, span) > maximum_distance_sq) return false;
    }
  }
  return true;
}

template <typename RotorArray>
inline bool robustSignedRollLever(const Candidate& c,
                                  const RotorArray& rotors,
                                  double moment_sign,
                                  double& minimum_signed_lever,
                                  double& maximum_signed_lever,
                                  Vec3& worst_support) {
  if (!c.contact_feasible || c.support_vertex_count > c.support_vertices.size())
    return false;
  minimum_signed_lever = std::numeric_limits<double>::infinity();
  maximum_signed_lever = -std::numeric_limits<double>::infinity();
  for (std::size_t support_index = 0;
       support_index < supportVertexCount(c); ++support_index) {
    const Vec3 support = supportVertex(c, support_index);
    if (!finite(support)) return false;
    double lever = 0.0;
    for (const auto& rotor : rotors)
      lever += rollLever(rotor.position, rotor.unit_force, support);
    const double signed_lever = moment_sign * lever;
    if (!std::isfinite(signed_lever)) return false;
    if (signed_lever < minimum_signed_lever) {
      minimum_signed_lever = signed_lever;
      worst_support = support;
    }
    maximum_signed_lever = std::max(maximum_signed_lever, signed_lever);
  }
  return std::isfinite(minimum_signed_lever) &&
      std::isfinite(maximum_signed_lever);
}

inline Evaluation evaluate(const Candidate& c, double required_moment,
                           const Config& cfg) {
  Evaluation e;
  if (!c.contact_feasible || c.support_vertex_count > c.support_vertices.size() ||
      (c.support_vertex_count == 0 && !finite(c.support_body))) {
    e.failure_reason = EvaluationFailureReason::SUPPORT_UNAVAILABLE;
    return e;
  }
  if (!std::isfinite(required_moment) || !std::isfinite(cfg.thrust_limit) ||
      cfg.thrust_limit <= 0.0 ||
      !std::isfinite(cfg.minimum_command_thrust) ||
      cfg.minimum_command_thrust < 0.0 ||
      cfg.minimum_command_thrust > cfg.thrust_limit ||
      !std::isfinite(c.transition_seconds) ||
      c.transition_seconds < 0.0 || !std::isfinite(c.release_fraction) ||
      c.release_fraction < 0.0 || c.release_fraction > 1.0) return e;
  if (!std::isfinite(c.extension_min_thrust) ||
      c.extension_min_thrust < 0.0) {
    e.failure_reason = c.release
        ? EvaluationFailureReason::EXTENSION_UNREACHABLE
        : EvaluationFailureReason::INVALID_INPUT;
    return e;
  }
  for (const auto& rotor : c.rotors) {
    if (!finite(rotor.position) || !finite(rotor.unit_force) ||
        std::abs(dot(rotor.unit_force, rotor.unit_force) - 1.0) > 0.02) {
      e.failure_reason = EvaluationFailureReason::INVALID_ROTOR_GEOMETRY;
      return e;
    }
  }
  if (!supportRotorGeometryPlausible(
          c, c.rotors, cfg.max_support_rotor_distance)) {
    e.failure_reason = EvaluationFailureReason::INVALID_ROTOR_GEOMETRY;
    return e;
  }
  if (!robustSignedRollLever(c, c.rotors,
          required_moment >= 0.0 ? 1.0 : -1.0,
          e.signed_lever, e.maximum_signed_lever,
          e.worst_support_body)) {
    e.failure_reason = EvaluationFailureReason::INVALID_ROTOR_GEOMETRY;
    return e;
  }
  e.support_vertex_count = supportVertexCount(c);
  // The minimum answers whether the sign is guaranteed for every plausible
  // support point.  A below-side grasp can still be useful when only some
  // support hypotheses are restorative.  Admit that uncertainty solely as a
  // low-authority response probe; maximum_signed_lever is not used to size
  // the final recovery thrust.
  const bool conditional_grasp = !c.release &&
      c.conditional_grasp_probe &&
      std::isfinite(e.maximum_signed_lever) &&
      e.maximum_signed_lever > 0.0 &&
      e.signed_lever < std::max(1.0e-9, cfg.min_signed_lever);
  if (conditional_grasp) {
    if (!std::isfinite(cfg.grasp_probe_thrust) ||
        cfg.grasp_probe_thrust <= 0.0) {
      e.failure_reason = EvaluationFailureReason::INVALID_INPUT;
      return e;
    }
    e.roll_thrust = INFINITY;
    e.command_thrust = std::max(cfg.minimum_command_thrust,
                                cfg.grasp_probe_thrust);
    if (e.command_thrust > cfg.thrust_limit) {
      e.failure_reason = EvaluationFailureReason::THRUST_LIMIT;
      return e;
    }
    e.score = e.command_thrust;
    e.response_probe_required = true;
    e.feasible = true;
    e.failure_reason = EvaluationFailureReason::NONE;
    return e;
  }
  if (e.signed_lever <= 0.0) {
    e.failure_reason = EvaluationFailureReason::WRONG_MOMENT_SIGN;
    return e;
  }
  if (e.signed_lever < std::max(1.0e-9, cfg.min_signed_lever)) {
    e.failure_reason = EvaluationFailureReason::LEVER_TOO_SMALL;
    return e;
  }
  e.roll_thrust = std::abs(required_moment) / e.signed_lever;
  // If ONE thrust input both straightens the arm and produces roll moment,
  // its two lower bounds are combined by max(), not blindly added.
  e.command_thrust = std::max(cfg.minimum_command_thrust,
      std::max(e.roll_thrust,
               c.release ? c.extension_min_thrust : 0.0));
  if (!std::isfinite(e.command_thrust) || e.command_thrust > cfg.thrust_limit) {
    e.failure_reason = EvaluationFailureReason::THRUST_LIMIT;
    return e;
  }
  e.score = e.command_thrust +
      cfg.release_penalty * c.release_fraction +
      cfg.transition_penalty * c.transition_seconds;
  e.feasible = std::isfinite(e.score);
  e.failure_reason = e.feasible ? EvaluationFailureReason::NONE
                                : EvaluationFailureReason::INVALID_INPUT;
  return e;
}

// Pressure and rotor thrust BOTH change the equilibrium arm geometry.  A
// released pair is therefore better evaluated by searching one common thrust
// command and recomputing its geometry through the measured bend lookup:
//   alpha_i = f_i(P_target, T),  tau_x = b(alpha_0, alpha_1) * T.
// This accounts for extension effort without adding an artificial second
// copy of the same rotor thrust.  The lookup is a steady-state approximation;
// the caller must separately supervise the pressure/shape transition.
template <typename GeometryAtThrust>
inline Evaluation evaluateCoupled(const Candidate& c, double required_moment,
                                  const Config& cfg,
                                  GeometryAtThrust geometry_at_thrust,
                                  double thrust_step = 0.1) {
  Evaluation result;
  if (!c.contact_feasible || c.support_vertex_count > c.support_vertices.size() ||
      (c.support_vertex_count == 0 && !finite(c.support_body))) {
    result.failure_reason = EvaluationFailureReason::SUPPORT_UNAVAILABLE;
    return result;
  }
  if (!std::isfinite(required_moment) || !std::isfinite(cfg.thrust_limit) ||
      cfg.thrust_limit <= 0.0 || !std::isfinite(thrust_step) ||
      !std::isfinite(cfg.minimum_command_thrust) ||
      cfg.minimum_command_thrust < 0.0 ||
      cfg.minimum_command_thrust > cfg.thrust_limit ||
      thrust_step <= 0.0 || !std::isfinite(c.release_fraction) ||
      c.release_fraction < 0.0 || c.release_fraction > 1.0 ||
      !std::isfinite(c.transition_seconds) || c.transition_seconds < 0.0)
    return result;
  if (!std::isfinite(c.extension_min_thrust) ||
      c.extension_min_thrust < 0.0) {
    result.failure_reason = EvaluationFailureReason::EXTENSION_UNREACHABLE;
    return result;
  }
  const double moment_sign = required_moment >= 0.0 ? 1.0 : -1.0;
  const int steps = static_cast<int>(std::ceil(cfg.thrust_limit / thrust_step));
  // A released candidate is a distinct physical topology, not merely the
  // zero-pressure command.  Start the common thrust search only after the
  // calibrated pressure/thrust/bend lookup predicts that both arms can reach
  // the configured extension shape.  The same thrust then supplies roll
  // moment, so this is a lower bound rather than an additive force cost.
  const double minimum_thrust = std::max(cfg.minimum_command_thrust,
      c.release ? c.extension_min_thrust : 0.0);
  if (minimum_thrust > cfg.thrust_limit) {
    result.failure_reason = EvaluationFailureReason::EXTENSION_UNREACHABLE;
    return result;
  }
  const int first_step = std::max(0, static_cast<int>(
      std::ceil((minimum_thrust - 1.0e-9) / thrust_step)));
  bool saw_valid_geometry = false;
  bool saw_restoring_sign = false;
  bool saw_usable_lever = false;
  for (int j = first_step; j <= steps; ++j) {
    const double thrust = std::min(cfg.thrust_limit, j * thrust_step);
    const auto rotors = geometry_at_thrust(thrust);
    bool geometry_valid = true;
    for (const auto& rotor : rotors) {
      geometry_valid = geometry_valid && finite(rotor.position) &&
          finite(rotor.unit_force) &&
          std::abs(dot(rotor.unit_force, rotor.unit_force) - 1.0) <= 0.02;
      if (!geometry_valid) break;
    }
    if (!geometry_valid) continue;
    if (!supportRotorGeometryPlausible(
            c, rotors, cfg.max_support_rotor_distance))
      continue;
    saw_valid_geometry = true;
    double signed_lever = NAN;
    double maximum_signed_lever = NAN;
    Vec3 worst_support{{NAN, NAN, NAN}};
    if (!robustSignedRollLever(c, rotors, moment_sign, signed_lever,
                              maximum_signed_lever, worst_support))
      continue;
    result.signed_lever = signed_lever;
    result.maximum_signed_lever = maximum_signed_lever;
    result.worst_support_body = worst_support;
    result.support_vertex_count = supportVertexCount(c);
    if (signed_lever > 0.0) saw_restoring_sign = true;
    if (signed_lever < std::max(1.0e-9, cfg.min_signed_lever)) continue;
    saw_usable_lever = true;
    if (thrust * signed_lever < std::abs(required_moment)) continue;
    result.feasible = true;
    result.failure_reason = EvaluationFailureReason::NONE;
    result.roll_thrust = thrust;
    result.command_thrust = thrust;
    result.score = thrust + cfg.release_penalty * c.release_fraction +
        cfg.transition_penalty * c.transition_seconds;
    break;  // Ascending thrust search: first feasible is least thrust.
  }
  if (!result.feasible) {
    if (!saw_valid_geometry)
      result.failure_reason =
          EvaluationFailureReason::INVALID_ROTOR_GEOMETRY;
    else if (!saw_restoring_sign)
      result.failure_reason = EvaluationFailureReason::WRONG_MOMENT_SIGN;
    else if (!saw_usable_lever)
      result.failure_reason = EvaluationFailureReason::LEVER_TOO_SMALL;
    else
      result.failure_reason = EvaluationFailureReason::THRUST_LIMIT;
  }
  return result;
}

struct Selection {
  bool valid = false;
  std::size_t index = 0;
  Evaluation evaluation;
  std::array<Candidate, 4> candidates;
  std::array<Evaluation, 4> alternatives;
};

inline Selection select(const std::array<Candidate, 4>& candidates,
                        double required_moment, const Config& cfg,
                        int previous_index = -1) {
  Selection result;
  result.candidates = candidates;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    result.alternatives[i] = evaluate(candidates[i], required_moment, cfg);
    if (result.alternatives[i].feasible &&
        (!result.valid || result.alternatives[i].score < result.evaluation.score)) {
      result.valid = true;
      result.index = i;
      result.evaluation = result.alternatives[i];
    }
  }
  if (result.valid && previous_index >= 0 && previous_index < 4 &&
      result.alternatives[previous_index].feasible &&
      result.alternatives[previous_index].score <=
          result.evaluation.score + cfg.switch_improvement) {
    result.index = static_cast<std::size_t>(previous_index);
    result.evaluation = result.alternatives[result.index];
  }
  return result;
}

// Predict whether roll will reach the unsafe angle before an extension can
// take effect.  The rate must be moving away from the target; angle alone
// must not trigger an emergency.  The caller owns the emergency mode latch.
inline bool imminentOutwardFall(double error, double error_rate,
                               double unsafe_abs_error, double response_seconds,
                               double margin_seconds) {
  if (!std::isfinite(error) || !std::isfinite(error_rate) ||
      !std::isfinite(unsafe_abs_error) || !std::isfinite(response_seconds) ||
      !std::isfinite(margin_seconds) || unsafe_abs_error <= 0.0 ||
      response_seconds < 0.0 || margin_seconds < 0.0)
    return false;
  const double outward_rate = (error >= 0.0 ? 1.0 : -1.0) * error_rate;
  if (outward_rate <= 1.0e-5) return false;
  if (std::abs(error) >= unsafe_abs_error) return true;
  return (unsafe_abs_error - std::abs(error)) / outward_rate <=
      response_seconds + margin_seconds;
}

}  // namespace recovery
}  // namespace hugmy

#endif  // HUGMY_ROLL_RECOVERY_PLANNER_H
