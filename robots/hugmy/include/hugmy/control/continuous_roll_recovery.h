#ifndef HUGMY_CONTINUOUS_ROLL_RECOVERY_H
#define HUGMY_CONTINUOUS_ROLL_RECOVERY_H
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>

namespace hugmy {
inline double recoveryClamp(double x, double low, double high) {
  return std::max(low, std::min(high, x));
}
// Spinal p,q,r are body rates, not Euler angle derivatives. Fail closed near
// the ZYX singularity; do not hide a singularity by clamping tan(pitch).
inline double recoveryRollRate(double roll, double pitch,
                               const std::array<double, 3>& gyro) {
  if (!std::isfinite(roll) || !std::isfinite(pitch) || std::abs(std::cos(pitch)) < 0.25)
    return std::numeric_limits<double>::quiet_NaN();
  return gyro[0] + std::tan(pitch) *
      (std::sin(roll) * gyro[1] + std::cos(roll) * gyro[2]);
}
struct RollRecoveryConfig {
  bool learn_response = true;
  double angle_gain = 1.0; // (rad/s)/rad
  double max_target_rate = 0.12; // rad/s; distinct from emergency limit
  double rate_kp = 4.0, rate_ki = 3.0; // N/(rad/s), N/rad
  double integral_rate = 0.30; // N/s; the ONE integrator's slew ceiling
  double stop = 0.08726646, horizon = 0.20;
  double settle_rate = 0.05, settle_sec = 0.15;
  double stop_margin = 0.0174533; // 1 deg measurement/contact tolerance
  double limit = 4.0, rise = 1.5, fall = 6.0;
  double fallback_thrust = 0.5, min_lever = 0.015;
  double start_rate = 0.05, detect_thrust = 0.5;
  double confirm_sec = 0.15, confirm_angle = 0.0174533;
  double progress_angle = 0.00872665, stall_sec = 0.50;
  double progress_window = 1.0; // compare averaged ends of the recent window
  double abort_sec = 10.0, max_rate = 0.8; // controller overrides this with the configured 30-s timeout
  double response_delay = 0.20, response_window = 0.40;
  double stable_pressure = 1.0, stable_bend = 0.035;
  double min_gain = 0.02, max_gain = 0.25; // local (rad/s)/N, not physical inertia
};
struct RollRecoveryModel {
  double moment = 0.0, lever = 0.0; // projected into selected correction direction
  bool valid = false;
  std::array<double, 9> context{}; // four pressures, bottom, four measured bends
};
// Separate instances for positive/negative roll. This estimates a local
// command-to-motion response, NOT friction or the physical contact point.
struct RollRecoveryResponse {
  double gain = 0.08;
  unsigned samples = 0;
  std::array<double, 9> context{};
  bool sameContext(const std::array<double, 9>& c, const RollRecoveryConfig& cfg) const {
    for (size_t i = 0; i < c.size(); ++i)
      if (!std::isfinite(c[i]) || std::abs(c[i] - context[i]) >
          3.0 * (i < 5 ? cfg.stable_pressure : cfg.stable_bend)) return false;
    return true;
  }
  bool compatible(const std::array<double, 9>& c, const RollRecoveryConfig& cfg) const {
    return samples >= 3 && sameContext(c, cfg);
  }
};
struct ContinuousRollRecovery {
  double thrust = 0.0, hold = 0.0, boost = 0.0;
  double target_rate = 0.0, filtered_rate = 0.0, feedforward = 0.0;
  double target_thrust = 0.0, effective_kp = 0.0;
  double braking_horizon = 0.0;
  double best_error = 0.0, last_progress = 0.0, last_update = 0.0;
  double candidate_start = -1.0, candidate_error = 0.0;
  double stop_band_since = -1.0;
  double stable_since = -1.0, last_learning = -1.0;
  bool moving = false, finished = false, fault = false, stalled = false;
  bool integrating = false, model_valid = false, learned_response = false;
  bool rate_initialized = false;
  bool speed_braking = false, predicted_braking = false;
  bool timed_out = false, feedforward_initialized = false;
  double running_feedforward = 0.0;
  std::array<double, 9> stable_context{};
  struct Sample { double time, command, rate; };
  std::deque<Sample> history;
  struct ErrorSample { double time, error; };
  std::deque<ErrorSample> progress_history;
  double recent_progress = 0.0;
  // Pneumatic regrasp changes the arm bend and therefore the recovery lever.
  // Speed/prediction braking is only a rotor-thrust decision: it may occur far
  // from the attitude target and must not authorize a return to grasp pressure.
  // Keep the selected pair released until recovery has actually completed or
  // the controller is deliberately handing an abort/timeout to the grasp state.
  bool pressureHandoffReady() const { return finished || timed_out || fault; }
  void reset(double error, double now) {
    *this = ContinuousRollRecovery(); best_error = std::abs(error);
    last_progress = last_update = now;
    progress_history.push_back({now, error});
  }
  void updateProgress(double error, double now, const RollRecoveryConfig& c) {
    // Keep best_error for diagnostics only: returning from a worse posture
    // is progress even before the initial/best-ever error is reached again.
    best_error = std::min(best_error, std::abs(error));
    if (!progress_history.empty() && progress_history.back().time == now)
      progress_history.back().error = error;
    else progress_history.push_back({now, error});
    const double begin = now - c.progress_window;
    while (progress_history.size() > 1 && progress_history[1].time <= begin)
      progress_history.pop_front();
    recent_progress = 0.0;
    if (progress_history.front().time > begin + 1.0e-9) return;
    // Average the first/last quarter instead of comparing two noisy IMU
    // samples. Repeated small jitter must not indefinitely renew the timer.
    double first = 0.0, last = 0.0;
    size_t first_count = 0, last_count = 0;
    for (const auto& sample : progress_history) {
      if (sample.time < begin - 1.0e-9) continue;
      if (sample.time <= begin + 0.25 * c.progress_window) {
        first += sample.error; ++first_count;
      }
      if (sample.time >= now - 0.25 * c.progress_window) {
        last += sample.error; ++last_count;
      }
    }
    if (!first_count || !last_count) return;
    recent_progress = first / first_count - last / last_count;
    if (recent_progress >= c.progress_angle) last_progress = now;
  }
  void observe(double now, const RollRecoveryModel& m, const RollRecoveryConfig& c,
               RollRecoveryResponse& response, bool allow_learning) {
    bool stable = stable_since >= 0.0;
    for (size_t i = 0; i < m.context.size(); ++i)
      stable = stable && std::isfinite(m.context[i]) &&
          std::abs(m.context[i] - stable_context[i]) <=
              (i < 5 ? c.stable_pressure : c.stable_bend);
    if (!stable || !allow_learning) {
      stable_since = now; stable_context = m.context; history.clear();
    }
    history.push_back({now, thrust, filtered_rate});
    const double span = c.response_delay + c.response_window;
    while (history.size() > 2 && history[1].time < now - span - 0.1) history.pop_front();
    if (!allow_learning || now - stable_since < span ||
        now - last_learning < c.response_window) return;
    auto at = [&](double time) {
      Sample result = history.front();
      for (const Sample& s : history) { if (s.time > time) break; result = s; }
      return result;
    };
    const Sample old_rate = at(now - c.response_window);
    const Sample old_input = at(now - span);
    const Sample input = at(now - c.response_delay);
    if (std::abs(old_input.time - (now - span)) > 0.05 ||
        std::abs(input.time - (now - c.response_delay)) > 0.05) return;
    const double df = input.command - old_input.command;
    const double dw = filtered_rate - old_rate.rate;
    if (std::abs(df) < 0.10 || std::abs(dw) < 0.005 || df * dw <= 0.0) return;
    const double gain = dw / df;
    if (!std::isfinite(gain) || gain < c.min_gain || gain > c.max_gain) return;
    if (response.samples && !response.sameContext(m.context, c)) response = RollRecoveryResponse();
    response.gain = recoveryClamp(response.gain +
        recoveryClamp(0.1 * (gain - response.gain), -0.1 * response.gain, 0.1 * response.gain),
        c.min_gain, c.max_gain);
    response.context = m.context; ++response.samples; last_learning = now;
  }
  double update(double error, double omega, double now, double dt,
                const RollRecoveryModel& m, const RollRecoveryConfig& c,
                RollRecoveryResponse& response,
                double predictive_target = std::numeric_limits<double>::quiet_NaN()) {
    integrating = false;
    speed_braking = predicted_braking = false;
    if (fault) return thrust = 0.0;
    if (!std::isfinite(error) || !std::isfinite(omega) || !std::isfinite(now) ||
        !std::isfinite(dt) || dt < 0.0 || now < last_update || now - last_update > 0.25 ||
        std::abs(omega) >= c.max_rate || !std::isfinite(c.fall) || c.fall <= 0.0 ||
        !std::isfinite(c.max_target_rate) || c.max_target_rate <= 0.0 ||
        !std::isfinite(c.progress_window) || c.progress_window <= 0.0 ||
        !std::isfinite(c.progress_angle) || c.progress_angle <= 0.0) {
      fault = true; boost = 0.0; return thrust = 0.0;
    }
    dt = std::min(0.1, dt); last_update = now;
    filtered_rate = rate_initialized ? filtered_rate + dt / (0.08 + dt) * (omega - filtered_rate) : omega;
    rate_initialized = true;
    if (finished) {
      boost = 0.0;
      return thrust = std::max(0.0, thrust - c.fall * dt);
    }
    // Entering the angle band is not enough: c14 reached +/-3 deg with
    // roughly 0.2 rad/s still present, immediately changed to RECOVER, and
    // coasted through to the opposite side while the pneumatic grasp was
    // rebuilding.  Brake to zero and require a short low-rate dwell before
    // declaring success.  A crossed target outside the band is deliberately
    // not success; the caller may reselect the opposite correcting pair.
    // Enter the capture state only after reaching the actual +/-stop target.
    // The old stop+margin entry cut thrust at about 3.6--4 deg, erased the
    // integral, and repeatedly rebounded to 6--12 deg.  The margin is now
    // hysteresis after entry, not an early stopping threshold.
    if (std::abs(error) <= c.stop ||
        (stop_band_since >= 0.0 && std::abs(error) <= c.stop + c.stop_margin)) {
      updateProgress(std::abs(error), now, c);
      target_rate = target_thrust = 0.0;
      speed_braking = predicted_braking = true;
      thrust = std::max(0.0, thrust - c.fall * dt);
      if (std::abs(filtered_rate) <= c.settle_rate) {
        if (stop_band_since < 0.0) stop_band_since = now;
        if (now - stop_band_since >= c.settle_sec) {
          finished = true;
          boost = 0.0;
        }
      } else stop_band_since = -1.0;
      return thrust;
    }
    stop_band_since = -1.0;
    updateProgress(error, now, c);
    if (!moving) {
      // Once thrust has started a plausible body response, allow the
      // confirmation window to finish while predictive braking ramps thrust
      // down. Requiring thrust>=detect_thrust on every sample reset the
      // candidate exactly when a fast, genuine correction began braking.
      if (candidate_start < 0.0 && thrust >= c.detect_thrust &&
          filtered_rate >= c.start_rate) {
        candidate_start = now; candidate_error = error;
      }
      if (candidate_start >= 0.0 && filtered_rate >= 0.0 &&
          error <= candidate_error + c.confirm_angle) {
        if (now - candidate_start >= c.confirm_sec && candidate_error - error >= c.confirm_angle) {
          moving = true; hold = thrust; // breakaway diagnostic, never latched as command
          last_progress = now; best_error = std::abs(error);
        }
      } else candidate_start = -1.0;
    }
    stalled = now - last_progress >= c.stall_sec && filtered_rate < c.start_rate;
    if (timed_out || now - last_progress >= c.abort_sec) {
      // A no-progress timeout is a controlled handoff to pneumatic grasp, not
      // an emergency motor cut.  An instantaneous 2--4 N -> 0 N transition
      // released the loaded arm and its rebound increased roll.  The caller
      // starts regrasp as soon as timed_out becomes true; keep this state alive
      // while the normal fall-rate removes thrust, then report the timeout.
      timed_out = true;
      target_rate = target_thrust = 0.0;
      speed_braking = predicted_braking = true;
      boost = 0.0;
      thrust = std::max(0.0, thrust - c.fall * dt);
      if (thrust <= 1.0e-6) fault = true;
      return thrust;
    }
    // Include the time needed to actually remove the current command.
    // A fixed 0.2-s prediction missed up to 4/6=0.67 s of thrust rundown.
    // This is kinematic lookahead, not an identified stopping distance:
    // contact changes and rotor lag can still alter the motion.
    const double helpful_rate = std::max(0.0, std::max(omega, filtered_rate));
    braking_horizon = c.horizon + std::max(0.0, thrust) / c.fall;
    const double predicted = error - helpful_rate * braking_horizon;
    const double remaining = std::max(0.0, error - c.stop);
    // Prediction limits velocity smoothly instead of toggling between full
    // speed and zero at the stop-band edge. Aim at zero; +/-stop is success.
    const double approach_rate = std::max(0.0, error) / std::max(0.05, braking_horizon);
    target_rate = std::min(c.max_target_rate, std::min(c.angle_gain * std::max(0.0, error), approach_rate));
    speed_braking = helpful_rate > target_rate;
    predicted_braking = predicted <= 0.0;
    model_valid = m.valid && std::isfinite(m.moment) && std::isfinite(m.lever) && m.lever >= c.min_lever;
    // Blend near the geometry-validity boundary: no 0.5 -> 4 N switch.
    const double confidence = model_valid ? recoveryClamp(
        (m.lever - c.min_lever) / c.min_lever, 0.0, 1.0) : 0.0;
    const double nominal = model_valid ? recoveryClamp(m.moment / m.lever, 0.0, c.limit) : c.fallback_thrust;
    const double requested_ff = c.fallback_thrust + confidence * (nominal - c.fallback_thrust);
    if (!feedforward_initialized) {
      running_feedforward = requested_ff; feedforward_initialized = true;
    } else if (!moving) {
      running_feedforward += recoveryClamp((requested_ff - running_feedforward) * dt / (0.25 + dt),
                                           -c.rise * dt, c.rise * dt);
    }
    // Geometry initializes recovery. Once motion confirms, keep this baseline
    // independent of flex/rebend; the single rate PI still adjusts thrust.
    feedforward = running_feedforward;
    observe(now, m, c, response, c.learn_response && moving && !stalled && !speed_braking &&
        filtered_rate >= 0.02 && model_valid && remaining > 0.0 &&
        target_rate >= 0.9 * c.max_target_rate && thrust < c.limit - 0.1);
    learned_response = c.learn_response && model_valid && response.compatible(m.context, c);
    const double requested_kp = learned_response ? recoveryClamp(1.0 / response.gain, 2.0, 10.0) : c.rate_kp;
    if (effective_kp == 0.0) effective_kp = c.rate_kp;
    effective_kp += recoveryClamp(requested_kp - effective_kp, -dt, dt);
    // A low-pass lag must not integrate UP while the body is already fast.
    const double rate_error = target_rate - (speed_braking ? helpful_rate : filtered_rate);
    const double base = feedforward + effective_kp * rate_error;
    const double unclamped = base + boost;
    double delta = recoveryClamp(c.rate_ki * rate_error, -c.integral_rate, c.integral_rate) * dt;
    // Anti-windup for both amplitude and actuator slew saturation. This is
    // the ONLY integral: old fixed stall boosts are not superimposed.
    if ((delta > 0.0 && (unclamped >= c.limit || unclamped > thrust + c.rise * dt)) ||
        (delta < 0.0 && (unclamped <= 0.0 || unclamped < thrust - c.fall * dt))) delta = 0.0;
    // Predictive mode replaces the nominal FF+PI target, never adds to it.
    // Freeze the unused PI integral; all actual-rate/slew/fault guards below
    // still apply to the planner exactly as they do to the baseline.
    if(std::isfinite(predictive_target)) delta = 0.0;
    boost = recoveryClamp(boost + delta, -c.limit, c.limit); integrating = delta != 0.0;
    target_thrust = recoveryClamp(base + boost, 0.0, c.limit);
    if(std::isfinite(predictive_target))
      target_thrust = recoveryClamp(predictive_target, 0.0, c.limit);
    if (speed_braking) {
      // Priority over model FF and retained I, before motion confirms too.
      // Reduce smoothly in proportion to overspeed, not an OFF pulse or an
      // opposite-pair command. Raw or filtered overspeed suffices.
      const double fraction = recoveryClamp((helpful_rate - target_rate) / c.max_target_rate, 0.0, 1.0);
      const double speed_cap = std::max(0.0, thrust - c.fall * fraction * dt);
      target_thrust = std::min(target_thrust, speed_cap);
    }
    // Predictive braking must NOT erase learned contact support. Resetting I
    // caused re-bend/stall/rebuilding cycles in c14. Finish/fault resets it.
    if (predicted_braking) target_thrust = 0.0;
    thrust += recoveryClamp(target_thrust - thrust, -c.fall * dt, c.rise * dt);
    return thrust = recoveryClamp(thrust, 0.0, c.limit);
  }
};
} // namespace hugmy
#endif
