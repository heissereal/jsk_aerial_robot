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
  // Dynamic phase flag set by the outer recovery state machine.  It is not a
  // user mode: it tells the common FF+rate-PI law to keep increasing bounded
  // breakaway authority until outward roll has actually been arrested.
  bool rate_arrest_mode = false;
  double angle_gain = 0.65; // (rad/s)/rad; gentler final approach near the roll target
  double max_target_rate = 0.12; // rad/s; distinct from emergency limit
  // Include only bounded, filtered acceleration in stopping prediction so a
  // stick-slip breakaway is seen before velocity alone reaches the target.
  double max_prediction_acceleration = 2.0; // rad/s^2
  // Detect contact breakaway from the filtered whole-body angular
  // acceleration before the slower retained-angle detector completes.  Once
  // detected, outward-rate arrest must stop searching for more authority and
  // retain only the force which actually started the motion.
  double breakaway_acceleration = 0.50; // rad/s^2
  double breakaway_thrust_margin = 0.15; // N
  // Contact breakaway acceleration is short lived.  Extrapolating it over
  // the complete rotor-rundown horizon predicted a target crossing while the
  // body was still more than 20 deg away.  Limit only the acceleration term
  // to this short horizon; velocity still uses the full rundown time.
  double prediction_acceleration_horizon = 0.20; // s
  double rate_kp = 4.0, rate_ki = 3.0; // N/(rad/s), N/rad
  double integral_rate = 0.30; // N/s; the ONE integrator's slew ceiling
  double stop = 0.08726646, horizon = 0.20;
  double settle_rate = 0.05, settle_sec = 0.15;
  double stop_margin = 0.0174533; // 1 deg measurement/contact tolerance
  // Multiplier used only for the rotor-to-pneumatic handoff boundary when no
  // wider contact minimum is configured. A flexed contact may hand off early
  // to avoid a lever-sign reversal; a released pair can set this to zero and
  // use its large free-space lever all the way to the requested stop angle.
  double capture_margin_factor = 2.0;
  // Optional measured-contact equilibrium accepted after real improvement.
  // Zero retains the legacy stop+2*margin rule. Action C sets this to 12 deg
  // so it does not drive through a useful soft-contact lever merely to chase
  // the nominal 8-deg resume band.
  double contact_minimum_limit = 0.0;
  double limit = 4.0, rise = 1.5, fall = 6.0;
  // Optional lower bound on the model feed-forward.  The ordinary angle
  // return leaves this at zero.  Outward-rate arrest sets it to the measured
  // safe breakaway floor so a small/transitioning lever estimate cannot
  // reduce emergency authority below that floor.
  double minimum_feedforward = 0.0;
  // Faster slew used after stopping-distance prediction says that the current
  // motion will cross the target. The command is tapered monotonically to
  // zero; a residual error is handled by a fresh measured-contact pass.
  double terminal_brake_fall = 8.0;
  // Far from level, a stopping-distance prediction is only a continuous
  // speed limit.  Reserve the latched rotor rundown for the final envelope;
  // otherwise contact-rate impulses repeatedly unload and reload the arm.
  double terminal_brake_max_error = 0.08726646; // 5 deg for a 3-deg stop band
  // Initial authority for a non-terminal, explicitly reset re-approach.
  double reapproach_limit = 1.5, reapproach_rate = 0.04;
  // Start a post-brake retry gently, but do not hold it below static-friction
  // breakaway forever.  If retained roll does not improve, grow the retry
  // authority monotonically toward the ordinary recovery limit.
  double reapproach_authority_rise = 0.8; // N/s
  double reapproach_escalation_delay = 0.50; // s
  double fallback_thrust = 0.5, min_lever = 0.015;
  double start_rate = 0.05, detect_thrust = 0.5;
  // Do not identify a wrong-sign contact response from the pressure-release
  // transient or before the rotors have meaningful authority.
  double response_fault_min_thrust = 1.5;
  double confirm_sec = 0.15, confirm_angle = 0.0174533;
  double progress_angle = 0.00872665, stall_sec = 0.50;
  double progress_window = 1.0; // compare averaged ends of the recent window
  // If a released pair produces no retained roll improvement under meaningful
  // thrust, its contact lever/sign is not observable from geometry alone.
  // Run down that pair and let the caller try the other pair once instead of
  // spending the full global timeout at a static contact equilibrium.
  double pair_retry_sec = 3.0;
  // A pair is not "unresponsive" merely because static contact has not
  // moved at the first small feed-forward command.  Start the retry dwell
  // only after the command has reached this fraction of the configured safe
  // limit.  This gives the rate PI enough authority to overcome an unknown
  // contact breakaway moment without increasing the safety ceiling.
  double pair_retry_thrust_fraction = 0.75;
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
  double filtered_acceleration = 0.0, previous_filtered_rate = 0.0;
  double target_thrust = 0.0, effective_kp = 0.0;
  double reapproach_authority = 0.0;
  double reapproach_best_error = std::numeric_limits<double>::infinity();
  double reapproach_last_improvement = -1.0;
  // A stopping-distance brake is latched against a one-sample gyro reversal,
  // but it must not become a permanent zero-force state.  This timer confirms
  // that the correcting coast has actually stopped while a residual roll
  // error remains, before the same measured contact is allowed to approach
  // again with bounded authority.
  double terminal_rearm_since = -1.0;
  double braking_horizon = 0.0;
  double best_error = 0.0, last_progress = 0.0, last_update = 0.0;
  double candidate_start = -1.0, candidate_error = 0.0;
  double stop_band_since = -1.0;
  double stable_since = -1.0, last_learning = -1.0;
  bool moving = false, finished = false, fault = false, stalled = false;
  bool integrating = false, model_valid = false, learned_response = false;
  bool rate_initialized = false, acceleration_initialized = false;
  bool speed_braking = false, predicted_braking = false;
  bool timed_out = false, wrong_way = false, feedforward_initialized = false;
  bool terminal_brake_latched = false;
  bool breakaway_latched = false;
  double breakaway_authority = 0.0;
  bool low_authority_reapproach = false;
  // The selected pair can only generate moment in one direction.  Crossing
  // beyond the capture hysteresis therefore requires a fresh pair selection,
  // even if the old rotor command has already reached zero.
  bool target_crossed = false;
  // A pair selected immediately after crossing the target first has to brake
  // motion inherited from the previous pair.  The caller may raise this
  // threshold for that one pass so residual velocity is not mistaken for a
  // newly commanded wrong-sign moment at the ordinary low detection thrust.
  double wrong_way_detection_thrust = 0.0;
  double wrong_way_detection_until = -1.0;
  bool unresponsive = false;
  bool progress_observed = false;
  bool capture_active = false;
  double unresponsive_since = -1.0;
  // Time at which a previously useful contact candidate first remained
  // materially worse than its own best retained attitude.  This catches a
  // lever which reverses into a limit cycle: a short progress window can
  // alternate signs forever even though the long-term posture is lost.
  double retained_reversal_since = -1.0;
  double running_feedforward = 0.0;
  std::array<double, 9> stable_context{};
  struct Sample { double time, command, rate; };
  std::deque<Sample> history;
  // Keep the actuator history in the same window as the retained roll
  // progress.  A useful contact often rebounds slightly while correction
  // thrust is being removed.  That is actuator rundown, not evidence that
  // the contact geometry has reversed its moment sign.  Conversely, roll
  // which worsens while thrust is held or increased is direct evidence that
  // the current pair/contact state is no longer useful.
  struct ErrorSample { double time, error, thrust; };
  std::deque<ErrorSample> progress_history;
  double recent_progress = 0.0;
  double recent_thrust_change = 0.0;
  // Pneumatic regrasp changes the arm bend and therefore the recovery lever.
  // Speed/prediction braking alone may occur far from the target and must not
  // authorize regrasp.  Once the angle capture is latched AND rotor force has
  // run down, however, a small measured-pressure preload is useful damping:
  // leaving the pair fully exhausted let the body coast through zero for more
  // than two seconds and caused an opposite-pair limit cycle in MuJoCo.
  bool pressureHandoffReady() const {
    return finished || timed_out || fault || (capture_active && thrust <= 0.05);
  }
  void reset(double error, double now) {
    *this = ContinuousRollRecovery(); best_error = std::abs(error);
    last_progress = last_update = now;
    progress_history.push_back({now, std::abs(error), thrust});
  }
  // Re-selecting another contact mode of the same physical rotor pair does
  // not require removing its already verified correction authority.  Keep
  // the commanded force and filtered body motion, but start fresh response
  // and terminal-brake observations for the new pressure/contact state.
  // The caller must verify that the pair and correction direction are
  // unchanged and that measured motion is not wrong-way.
  void retargetKeepingThrust(
      double error, double now,
      double measured_command = std::numeric_limits<double>::quiet_NaN()) {
    const double retained_thrust = std::isfinite(measured_command)
        ? std::max(0.0, measured_command) : thrust;
    const double retained_rate = filtered_rate;
    const double retained_acceleration = filtered_acceleration;
    const bool retained_rate_initialized = rate_initialized;
    const bool retained_acceleration_initialized = acceleration_initialized;
    *this = ContinuousRollRecovery();
    thrust = hold = retained_thrust;
    filtered_rate = previous_filtered_rate = retained_rate;
    filtered_acceleration = retained_acceleration;
    rate_initialized = retained_rate_initialized;
    acceleration_initialized = retained_acceleration_initialized;
    running_feedforward = feedforward = retained_thrust;
    feedforward_initialized = retained_thrust > 0.0;
    best_error = std::abs(error);
    last_progress = last_update = now;
    progress_history.push_back({now, std::abs(error), retained_thrust});
  }
  // Transfer from outward-rate arrest to angle return without an artificial
  // drop in rotor force.  The force already required to stop deterioration
  // is the best available local contact estimate.  Make it the new model
  // baseline and clear the arrest integrator so the angle-rate PI adds only
  // the authority needed to return to zero roll.
  void beginAngleReturn() {
    running_feedforward = std::max(running_feedforward,
        std::max(thrust, breakaway_authority));
    feedforward = running_feedforward;
    feedforward_initialized = true;
    boost = 0.0;
    integrating = false;
    // The retained force becomes the angle-return feed-forward above.  The
    // arrest-only cap must not prevent the angle controller from adding
    // authority later if the body genuinely stalls before reaching zero.
    breakaway_latched = false;
    breakaway_authority = 0.0;
  }
  void updateProgress(double error, double now, const RollRecoveryConfig& c) {
    // Keep best_error for diagnostics only: returning from a worse posture
    // is progress even before the initial/best-ever error is reached again.
    best_error = std::min(best_error, std::abs(error));
    const double magnitude = std::abs(error);
    if (!progress_history.empty() && progress_history.back().time == now) {
      progress_history.back().error = magnitude;
      progress_history.back().thrust = thrust;
    } else progress_history.push_back({now, magnitude, thrust});
    const double begin = now - c.progress_window;
    while (progress_history.size() > 1 && progress_history[1].time <= begin)
      progress_history.pop_front();
    recent_progress = 0.0;
    recent_thrust_change = 0.0;
    if (progress_history.front().time > begin + 1.0e-9) return;
    // Average the first/last quarter instead of comparing two noisy IMU
    // samples. Repeated small jitter must not indefinitely renew the timer.
    double first = 0.0, last = 0.0;
    double first_thrust = 0.0, last_thrust = 0.0;
    size_t first_count = 0, last_count = 0;
    for (const auto& sample : progress_history) {
      if (sample.time < begin - 1.0e-9) continue;
      if (sample.time <= begin + 0.25 * c.progress_window) {
        first += sample.error;
        first_thrust += sample.thrust;
        ++first_count;
      }
      if (sample.time >= now - 0.25 * c.progress_window) {
        last += sample.error;
        last_thrust += sample.thrust;
        ++last_count;
      }
    }
    if (!first_count || !last_count) return;
    recent_progress = first / first_count - last / last_count;
    recent_thrust_change =
        last_thrust / last_count - first_thrust / first_count;
    if (recent_progress >= c.progress_angle) {
      last_progress = now;
      progress_observed = true;
    }
  }
  void resetResponseWindow(double error, double now) {
    progress_history.clear();
    progress_history.push_back({now, std::abs(error), thrust});
    recent_progress = recent_thrust_change = 0.0;
    // This reset belongs to contact-response identification only.  It must
    // never renew the independent no-progress watchdog: pressure and bend
    // targets can change repeatedly while the retained body roll remains
    // completely stationary.  Renewing last_progress here allowed a stalled
    // 4-N recovery to run forever as long as the pneumatic state was still
    // transitioning.
    candidate_start = -1.0;
    unresponsive_since = -1.0;
    retained_reversal_since = -1.0;
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
                double predictive_target = std::numeric_limits<double>::quiet_NaN(),
                bool allow_response_faults = true,
                double external_authority_limit =
                    std::numeric_limits<double>::infinity()) {
    integrating = false;
    speed_braking = predicted_braking = false;
    if (fault) {
      target_rate = target_thrust = 0.0;
      boost = 0.0;
      integrating = false;
      return thrust = 0.0;
    }
    if (!std::isfinite(error) || !std::isfinite(omega) || !std::isfinite(now) ||
        !std::isfinite(dt) || dt < 0.0 || now < last_update || now - last_update > 0.25 ||
        // `omega` is projected into the restorative direction.  A fast
        // positive rate is therefore a predictive-braking condition, not a
        // sensor fault.  Only a rate that is both fast and worsening fails
        // this directional envelope.  Sensor freshness/non-finiteness is
        // checked independently above and by the parent controller.
        omega <= -c.max_rate ||
        !std::isfinite(c.fall) || c.fall <= 0.0 ||
        !std::isfinite(c.max_target_rate) || c.max_target_rate <= 0.0 ||
        !std::isfinite(c.max_prediction_acceleration) ||
        c.max_prediction_acceleration <= 0.0 ||
        !std::isfinite(c.breakaway_acceleration) ||
        c.breakaway_acceleration <= 0.0 ||
        !std::isfinite(c.breakaway_thrust_margin) ||
        c.breakaway_thrust_margin < 0.0 ||
        !std::isfinite(c.prediction_acceleration_horizon) ||
        c.prediction_acceleration_horizon <= 0.0 ||
        !std::isfinite(c.progress_window) || c.progress_window <= 0.0 ||
        !std::isfinite(c.progress_angle) || c.progress_angle <= 0.0 ||
        !std::isfinite(c.contact_minimum_limit) ||
        c.contact_minimum_limit < 0.0 ||
        !std::isfinite(c.minimum_feedforward) ||
        c.minimum_feedforward < 0.0 ||
        c.minimum_feedforward > c.limit ||
        !std::isfinite(c.capture_margin_factor) ||
        c.capture_margin_factor < 0.0 ||
        !std::isfinite(c.pair_retry_sec) || c.pair_retry_sec <= c.stall_sec ||
        !std::isfinite(c.pair_retry_thrust_fraction) ||
        c.pair_retry_thrust_fraction < 0.0 ||
        c.pair_retry_thrust_fraction > 1.0 ||
        !std::isfinite(c.terminal_brake_fall) ||
        c.terminal_brake_fall < c.fall ||
        !std::isfinite(c.terminal_brake_max_error) ||
        c.terminal_brake_max_error < c.stop ||
        !std::isfinite(c.reapproach_limit) || c.reapproach_limit <= 0.0 ||
        !std::isfinite(c.reapproach_rate) || c.reapproach_rate <= 0.0 ||
        !std::isfinite(c.reapproach_authority_rise) ||
        c.reapproach_authority_rise <= 0.0 ||
        !std::isfinite(c.reapproach_escalation_delay) ||
        c.reapproach_escalation_delay < 0.0 ||
        !std::isfinite(c.response_fault_min_thrust) ||
        c.response_fault_min_thrust < 0.0 ||
        std::isnan(external_authority_limit) ||
        external_authority_limit < 0.0) {
      fault = true; boost = 0.0; return thrust = 0.0;
    }
    dt = std::min(0.1, dt); last_update = now;
    previous_filtered_rate = filtered_rate;
    filtered_rate = rate_initialized
        ? filtered_rate + dt / (0.08 + dt) * (omega - filtered_rate)
        : omega;
    if (rate_initialized && dt > 1.0e-6) {
      const double measured_acceleration = recoveryClamp(
          (filtered_rate - previous_filtered_rate) / dt,
          -c.max_prediction_acceleration, c.max_prediction_acceleration);
      filtered_acceleration = acceleration_initialized
          ? filtered_acceleration + dt / (0.12 + dt) *
              (measured_acceleration - filtered_acceleration)
          : measured_acceleration;
      acceleration_initialized = true;
    } else filtered_acceleration = 0.0;
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
    // Enter the capture state at the actual target, or inside its measurement
    // margin once angular motion is already slow.  Waiting for an exact
    // boundary caused the selected contact pair to pass through its useful
    // moment geometry and reverse the roll direction at 3.2 deg.  Unlike the
    // former open-loop handoff, the caller now restores pressure under roll
    // feedback, so accepting this low-rate margin does not recreate the old
    // regrasp rebound.
    // Latch capture as soon as the measured error first enters the target
    // band.  stop_band_since is also the hysteresis state: while rotor thrust
    // is running down, a small contact rebound may leave `stop` but remain
    // inside `stop + stop_margin`.  The previous code did not set this state
    // until thrust was already zero, so an 8.0 -> 8.1-deg rebound immediately
    // restarted PI, drove the arm through its useful moment geometry, and
    // produced a 0/1-N limit cycle.
    // `contact_minimum_limit` is an explicit opt-in escape hatch for a
    // compliant contact which cannot reach the nominal target.  A zero value
    // must not silently widen a +/-3-deg target to +/-5 deg: doing so handed
    // control to pneumatic regrasp early, and the re-bending reaction drove
    // the body back toward the recovery threshold.  Keep a small hysteresis
    // for an already-latched capture, but require the real stop target for a
    // new successful capture unless the caller configured a wider contact
    // minimum deliberately.  The rotor-only phase is allowed to hand over
    // within a small low-rate approach band: forcing an extended pneumatic
    // arm through the final degree can pass its useful contact geometry and
    // reverse the measured moment.  This is not the final recovery success
    // criterion; the caller's closed-loop pressure handoff still requires
    // the true `stop` angle after support has been restored.
    const double capture_entry_limit = c.contact_minimum_limit > c.stop
        ? c.contact_minimum_limit
        : c.stop + c.capture_margin_factor * c.stop_margin;
    const double capture_hysteresis_limit = std::max(
        capture_entry_limit + c.stop_margin,
        c.stop + 2.0 * c.stop_margin);
    const double stable_contact_limit = c.contact_minimum_limit > c.stop
        ? c.contact_minimum_limit : capture_hysteresis_limit;
    // A complete rotor rundown is a terminal action, not a generic speed
    // controller.  Limit it to +/-5 deg even if an older launch file still
    // supplies the former 15-deg envelope or a wider compliant-contact
    // minimum.  Outside this hard neighbourhood prediction may reduce speed,
    // but it must not command zero and unload a still-useful pair.
    const double terminal_full_brake_limit = std::min(
        0.08726646259971647,
        std::min(c.terminal_brake_max_error,
                 c.stop + 2.0 * c.stop_margin));
    if (capture_active && std::abs(error) > terminal_full_brake_limit) {
      capture_active = false;
      stop_band_since = -1.0;
    }
    // If terminal rundown stops the body just outside the requested band,
    // keep the same physical pair and continue from the remaining command.
    // Do not report the rotor pass as finished and force SAFE_TRANSFER merely
    // because capture hysteresis is wider than the actual success band.
    const bool captured_but_outside_target = capture_active &&
        c.contact_minimum_limit <= c.stop &&
        std::abs(error) > c.stop &&
        std::abs(filtered_rate) <= c.settle_rate && thrust <= 0.05;
    if (captured_but_outside_target) {
      capture_active = false;
      stop_band_since = -1.0;
      terminal_brake_latched = false;
      low_authority_reapproach = true;
      reapproach_authority = recoveryClamp(
          std::max(thrust, c.reapproach_limit), 0.05, c.limit);
      reapproach_best_error = std::abs(error);
      reapproach_last_improvement = now;
      terminal_rearm_since = -1.0;
      boost = 0.0;
      integrating = false;
      resetResponseWindow(error, now);
    }
    // Do not clear capture merely because a compliant contact settles just
    // outside the entry boundary after rotor rundown.  The wider bound above
    // is the explicit hysteresis for exactly that rebound; the caller keeps
    // roll feedback active while rebuilding pressure and applies the strict
    // final stop test after regrasp.
    // The rotor-only pass hands over just outside the final target when the
    // body is already slow.  RECOVER then closes the remaining error while
    // pressure is restored.  This prevents the extended-arm moment from
    // changing sign while trying to force the last fraction of a degree.
    // This is only a handoff boundary; the caller still requires the true
    // `stop` angle before declaring posture recovery complete.
    const bool settled_at_numeric_boundary =
        !(low_authority_reapproach && std::abs(error) > c.stop) &&
        progress_observed &&
        std::abs(error) <= capture_entry_limit &&
        std::abs(filtered_rate) <= c.settle_rate;
    // A compliant contact can have a local reachable equilibrium before the
    // nominal stop angle. Accept it only after real measured improvement and
    // whole-body motion confirmation, and only when roll has nearly stopped
    // inside the ordinary non-recovery band (8 deg stop + 2x1 deg margin in
    // the current Action-C setup). This prevents the controller from driving
    // through the useful contact geometry and reversing roll at 9--10 deg.
    const bool settled_contact_minimum = c.contact_minimum_limit > c.stop &&
        moving && progress_observed &&
        recent_progress >= c.progress_angle &&
        std::abs(error) <= c.contact_minimum_limit &&
        std::abs(filtered_rate) <= c.settle_rate;
    // A correcting rotor pair is one-directional.  Once the body has crossed
    // the requested attitude and is still moving away from it, capture
    // hysteresis must not keep the same contact state active until the full
    // outer margin is consumed.  This is particularly important for a
    // released pair on a small cylinder: chamber exhaust can carry the body
    // through the target after rotor thrust has already reached zero.  Ask
    // the caller to freeze the current contact pressure and reselect from the
    // opposite error sign at the first measured crossing.
    const bool moving_through_target = capture_active && error < 0.0 &&
        filtered_rate > c.settle_rate;
    if (moving_through_target) {
      target_crossed = true;
      timed_out = true;
      target_rate = target_thrust = 0.0;
      speed_braking = predicted_braking = true;
      boost = 0.0;
      thrust = std::max(0.0, thrust - c.fall * dt);
      if (thrust <= 1.0e-6) fault = true;
      return thrust;
    }
    const bool terminal_capture_allowed =
        std::abs(error) <= terminal_full_brake_limit;
    if (terminal_capture_allowed &&
        (std::abs(error) <= c.stop || settled_at_numeric_boundary ||
         settled_contact_minimum ||
         (capture_active && std::abs(error) <= stable_contact_limit))) {
      capture_active = true;
      updateProgress(std::abs(error), now, c);
      target_rate = target_thrust = 0.0;
      speed_braking = predicted_braking = true;
      thrust = std::max(0.0, thrust - c.fall * dt);
      // Do not hand pneumatic regrasp a nominally slow body while the rotor
      // pair is still carrying correction force.  Removing that force and
      // changing the contact stiffness at the same instant produced an
      // immediate roll rebound in MuJoCo (8 -> 20 deg).  Completion therefore
      // means angle, rate *and* actuator rundown have all settled.  If the
      // error leaves the band during rundown, the ordinary controller below
      // resumes without changing the contact pressure.
      // `finished` must mean that the requested angular band is still
      // satisfied after actuator rundown.  The old hysteresis completion
      // returned finished at 5.2 deg for a 3-deg target; the outer safety
      // layer correctly rejected that as a success and entered
      // SAFE_TRANSFER.  A rebound outside the true band now reuses this same
      // measured-good pair through the bounded re-approach path above.
      const bool final_angle_ready =
          capture_active && std::abs(error) <= c.stop;
      if (final_angle_ready && std::abs(filtered_rate) <= c.settle_rate &&
          thrust <= 0.05) {
        if (stop_band_since < 0.0) stop_band_since = now;
        if (now - stop_band_since >= c.settle_sec) {
          finished = true;
          boost = 0.0;
        }
      } else stop_band_since = -1.0;
      return thrust;
    }
    // Once the body has crossed through the target and left the capture
    // hysteresis, this one-direction pair cannot brake it.  Waiting for the
    // generic no-progress timeout leaves the released pair unsupported for
    // many seconds.  Finish the normal thrust rundown immediately so the
    // caller can choose the correcting pair for the newly measured sign.
    if (error <= -stable_contact_limit) {
      target_crossed = true;
      timed_out = true;
      target_rate = target_thrust = 0.0;
      speed_braking = predicted_braking = true;
      boost = 0.0;
      thrust = std::max(0.0, thrust - c.fall * dt);
      if (thrust <= 1.0e-6) fault = true;
      return thrust;
    }
    capture_active = false;
    stop_band_since = -1.0;
    // Retained roll progress is a global safety observation and is valid
    // even while pressure is in transit.  Only candidate attribution and
    // response learning are gated by pressure/bend settling.  Keeping these
    // clocks separate prevents a pneumatic lag from hiding a true stall.
    updateProgress(error, now, c);
    if (!allow_response_faults) {
      candidate_start = -1.0;
      unresponsive_since = -1.0;
      retained_reversal_since = -1.0;
    }
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
    // The contact pivot can change as a released arm extends. A pair which
    // initially corrected roll may then reverse its effective moment even
    // though the kinematic lever estimate still has the old sign. Do not
    // respond to a full-window measured deterioration by integrating harder.
    // Smoothly remove thrust and regrasp; the outer state machine can retry
    // from the newly measured contact condition.
    // Do not require the breakaway detector to have latched before accepting
    // a full-window deterioration. With a deformable contact, the selected
    // pair can first reduce the error slowly and then reverse as its arm
    // straightens. The full history window plus minimum applied thrust still
    // rejects a single IMU spike at startup.
    // A full-window deterioration under meaningful thrust is sufficient
    // evidence that this contact/pair is moving the body the wrong way.  Do
    // not wait for the positive-direction breakaway detector: an immediately
    // wrong response can never satisfy that detector and previously continued
    // integrating until the physical fall limit.
    // Latch this evidence through the commanded rundown. Otherwise the flag
    // became false again as soon as thrust fell below detect_thrust, and the
    // caller could no longer distinguish a wrong-sign contact response from
    // an ordinary no-progress timeout when it was finally safe to switch.
    // A full-window deterioration under meaningful thrust plus even a small
    // same-direction body rate is a whole-body observation. Requiring the
    // full breakaway-rate threshold delayed rejection of a slow wrong-sign
    // response until the body had accumulated tens of degrees of roll. Keep
    // a tenth-threshold rate gate for a pair with no prior progress so a
    // static external displacement is not blamed on the actuator.
    // `recent_progress` uses averaged window ends and therefore rejects a
    // single gyro spike or the short initial pneumatic transient.  Also
    // require that correction thrust was not being deliberately run down
    // during that window.  The latter distinguishes a normal compliant
    // rebound after predictive braking from a true contact-moment reversal.
    // A half-degree retained change is sufficient to renew useful progress,
    // but it is too small to prove moment-sign reversal: pneumatic capture
    // routinely rebounds by that amount. Reject a candidate only after at
    // least twice that loss. The raised thrust threshold used for inherited
    // motion is temporary; keeping it for the whole retry hid a genuine
    // reversal until the new pair had climbed above 3 N.
    // Terminal braking is evaluated below from the current stopping-distance
    // prediction.  It deliberately remains in this control pass: forcing a
    // complete zero-force dwell here produced the observed OFF--ON correction
    // close to the target on hardware.
    const double wrong_way_thrust = std::max(
        c.response_fault_min_thrust,
        now <= wrong_way_detection_until
            ? std::max(c.detect_thrust, wrong_way_detection_thrust)
            : c.detect_thrust);
    const double wrong_way_progress = 2.0 * c.progress_angle;
    const double pair_retry_thrust = std::max(c.detect_thrust,
        recoveryClamp(c.pair_retry_thrust_fraction, 0.0, 1.0) * c.limit);
    const bool correction_authority_not_decreasing =
        recent_thrust_change >= -0.1;
    wrong_way = wrong_way || (allow_response_faults &&
        ((moving || progress_observed ||
          filtered_rate <= -0.1 * c.start_rate) &&
         recent_progress <= -wrong_way_progress &&
         correction_authority_not_decreasing &&
         thrust >= wrong_way_thrust));
    // A changed contact pivot can also create a bounded oscillation rather
    // than a monotonic wrong-way response.  In that case consecutive
    // one-second windows alternate between positive and negative progress,
    // but the body never returns to the candidate's best achieved pose.  Do
    // not let those local oscillations renew the candidate indefinitely:
    // after one real improvement, require it to recover to within one
    // progress quantum of that best pose.  Only reject after the larger
    // two-quantum loss persists for the full loaded retry dwell.
    const bool retained_reversal_loaded = allow_response_faults &&
        progress_observed &&
        std::abs(error) >= best_error + wrong_way_progress &&
        thrust >= pair_retry_thrust;
    if (retained_reversal_loaded && retained_reversal_since < 0.0)
      retained_reversal_since = now;
    else if (std::abs(error) <= best_error + c.progress_angle)
      retained_reversal_since = -1.0;
    if (retained_reversal_since >= 0.0 &&
        now - retained_reversal_since >= c.pair_retry_sec &&
        thrust >= pair_retry_thrust)
      wrong_way = true;
    if (wrong_way) timed_out = true;
    stalled = now - last_progress >= c.stall_sec && filtered_rate < c.start_rate;
    const bool pair_retry_loaded = allow_response_faults && stalled &&
        thrust >= pair_retry_thrust &&
        std::abs(error) > stable_contact_limit;
    if (pair_retry_loaded) {
      if (unresponsive_since < 0.0) unresponsive_since = now;
      unresponsive = unresponsive ||
          now - unresponsive_since >= c.pair_retry_sec;
    } else {
      unresponsive_since = -1.0;
    }
    if (wrong_way || unresponsive) timed_out = true;
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
    // Normal braking follows the filtered whole-body roll rate.  The raw
    // Spinal gyro contains short contact/arm impacts; treating every such
    // sample as body overspeed repeatedly cut a 5-N target to about 3 N on
    // the 100/130-mm fixtures although the retained roll angle had stopped
    // improving.  Gross raw-rate safety is still enforced independently by
    // the `abs(omega) >= max_rate` fault above.
    const double helpful_rate = std::max(0.0, filtered_rate);
    const double helpful_acceleration = std::max(0.0,
        std::min(c.max_prediction_acceleration, filtered_acceleration));
    // Static friction releases faster than the retained-angle window can
    // confirm motion.  Latch that first physically useful response and stop
    // the rate-arrest integrator immediately; otherwise the command keeps
    // climbing for another few seconds and produces the large late impulse
    // seen on hardware.
    // Angular-rate/acceleration spikes occur while a pneumatic arm changes
    // contact even when the body keeps falling.  They may request a small
    // instantaneous brake, but they must not become the persistent authority
    // ceiling until the retained-angle detector has confirmed that roll error
    // actually decreased.  The 2026-11-02 bag otherwise latched 2.85 N while
    // `moving` remained false and roll worsened from 15.4 to 17.8 deg.
    const bool breakaway_acceleration_event = c.rate_arrest_mode && moving &&
        thrust >= c.detect_thrust &&
        helpful_rate >= 0.25 * c.start_rate &&
        helpful_acceleration >= c.breakaway_acceleration;
    const bool breakaway_rate_event = c.rate_arrest_mode && moving &&
        thrust >= c.detect_thrust &&
        helpful_rate >= 0.5 * c.start_rate;
    if (!breakaway_latched &&
        (breakaway_acceleration_event || breakaway_rate_event)) {
      breakaway_latched = true;
      breakaway_authority = recoveryClamp(
          thrust + c.breakaway_thrust_margin, 0.0, c.limit);
      hold = breakaway_authority;
    }
    // Predict the same rundown rate that will actually be applied.  The old
    // code predicted with 3 N/s and then executed the terminal brake at
    // 8 N/s, overestimating coast distance and braking several degrees too
    // early on high-friction contact.
    const bool inside_terminal_brake_envelope =
        std::abs(error) <= terminal_full_brake_limit;
    const double predicted_fall_rate = inside_terminal_brake_envelope
        ? c.terminal_brake_fall : c.fall;
    braking_horizon = c.horizon +
        std::max(0.0, thrust) / predicted_fall_rate;
    const double acceleration_horizon = std::min(
        braking_horizon, c.prediction_acceleration_horizon);
    const double predicted = error - helpful_rate * braking_horizon -
        0.5 * helpful_acceleration * acceleration_horizon *
            acceleration_horizon;
    predicted_braking = predicted <= 0.0;
    const bool terminal_brake_request = predicted_braking &&
        inside_terminal_brake_envelope &&
        thrust >= c.detect_thrust && helpful_rate >= c.start_rate;
    if (terminal_brake_request && !terminal_brake_latched) {
      // Discard accumulated static-friction integral, but retain the actual
      // force and taper it from the current value.  This prevents the old PI
      // command from returning after a short gyro reversal without inserting
      // a zero-force gap into the same correction.
      terminal_brake_latched = true;
      low_authority_reapproach = true;
      boost = 0.0;
      integrating = false;
      reapproach_authority = recoveryClamp(thrust, 0.05, c.limit);
      reapproach_best_error = std::abs(error);
      reapproach_last_improvement = now;
      terminal_rearm_since = -1.0;
      resetResponseWindow(error, now);
    }
    if (terminal_brake_latched) {
      low_authority_reapproach = true;
      // Follow the force downward only while the body is still coasting
      // toward the target.  Once that coast has stopped outside the true
      // +/-stop band, re-arm the same contact state without requiring a
      // zero-force dwell.  The old implementation kept this latch forever:
      // the authority was repeatedly reduced to 0.05 N, leaving hardware at
      // 4 deg or even 16 deg residual roll until the global timeout.
      const bool residual_needs_correction = std::abs(error) > c.stop;
      const bool correcting_coast_stopped =
          filtered_rate <= c.settle_rate && !predicted_braking;
      if (residual_needs_correction && correcting_coast_stopped) {
        if (terminal_rearm_since < 0.0)
          terminal_rearm_since = now;
      } else {
        terminal_rearm_since = -1.0;
      }
      const bool terminal_rearm_confirmed = terminal_rearm_since >= 0.0 &&
          now - terminal_rearm_since >= c.confirm_sec;
      if (terminal_rearm_confirmed) {
        terminal_brake_latched = false;
        // A stopping prediction can be triggered by a short contact-rate
        // impulse while the body is still far from level.  Once that coast
        // has stopped, do not keep the complete remaining recovery behind
        // the gentle retry ceiling.  Reserve low-authority re-approach for
        // the actual terminal neighbourhood; a far residual resumes the
        // ordinary angle/rate law continuously from the rundown force.
        const bool terminal_neighbourhood =
            std::abs(error) <= stable_contact_limit;
        low_authority_reapproach = terminal_neighbourhood;
        reapproach_authority = terminal_neighbourhood
            ? recoveryClamp(std::max(thrust, c.reapproach_limit),
                            0.05, c.limit)
            : c.limit;
        reapproach_best_error = std::abs(error);
        reapproach_last_improvement = now;
        terminal_rearm_since = -1.0;
        boost = 0.0;
        integrating = false;
        resetResponseWindow(error, now);
      } else {
        reapproach_authority = std::min(reapproach_authority,
            recoveryClamp(thrust, 0.05, c.limit));
      }
    }
    const double remaining = std::max(0.0, error - c.stop);
    // Prediction limits velocity smoothly instead of toggling between full
    // speed and zero at the stop-band edge.
    // The requested attitude is zero roll; `stop` is only the accepted final
    // tolerance.  Using error-stop here made the requested rate collapse as
    // the body approached +/-3 deg and left hardware at a nonzero plateau.
    // Aim the dynamic law at zero while the capture branch above still
    // accepts a quiet measurement anywhere inside the configured band.
    const double zero_centered_error = std::max(0.0, error);
    const double approach_rate = zero_centered_error /
        std::max(0.05, braking_horizon);
    target_rate = std::min(c.max_target_rate,
        std::min(c.angle_gain * zero_centered_error, approach_rate));
    if (low_authority_reapproach)
      target_rate = std::min(target_rate, c.reapproach_rate);
    // Ordinary rate error is already handled continuously by the PI term.
    // Treating every small target-rate crossing as a separate actuator brake
    // produced the observed ON/OFF sawtooth: the command fell, contact stuck,
    // roll worsened, and the same authority was rebuilt.  The direct thrust
    // brake is now reserved for a genuine breakaway acceleration, severe
    // overspeed, or the stopping-distance terminal prediction below.
    speed_braking = breakaway_acceleration_event;
    // Static friction can hold several newtons and then release abruptly.
    // That transition is whole-body motion even before the slower retained-
    // progress detector has completed its window.  A clearly supercritical
    // rate under meaningful rotor authority therefore requests the same
    // latched rundown as the stopping-distance predictor.
    const bool severe_overspeed = thrust >= c.detect_thrust &&
        helpful_rate >= std::max(2.0 * target_rate,
                                 1.5 * c.max_target_rate) &&
        helpful_rate >= c.start_rate;
    // Far from the target, an overspeed is a request to reduce thrust, not to
    // run it all the way to zero.  A full terminal brake is reserved for the
    // bounded stopping-distance prediction below.
    const bool far_prediction_braking = predicted_braking &&
        !inside_terminal_brake_envelope &&
        thrust >= c.detect_thrust && helpful_rate >= c.start_rate;
    speed_braking = speed_braking || severe_overspeed ||
        far_prediction_braking;
    // Do not release terminal braking merely because measured rate has
    // fallen. Static/contact rebound can make the rate briefly small and
    // previously restored the full feed-forward command in the same pass.
    // reset() clears this latch after the caller has accepted the handoff or
    // selected a fresh measured contact.
    model_valid = m.valid && std::isfinite(m.moment) && std::isfinite(m.lever) && m.lever >= c.min_lever;
    // Blend near the geometry-validity boundary: no 0.5 -> 4 N switch.
    const double confidence = model_valid ? recoveryClamp(
        (m.lever - c.min_lever) / c.min_lever, 0.0, 1.0) : 0.0;
    const double nominal = model_valid ? recoveryClamp(m.moment / m.lever, 0.0, c.limit) : c.fallback_thrust;
    const double requested_ff = std::max(c.minimum_feedforward,
        c.fallback_thrust + confidence *
            (nominal - c.fallback_thrust));
    if (!feedforward_initialized) {
      running_feedforward = requested_ff; feedforward_initialized = true;
    } else if (!terminal_brake_latched) {
      // Recompute tau/l throughout angle return.  The low-pass term and
      // actuator slew bounds reject bend/CoP jitter, while retaining the
      // measured moment model after whole-body motion has begun.  Freezing
      // this value at breakaway was the reason a large residual roll could
      // remain at an almost constant command.
      const bool measured_return_motion =
          filtered_rate >= 0.5 * c.start_rate || progress_observed;
      // Before a retained correcting response exists, a smaller instantaneous
      // tau/l estimate is not evidence that the breakaway force may be
      // removed.  Bend/contact motion made the estimate oscillate and caused
      // the 2026-11-02 hardware command to fall after every phase handoff.
      const double model_fall = (measured_return_motion || speed_braking)
          ? -c.fall * dt : 0.0;
      running_feedforward += recoveryClamp(
          (requested_ff - running_feedforward) * dt / (0.25 + dt),
          model_fall, c.rise * dt);
    }
    if (c.rate_arrest_mode && breakaway_latched &&
        filtered_rate >= 0.5 * c.start_rate)
      running_feedforward = std::min(
          running_feedforward, breakaway_authority);
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
    // In a contact-limited plateau, alternating gyro/contact impulses are not
    // retained roll progress.  They previously cancelled the integral and
    // held 100/130-mm fixtures at only 1.5--2.4 N indefinitely.  While the
    // full-window roll error is not improving, retain/increase breakaway
    // authority; the independent overspeed path may still reduce the actual
    // command immediately.  Genuine worsening is handled by `wrong_way`.
    const bool retained_plateau = now - last_progress + 1.0e-9 >= c.stall_sec &&
        recent_progress < c.progress_angle && !wrong_way;
    // Pressure settling intentionally disables wrong-way identification, but
    // it must not disable emergency breakaway search.  While rate arrest is
    // active, grow the one bounded integral whenever the filtered body rate
    // has not yet become clearly corrective.  This uses the existing PI and
    // its anti-windup; no parallel fixed boost is added.
    const bool breakaway_hold_active = c.rate_arrest_mode &&
        breakaway_latched && filtered_rate >= 0.5 * c.start_rate;
    const bool rate_arrest_authority_search = c.rate_arrest_mode &&
        !breakaway_hold_active &&
        error > c.stop && filtered_rate < 0.5 * c.start_rate;
    const bool authority_search =
        (retained_plateau || rate_arrest_authority_search) &&
        !speed_braking && !terminal_brake_latched &&
        !breakaway_hold_active;
    if (authority_search) {
      // A pre-plateau gyro impulse may have accumulated a small negative
      // value before the retained-motion window closes.  Once the measured
      // roll has demonstrably failed to improve, that value is not evidence
      // for lowering the breakaway command.  Start the bounded extra
      // authority from zero and only grow it while the plateau persists.
      boost = std::max(0.0, boost);
      delta = std::max(delta, c.integral_rate * dt);
    }
    // The measured breakaway force is now the best local model.  Do not hide
    // extra authority in the integral while the actuator command itself is
    // capped, and never integrate upward while velocity/acceleration asks for
    // braking.
    if ((breakaway_hold_active || speed_braking) && delta > 0.0)
      delta = 0.0;
    // Anti-windup for both amplitude and actuator slew saturation. This is
    // the ONLY integral: old fixed stall boosts are not superimposed.
    if ((delta > 0.0 && (unclamped >= c.limit ||
                         (!authority_search && unclamped > thrust + c.rise * dt))) ||
        (delta < 0.0 && (unclamped <= 0.0 || unclamped < thrust - c.fall * dt))) delta = 0.0;
    // Predictive mode replaces the nominal FF+PI target, never adds to it.
    // Freeze the unused PI integral; all actual-rate/slew/fault guards below
    // still apply to the planner exactly as they do to the baseline.
    if(std::isfinite(predictive_target)) delta = 0.0;
    boost = recoveryClamp(boost + delta, -c.limit, c.limit);
    integrating = delta != 0.0;
    if (terminal_brake_latched) {
      boost = 0.0;
      integrating = false;
    }
    target_thrust = recoveryClamp(base + boost, 0.0, c.limit);
    if(std::isfinite(predictive_target))
      target_thrust = recoveryClamp(predictive_target, 0.0, c.limit);
    if (breakaway_hold_active)
      target_thrust = std::min(target_thrust, breakaway_authority);
    if (low_authority_reapproach) {
      if (reapproach_authority <= 0.0)
        reapproach_authority = std::min(c.limit, c.reapproach_limit);
      // This progress clock is deliberately independent of the response-
      // identification gate.  Pressure and bend are changing during a
      // retry, so that gate resets its own history every cycle; using the
      // shared `last_progress` held hardware at 1.5 N for 20--40 seconds.
      if (!std::isfinite(reapproach_best_error)) {
        reapproach_best_error = std::abs(error);
        reapproach_last_improvement = now;
      }
      if (reapproach_best_error - std::abs(error) >= c.progress_angle) {
        reapproach_best_error = std::abs(error);
        reapproach_last_improvement = now;
      }
      const bool retry_stalled =
          reapproach_last_improvement >= 0.0 &&
          now - reapproach_last_improvement >=
              c.reapproach_escalation_delay &&
          helpful_rate < c.start_rate &&
          // `stable_contact_limit` is only capture hysteresis.  It is wider
          // than the requested final stop band, so using it here left the
          // real arm indefinitely at about 5 deg with the initial 1.5-N
          // retry cap.  Keep restoring authority while a quiet residual is
          // still outside the actual target; prediction/velocity braking
          // remains responsible for preventing overshoot.
          std::abs(error) > c.stop;
      if (retry_stalled)
        reapproach_authority = std::min(c.limit,
            reapproach_authority + c.reapproach_authority_rise * dt);
      target_thrust = std::min(target_thrust, reapproach_authority);
    }
    if (std::isfinite(external_authority_limit))
      target_thrust = std::min(target_thrust, external_authority_limit);
    if (terminal_brake_latched) {
      // Monotonic one-pass rundown. The quarter-rate lead keeps the taper
      // smooth while guaranteeing that neither PI nor model feed-forward can
      // re-accelerate this candidate after braking has begun.
      const double terminal_rundown_target = std::max(0.0,
          thrust - 0.25 * c.terminal_brake_fall * dt);
      target_thrust = std::min(target_thrust, terminal_rundown_target);
    }
    if (speed_braking || terminal_brake_request) {
      // Priority over model FF and retained I, before motion confirms too.
      // Reduce smoothly in proportion to overspeed, not an OFF pulse or an
      // opposite-pair command. Raw or filtered overspeed suffices.
      double fraction = recoveryClamp(
          (helpful_rate - target_rate) / c.max_target_rate, 0.0, 1.0);
      if (breakaway_acceleration_event)
        fraction = std::max(0.25, fraction);
      if (terminal_brake_request)
        fraction = std::max(0.25, fraction);
      const double fall_rate = terminal_brake_request
          ? c.terminal_brake_fall : c.fall;
      const double speed_cap = std::max(0.0,
          thrust - fall_rate * fraction * dt);
      target_thrust = std::min(target_thrust, speed_cap);
    }
    const double applied_fall_rate = terminal_brake_latched
        ? c.terminal_brake_fall : c.fall;
    thrust += recoveryClamp(target_thrust - thrust,
        -applied_fall_rate * dt, c.rise * dt);
    if (terminal_brake_latched)
      reapproach_authority = std::min(reapproach_authority,
          recoveryClamp(thrust, 0.05, c.limit));
    return thrust = recoveryClamp(thrust, 0.0, c.limit);
  }
};
} // namespace hugmy
#endif
