#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace hugmy {
inline double actionCPeakExtensionMismatch(const std::array<bool,4>& front,
    const std::array<double,4>& start, const std::array<double,4>& minimum) {
  double lo=INFINITY, hi=-INFINITY;
  for(size_t i=0;i<4;++i) if(front[i]) {
    if (!std::isfinite(start[i]) || !std::isfinite(minimum[i])) return INFINITY;
    const double peak=std::max(0.,start[i]-minimum[i]);
    lo=std::min(lo,peak); hi=std::max(hi,peak);
  }
  return std::isfinite(lo) ? hi-lo : INFINITY;
}
// Soft per-arm phase change. This never authorizes violating a hard guard.
inline bool actionCFlexionHandoff(double elapsed, double extension, double target,
    double rate, double bend_margin, double rate_limit, double lookahead) {
  for (double v : {elapsed, extension, target, rate, bend_margin, rate_limit, lookahead})
    if (!std::isfinite(v)) return false;
  if (elapsed<.10 || target<=0 || rate_limit<=0 || lookahead<0) return false;
  const double outward=std::max(0.,-rate);
  if (extension>=target) return true;
  if (extension<2.*M_PI/180.) return false;
  return extension+outward*lookahead>=target ||
      bend_margin-outward*lookahead<=2.*M_PI/180. || outward>=.5*rate_limit;
}
// Commanded aerodynamic force, NOT contact force or proof of unloading.
// Heading is relative to the body-x heading at the start of this task.
inline double actionCForwardProjection(double bend, double arm_yaw, double cant,
    double spin, double roll, double pitch, double heading, double travel_sign) {
  for (double v : {bend, arm_yaw, cant, spin, roll, pitch, heading, travel_sign})
    if (!std::isfinite(v)) return 0.;
  const double lx = -std::sin(bend)*std::cos(cant);
  const double ly = spin*std::sin(cant), z = std::cos(bend)*std::cos(cant);
  const double x = std::cos(arm_yaw)*lx-std::sin(arm_yaw)*ly;
  const double y = std::sin(arm_yaw)*lx+std::cos(arm_yaw)*ly;
  const double level_x = std::cos(pitch)*x + std::sin(pitch)*(std::sin(roll)*y+std::cos(roll)*z);
  const double level_y = std::cos(roll)*y-std::sin(roll)*z;
  return travel_sign*(std::cos(heading)*level_x-std::sin(heading)*level_y);
}

// Timing is learned per arm; settings are immutable within a trial. Command
// latency, pressure rise and observed flexion onset are separate diagnostics.
struct ActionCPropulsion {
  struct Response {
    double start=-1, base=0, goal=0, rise_at=-1, rate=0, delay=-1;
    double min_bend=0, flex_at=-1, flex_candidate=-1, sample_at=-1;
    double lead=0, pending=-1;
    bool censored=false, pressure_identified=false;
    void resetPulse() {
      start=rise_at=flex_at=flex_candidate=sample_at=-1;
      rate=0; delay=-1; censored=false; pressure_identified=false;
    }
    void command(double now, double measured, double target, double bend) {
      if (start>=0 || !std::isfinite(measured) || !std::isfinite(target) || target-measured<2.) return;
      start=now; base=measured; goal=target; min_bend=bend;
    }
    void observe(double now, double stamp, double measured, double bend, double bend_rate, bool fresh,
                 bool angle_fresh=true) {
      if (start<0 || censored) return;
      if (now<start) {censored=true; pending=-1; return;}
      // A later stale sample must not erase an already identified pressure rise.
      if (!fresh || !std::isfinite(measured)) {
        if (!pressure_identified) {censored=true; pending=-1;} return;
      }
      if (now-start>6.) return;
      if (stamp<=sample_at) return;
      sample_at=stamp;
      if (rise_at<0 && measured>=base+.5) {rise_at=now; delay=now-start;}
      if (rise_at>=0 && now-rise_at>=.04)
        rate=std::max(0.,(measured-base-.5)/(now-rise_at));
      if (!pressure_identified && rise_at>=0 && rate>1. && measured>=base+.5*(goal-base)) {
        pressure_identified=true;
        pending=std::min(.20,delay+.5*(goal-base)/rate);
      }
      // Pneumatic identification does not depend on gait success or a usable
      // bend estimate. Flexion onset is a separate observation, not causality.
      if (!angle_fresh || !std::isfinite(bend) || !std::isfinite(bend_rate)) return;
      if (!std::isfinite(min_bend)) min_bend=bend;
      min_bend=std::min(min_bend,bend);
      if (bend-min_bend>=.5*M_PI/180 && bend_rate>.01) {
        if (flex_candidate<0) flex_candidate=now;
        if (flex_at<0 && now-flex_candidate>=.04) flex_at=now;
      } else flex_candidate=-1;
      // Observe actual flexion, not a pressure spike caused by contact.
      // Prediction includes the pressure at onset, not necessarily full grasp P.
      if (flex_at==now && pressure_identified) {
        const double predicted=delay+std::max(0.,measured-base-.5)/rate;
        pending=std::min(.20,std::max(0.,std::max(flex_at-start,predicted)));
      }
    }
    bool early(double elapsed, double duration, double extension, double target, double rate_rad_s) const {
      if (lead<=0 || elapsed<.15 || extension<.5*target || !std::isfinite(rate_rad_s)) return false;
      const double eta=rate_rad_s<-.01 ? std::max(0.,target-extension)/(-rate_rad_s) : INFINITY;
      return std::min(std::max(0.,duration-elapsed),eta)<=lead;
    }
  };
  std::array<Response,4> response;
  std::array<bool,4> flexing{};
  std::array<double,4> flex_ceiling{};
  double regrip_start=-1, regrip_ready_since=-1, both_powered_sec=0;
  double extra_n=0, forward_impulse=0, last_force_time=-1;
  double trial_forward_impulse=0;
  int completed_pulls=0, timing_rounds=0;
  bool pulse_completed=false;
  void beginTrial() {
    completed_pulls=0; trial_forward_impulse=0;
    for (auto& r:response) r.pending=-1;
  }
  void beginPulse() {
    pulse_completed=false; forward_impulse=0; last_force_time=-1;
    flexing.fill(false); flex_ceiling.fill(0.);
    regrip_start=regrip_ready_since=-1; both_powered_sec=0;
    for (auto& r:response) r.resetPulse();
  }
  void force(double now, double forward, bool both_powered=true) {
    if (last_force_time>=0 && now>last_force_time && now-last_force_time<=.1 && std::isfinite(forward)) {
      forward_impulse+=forward*(now-last_force_time);
      if (both_powered && forward>0.) both_powered_sec+=now-last_force_time;
    }
    last_force_time=now;
  }
  void startFlexion(size_t arm, double actual_thrust, double residual) {
    if (flexing[arm]) return;
    flexing[arm]=true;
    flex_ceiling[arm]=std::max(0.,std::min(actual_thrust,residual));
  }
  bool allFlexing(const std::array<bool,4>& front) const {
    bool any=false;
    for(size_t i=0;i<4;++i) if(front[i]) {any=true; if(!flexing[i]) return false;}
    return any;
  }
  bool regripReady(double now, bool ready) {
    if (!ready) {regrip_ready_since=-1; return false;}
    if (regrip_ready_since<0 || now<regrip_ready_since) regrip_ready_since=now;
    return now-regrip_ready_since>=.15;
  }
  void complete(bool rebent) {pulse_completed=rebent;}
  void regrip(bool valid) {
    if (valid && pulse_completed) {++completed_pulls; trial_forward_impulse+=forward_impulse;}
    pulse_completed=false; // never credit the same pulse twice
  }
  // Timing observations can be committed after a failed movement. They do
  // NOT authorize amplitude learning. Only one owner changes inputs/boundary.
  bool commitTiming(const std::array<bool,4>& front) {
    bool changed=false;
    for (size_t i=0;i<4;++i) if(front[i] && response[i].pending>=0 && !response[i].censored) {
      auto& r=response[i];
      const double delta=std::max(-.03,std::min(.03,r.pending-r.lead));
      if (std::abs(delta)>.005) {r.lead+=delta; changed=true;}
      r.pending=-1;
    }
    if(changed) ++timing_rounds;
    return changed;
  }
  // 0=no change, 2=pull amplitude only. Never boost after a
  // timeout/guard/recovery, without two-arm flexion, or after backward motion.
  int finish(bool safe, int pulses, double retained, double goal,
             double base_residual, double max_residual, const std::array<bool,4>& front) {
    if (!safe || pulses<=0 || completed_pulls!=pulses || !std::isfinite(retained) || retained<0.) return 0;
    (void)front;
    if (retained<goal && trial_forward_impulse>0. && base_residual+extra_n<max_residual-1e-6) {
      extra_n=std::min(max_residual-base_residual,extra_n+.10); return 2;
    }
    return 0;
  }
};
} // namespace hugmy
