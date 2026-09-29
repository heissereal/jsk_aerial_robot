#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace hugmy {
// Small residual controller around the proven pulse, NOT a new trajectory.
// One trial owns immutable offsets/lead times. Measurements accumulate in
// pending storage; only a completed, unguarded trial can commit them.
struct ActionCCoordination {
  using Four = std::array<double, 4>;
  Four offset{}, lead{}, lag_sum{}, delta_sum{};
  std::array<int, 4> lag_count{};
  Four pressure_start{}, pressure_base{}, pressure_goal{}, observed_lag{};
  int samples = 0, rounds = 0;
  double max_assist = 0.3, max_lead = 0.12;
  double lag_kp = 6.0;
  int learning_rounds = 3;

  void clearTrial() { samples = 0; lag_sum.fill(0); delta_sum.fill(0); lag_count.fill(0); }
  void startPulse() {
    pressure_start.fill(-1); observed_lag.fill(-1);
    pressure_base.fill(0); pressure_goal.fill(0);
  }
  void startPressure(size_t arm, double now, double measured, double target) {
    if (pressure_start[arm] >= 0 || !std::isfinite(measured) ||
        !std::isfinite(target) || target - measured < 2.0) return;
    pressure_start[arm] = now; pressure_base[arm] = measured; pressure_goal[arm] = target;
  }
  void observePressure(double now, const Four& measured, bool fresh) {
    for (size_t i=0; i<4; ++i) {
      if (pressure_start[i] < 0 || observed_lag[i] != -1) continue;
      const double elapsed = now-pressure_start[i];
      // Missing samples censor this observation instead of inventing a delay.
      if (!fresh || !std::isfinite(measured[i]) || elapsed < 0 || elapsed > 2.0) {
        observed_lag[i] = -2; continue;
      }
      if (measured[i] >= pressure_base[i] + .5*(pressure_goal[i]-pressure_base[i]))
        observed_lag[i] = elapsed;  // empirical command-to-50%-rise time
    }
  }
  bool earlyPressure(size_t arm, double elapsed, double duration,
                     double extension, double target, double bend_rate) const {
    if (!std::isfinite(extension) || !std::isfinite(bend_rate) || target<=0 ||
        lead[arm]<=0 || elapsed<.15 || extension<.5*target) return false;
    const double time_to_angle = bend_rate < -0.01
        ? std::max(0.,target-extension)/(-bend_rate) : INFINITY;
    return std::min(std::max(0.,duration-elapsed),time_to_angle)<=lead[arm];
  }
  Four assist(const std::array<bool,4>& front, const Four& extension,
              double target, double roll, double rate, double relative_tilt,
              double tilt_limit, double elapsed, double ramp) const {
    Four out{};
    double fastest=0;
    for (size_t i=0;i<4;++i) if (front[i]) {
      if (!std::isfinite(extension[i])) return out;
      fastest=std::max(fastest,extension[i]);
    }
    // Never fight posture/overshoot brakes with an extra thrust source.
    if (!std::isfinite(roll) || !std::isfinite(rate) || !std::isfinite(relative_tilt) ||
        std::abs(roll)>=8*M_PI/180 || std::abs(rate)>=.3 || tilt_limit<=0) return out;
    const double authority=std::max(0.,1.-relative_tilt/(.5*tilt_limit));
    double sum=0;
    for (size_t i=0;i<4;++i) if (front[i] && extension[i]<target) {
      // A 2-deg deadband; use the bounded budget early enough to matter.
      out[i]=std::max(0.,offset[i]+lag_kp*std::max(0.,std::min(target,fastest)-extension[i]-2*M_PI/180));
      sum+=out[i];
    }
    const double scale=sum>0 ? std::min(1.,max_assist/sum)*authority : 0;
    for (double& value:out) value=std::min(value*scale,std::max(0.,elapsed*ramp));
    return out;
  }
  void observePulse(const std::array<bool,4>& front, const Four& extension,
                    double target, bool valid) {
    if (!valid || rounds>=learning_rounds) return;
    double fastest=0;
    for (size_t i=0;i<4;++i) if (front[i]) {
      if (!std::isfinite(extension[i]) || extension[i]<-2*M_PI/180) return;
      fastest=std::max(fastest,extension[i]);
    }
    ++samples;
    for (size_t i=0;i<4;++i) if (front[i]) {
      // Do not infer a left/right gain when NEITHER arm moved.
      if (fastest>=.25*target)
        delta_sum[i]+=std::max(-.05,std::min(.05,
            .5*(std::min(target,fastest)-extension[i]-2*M_PI/180)));
      if (observed_lag[i]>=0) {lag_sum[i]+=observed_lag[i];++lag_count[i];}
    }
  }
  bool finishTrial(bool safe) {
    bool changed=false;
    if (safe && samples>0 && rounds<learning_rounds) {
      ++rounds;
      Four next=offset;
      double total=0;
      for (size_t i=0;i<4;++i) {
        next[i]=std::max(0.,std::min(max_assist,offset[i]+delta_sum[i]/samples));
        total+=next[i];
        if (lag_count[i]>0) {
          const double desired=std::min(max_lead,lag_sum[i]/lag_count[i]);
          const double value=lead[i]+std::max(-.03,std::min(.03,desired-lead[i]));
          changed=changed || std::abs(value-lead[i])>1e-5; lead[i]=value;
        }
      }
      for (size_t i=0;i<4;++i) {
        next[i]*=total>max_assist && total>0 ? max_assist/total : 1.;
        changed=changed || std::abs(next[i]-offset[i])>1e-5;
      }
      offset=next;
    }
    clearTrial(); return changed;
  }
};

// Recovery cannot create gait credit, but backward slip during recovery must
// subtract from already earned progress. Never turn zero-motion into success.
inline double actionCRetainedAfterRecovery(double earned, bool recovery, double recovery_dx) {
  return earned + (recovery ? std::min(0.,recovery_dx) : 0.);
}
inline double actionCEfficiencyScore(double progress, double seconds, double effort,
    double effort_weight, double recovery_seconds, double recovery_weight) {
  if (!std::isfinite(seconds) || seconds<=0 || !std::isfinite(progress) ||
      !std::isfinite(effort) || !std::isfinite(recovery_seconds)) return -INFINITY;
  return progress/seconds - effort_weight*effort/seconds -
      recovery_weight*recovery_seconds/seconds;
}
} // namespace hugmy
