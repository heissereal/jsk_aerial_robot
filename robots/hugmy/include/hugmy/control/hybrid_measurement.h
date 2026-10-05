#pragma once
#include <hugmy/control/hybrid_locomotion_mpc.h>

namespace hugmy { namespace hybrid {
// Timestamped low-pass for contact chatter in soft prediction/settlement.
// Never use this signal instead of the raw gyro for the hard rate guard.
struct BodyRateFilter {
  Pair value{};
  double stamp=-1.;
  void reset() {*this=BodyRateFilter();}
  Pair update(const Pair& raw,double time) {
    if(!std::isfinite(time) || !std::isfinite(raw[0]) || !std::isfinite(raw[1])) {
      reset();return raw;
    }
    if(time==stamp) return value;
    const double dt=time-stamp;
    if(stamp<0. || dt<=0. || dt>.25) value=raw;
    else for(size_t j=0;j<2;++j) value[j]+=dt/(.10+dt)*(raw[j]-value[j]);
    stamp=time;return value;
  }
};
// Pressure excursions hold commands and skip learning for a bounded time.
// Both the soft and hard thresholds require the configured confirmation time:
// the real ADC can produce one impossible sample (99.3 kPa between 33.9 and
// 35.5 kPa samples).  Invalid/stale data still fail immediately, while a
// physically sustained overpressure still stops after the same confirmation
// interval used by the controller-wide pressure safety check.
struct PressureGuard {
  enum Status { Ready, Pending, Fault };
  double since=-1., last=-1.;
  void reset() {since=last=-1.;}
  Status update(const std::array<double,4>& pressure,double now,bool fresh,
                double soft,double hard,double confirmation) {
    if(!fresh || !std::isfinite(now) || (last>=0. && now<last) ||
       !std::isfinite(soft) || !std::isfinite(hard) || hard<=soft ||
       !std::isfinite(confirmation) || confirmation<0.) {reset();return Fault;}
    last=now;
    bool excursion=false;
    for(double p:pressure) {
      // A non-finite sample is safety critical.
      // Small negative gauge readings, however, are observed as isolated ADC
      // outliers while exhausting a real chamber.  Treat those with the same
      // short confirmation used for a high excursion.  A hard-threshold
      // sample is included in this confirmation rather than stopping on one
      // ADC spike; sustained hard pressure still fails closed below.
      if(!std::isfinite(p)) {reset();return Fault;}
      excursion=excursion || p< -5. || p>soft || p>=hard;
    }
    if(!excursion) {since=-1.;return Ready;}
    if(since<0.) since=now;
    return now-since+1e-9>=confirmation ? Fault : Pending;
  }
};

// Unique, consecutive mocap samples. Time-weighted mean avoids dependence on
// message rate; a quiet window is required for retained-position evaluation.
struct PositionWindow {
  struct Sample { double t,x; };
  std::deque<Sample> samples;
  double duration=.8;
  void reset() {samples.clear();}
  void add(double t,double x) {
    if(!std::isfinite(t)||!std::isfinite(x)) {reset();return;}
    if(!samples.empty() && t==samples.back().t) return;
    if(!samples.empty() && (t<samples.back().t || t-samples.back().t>.25)) reset();
    samples.push_back({t,x});
    while(samples.size()>2 && samples[1].t<=t-duration) samples.pop_front();
  }
  bool ready(double now) const {
    return samples.size()>=5 && now>=samples.back().t && now-samples.back().t<=.25 &&
        samples.back().t-samples.front().t>=duration-1e-6;
  }
  double mean() const {
    if(samples.size()<2) return std::numeric_limits<double>::quiet_NaN();
    double area=0.;
    for(size_t i=1;i<samples.size();++i)
      area+=(samples[i].t-samples[i-1].t)*.5*(samples[i].x+samples[i-1].x);
    return area/(samples.back().t-samples.front().t);
  }
  double speed() const {
    if(samples.size()<2) return 0.;
    // Least-squares velocity from a window, not a single difference.
    double t=0.,x=0.,tt=0.,tx=0.;
    for(const auto& s:samples) {double d=s.t-samples.front().t;t+=d;x+=s.x;tt+=d*d;tx+=d*s.x;}
    const double n=samples.size(),den=n*tt-t*t;
    return den>1e-12?(n*tx-t*x)/den:0.;
  }
  bool stable(double now,double max_span=.002,double max_speed=.002) const {
    if(!ready(now)) return false;
    // Test consecutive sub-window means, not raw extrema. A 1-mm mocap
    // noise floor has >2-mm peak-to-peak even when the body is stationary.
    std::array<double,4> sums{};std::array<unsigned,4> counts{};
    const double span=samples.back().t-samples.front().t;
    for(const auto& s:samples) {
      const size_t i=std::min(size_t(3),size_t(4.*(s.t-samples.front().t)/span));
      sums[i]+=s.x;++counts[i];
    }
    double lo=std::numeric_limits<double>::infinity(),hi=-lo;
    for(size_t i=0;i<4;++i) {
      if(!counts[i]) return false;
      const double m=sums[i]/counts[i];lo=std::min(lo,m);hi=std::max(hi,m);
    }
    if(std::abs(samples.back().x-mean())>2.*max_span) return false;
    return hi-lo<=max_span && std::abs(speed())<=max_speed;
  }
};

// Bound unsuccessful powered probing, not successful continuous locomotion.
// x is a recent mocap mean, not a raw sample or predicted motion.
struct ProbeBudget {
  double time=0.,effort=0.,start_x=0.;bool initialized=false;
  void reset() {*this=ProbeBudget();}
  bool update(double dt,double x,double thrust_squared,bool valid) {
    if(!valid || !std::isfinite(dt) || dt<=0. || dt>.1 ||
       !std::isfinite(x) || !std::isfinite(thrust_squared) || thrust_squared<0.) {
      reset();return false;
    }
    if(!initialized) {start_x=x;initialized=true;}
    time+=dt;effort+=dt*thrust_squared;
    if(time<6.) return false;
    if(x-start_x>=.002) {reset();return false;}
    return effort>=24.; // N^2 s, surrogate effort, not electrical joules
  }
};

// Slow bottom assistance within a powered segment. Only one demand is being
// increased at a time: thrust first; then bottom if demand is capped/constrained
// and speed remains low. No regrip is required for a bottom adjustment.
struct BottomAssist {
  double command=0., target=0., stalled=0., settling=0.;
  bool probing=false;
  void reset(double pressure) {command=target=clamp(pressure,0.,30.);stalled=settling=0.;probing=false;}
  bool update(double dt,double speed,bool speed_valid,bool limited,bool quiet,
              double measured,bool enabled,double target_speed=.005) {
    if(!enabled) {probing=false;stalled=0.;return false;}
    if(probing) {
      if(speed_valid && speed>=target_speed) {
        probing=false;target=command;stalled=0.;return false;
      }
      if(speed_valid && quiet) command=slew(command,target,1.,1.,dt);
      settling+=dt;
      // Bounded freeze, even if pressure cannot reach the target.
      if(settling>=8. || (std::abs(command-target)<.01 &&
          std::abs(measured-target)<=2. && settling>=6.)) {
        probing=false;stalled=0.;target=command;
      }
    } else if(!speed_valid || !quiet || speed>=target_speed) stalled=0.;
    else if(limited) {
      stalled+=dt;
      if(stalled>=2. && target<30.) {target=std::min(30.,target+5.);probing=true;settling=0.;stalled=0.;}
    } else stalled=0.;
    return probing;
  }
};
} }
