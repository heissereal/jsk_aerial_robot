#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace hugmy {
// Learn bottom inflation only during a commanded fill. No actuator writes.
// The gait owns thrust/arm pressure; this helper only schedules bottom fill
// and releases the PREPARE barrier when pressure is predicted ready at peak.
struct BottomPulseTiming {
  bool filling=false, learned=false;
  double rate=10.0, delay=.15, margin=.05;
  double stamp=-1., pressure=0., fill_start=0., fill_pressure=0.;
  double rise_start=-1., window_time=0., window_pressure=0., last_rise=-1.;
  bool local_rate_ready=false;
  double local_rate=0.;
  double peak_error=std::numeric_limits<double>::quiet_NaN();
  double peak_at=-1.;
  void beginCycle() { filling=false; peak_at=-1.; }
  bool fresh(double now) const { return stamp>=0 && now>=stamp && now-stamp<=.25; }
  void startFill(double now) {
    filling=true; fill_start=now; fill_pressure=pressure;
    rise_start=-1.; window_time=now; window_pressure=pressure; last_rise=-1.;
    local_rate_ready=false;
  }
  void observe(double p, double now, double target) {
    if (!std::isfinite(p) || !std::isfinite(now)) { stamp=-1.; learned=false; return; }
    if (stamp>=0 && (now<stamp || now-stamp>.25)) {
      learned=false;
      if (filling) { filling=false; peak_at=-1.; }
    }
    stamp=now; pressure=p;
    if (peak_at>=0 && now>=peak_at) { peak_error=p-target; peak_at=-1.; }
    if (!filling) return;
    if (rise_start<0 && p-fill_pressure>=.5) {
      const double measured_delay=std::max(0.,std::min(2.,now-fill_start));
      delay=learned ? .8*delay+.2*measured_delay : measured_delay;
      rise_start=now; window_time=now; window_pressure=p;
      last_rise=now;
    }
    if (rise_start>=0 && now-window_time>=.1) {
      const double slope=(p-window_pressure)/(now-window_time);
      local_rate_ready=slope>=1. && slope<=200. && p<=target+2.;
      local_rate=slope;
      if (local_rate_ready) {
        rate=learned ? (slope<rate ? .5*rate+.5*slope : .8*rate+.2*slope) : slope;
        learned=true; last_rise=now;
      }
      window_time=now; window_pressure=p;
    }
  }
  double eta(double target,double tolerance,double now) const {
    if (!fresh(now)) return std::numeric_limits<double>::infinity();
    if (pressure>=target-tolerance) return 0.;
    if (!learned || !local_rate_ready || last_rise<0 || now-last_rise>.25)
      return std::numeric_limits<double>::infinity();
    const double lag=rise_start>=0 ? 0. : std::max(0.,delay-(now-fill_start));
    return lag+std::max(0.,target-tolerance-pressure)/std::max(1.,std::min(rate,local_rate))+margin;
  }
  bool ready(double target,double tolerance,double peak_lead,double now) const {
    return filling && eta(target,tolerance,now)<=std::max(0.,peak_lead);
  }
  // Delay model determines how early supply must open; current measured
  // slope gates launch, so a slow/stalled fill cannot reuse a fast old model.
  double plannedFillLead(double target,double low) const {
    return delay+std::max(0.,target-low)/std::max(1.,rate)+margin;
  }
};
}
