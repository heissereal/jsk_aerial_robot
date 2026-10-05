#pragma once
#include <hugmy/control/hybrid_locomotion_mpc.h>
#include <vector>

namespace hugmy { namespace hybrid {
// Slow experiment selection. No hypothetical MPC rollout is a real sample.
// Context uses settled front/rear angles, roll/pitch and front/rear pressure;
// bottom pressure is an action, not a context coordinate.
using Context=std::array<double,8>;
struct Candidate { double bottom=5., minimum_ratio=1.; };
struct Result {
  Context context{};Candidate candidate;
  double progress=0.,seconds=0.,effort=0.,recovery_seconds=0.,goal=.02;
  bool completed=false, safe=false;
  bool productive() const {
    return safe && std::isfinite(progress) && std::isfinite(goal) && goal>0. &&
        progress>=std::min(.002,.5*goal);
  }
  double score() const {
    if(!std::isfinite(progress) || !std::isfinite(seconds) || seconds<=0. ||
       !std::isfinite(effort) || effort<0 || !std::isfinite(recovery_seconds) || recovery_seconds<0 ||
       !std::isfinite(goal) || goal<=0) return -std::numeric_limits<double>::infinity();
    // A guarded/incomplete episode may earn credit only after a stable,
    // deflated endpoint proves that its forward displacement was retained.
    const double retained=safe?progress:std::min(0.,progress);
    // Integral(sum commanded T_i^2 dt), NOT electrical joules. Idling cannot
    // dilute energy already spent. Meaningful retained progress is required.
    if(!productive()) return -1.-std::max(0.,1.-retained/goal)-.001*effort-
        .001*seconds-.01*recovery_seconds-(safe?0.:1.);
    const double cost_per_mm=(effort+.01*seconds+.1*recovery_seconds)/(1000.*retained);
    return 1./(1.+cost_per_mm);
  }
};
class Search {
 public:
  Candidate initial, current;
  std::vector<Result> samples;
  unsigned direction=0, trials=0;
  bool enabled=true;
  bool bottom_probe_pending=false, bottom_probe_active=false, repeat_current=false;
  bool alternative_pending=false, ratio_probe_active=false;
  Context alternative_context{};
  int bottom_direction=1;
  void requestBottomProbe() {if(enabled) bottom_probe_pending=true;}
  void keepCurrent() {repeat_current=true;}
  void finishProbe(const Result& r) {
    if(!enabled || !r.completed || !r.safe || !finite(r.context)) return;
    alternative_pending=!r.productive();repeat_current=r.productive();
    alternative_context=r.context;
  }
  void reset() {*this=Search();}
  static bool finite(const Context& c) { for(double v:c) if(!std::isfinite(v)) return false;return true; }
  static double distance(const Context& a,const Context& b) {
    const Context scale{{.3,.2,.3,.2,.12,.2,10.,10.}};
    double d=0.;for(size_t i=0;i<a.size();++i) d+=std::pow((a[i]-b[i])/scale[i],2);
    return d;
  }
  void record(const Result& r) {
    if(!finite(r.context) || !std::isfinite(r.score())) return;
    samples.push_back(r);if(samples.size()>64) samples.erase(samples.begin());++trials;
  }
  Candidate select(const Context& context) {
    bottom_probe_active=false;
    if(!enabled || !finite(context)) return current;
    if(alternative_pending) {
      alternative_pending=false;repeat_current=false;bottom_probe_pending=false;
      // Regripping itself changes bend/roll context. Do not reset the coarse
      // probe to m=1 on every such change; context still gates learned reuse.
      // Prefer a known efficient response, excluding the just-failed action.
      const Result* productive_best=nullptr;
      for(const auto& r:samples) if(r.productive() && distance(r.context,context)<=4. &&
          (std::abs(r.candidate.bottom-current.bottom)>.01 ||
           std::abs(r.candidate.minimum_ratio-current.minimum_ratio)>.001)) {
        const Result* latest=&r;
        for(auto it=samples.rbegin();it!=samples.rend();++it)
          if(distance(it->context,context)<=4. &&
             std::abs(it->candidate.bottom-r.candidate.bottom)<.01 &&
             std::abs(it->candidate.minimum_ratio-r.candidate.minimum_ratio)<.001) {latest=&*it;break;}
        if(latest->productive() && (!productive_best || latest->score()>productive_best->score()))
          productive_best=latest;
      }
      if(productive_best) current=productive_best->candidate;
      else if(current.minimum_ratio>.001)
        current.minimum_ratio=std::max(0.,current.minimum_ratio-.25);
      else {
        // All coarse mixes tried: reconsider bottom, starting with slip again.
        current.minimum_ratio=1.;
        if(current.bottom+5.*bottom_direction>30. || current.bottom+5.*bottom_direction<0.)
          bottom_direction=-bottom_direction;
        current.bottom=clamp(current.bottom+5.*bottom_direction,0.,30.);
      }
      ratio_probe_active=true;return current;
    }
    if(bottom_probe_pending) {
      bottom_probe_pending=false;
      if(current.bottom+5.*bottom_direction>30. || current.bottom+5.*bottom_direction<0.)
        bottom_direction=-bottom_direction;
      current.bottom=clamp(current.bottom+5.*bottom_direction,0.,30.);
      bottom_probe_active=true;repeat_current=true;
      return current; // compare bottom at the retained thrust search target
    }
    if(repeat_current) {repeat_current=false;return current;}
    Candidate best=initial;double value=-1e30;
    // Repeatability: mean minus half observed score spread for matched actions.
    for(const auto& sample:samples) {
      if(distance(sample.context,context)>4.) continue;
      double sum=0.,sq=0.;unsigned n=0;
      for(const auto& r:samples) if(distance(r.context,context)<=4. &&
          std::abs(r.candidate.bottom-sample.candidate.bottom)<.01 &&
          std::abs(r.candidate.minimum_ratio-sample.candidate.minimum_ratio)<.001) {
        sum+=r.score();sq+=r.score()*r.score();++n;
      }
      const double mean=sum/n,score=mean-.5*std::sqrt(std::max(0.,sq/n-mean*mean));
      if(score>value) {value=score;best=sample.candidate;}
    }
    if(value<=-1e29) {current=initial;return current;}
    // Include baseline rechecks, both axes, and a coupled step. A failed
    // no-motion candidate never globally lowers a thrust ceiling.
    const std::array<Candidate,7> offsets{{{0.,0.},{5.,0.},{0.,-.1},{-5.,0.},{0.,.1},{5.,-.1},{-5.,.1}}};
    for(size_t attempt=0;attempt<offsets.size();++attempt) {
      const auto delta=offsets[direction++%offsets.size()];
      Candidate next{clamp(best.bottom+delta.bottom,0.,30.),clamp(best.minimum_ratio+delta.minimum_ratio,0.,1.)};
      if((delta.bottom==0.&&delta.minimum_ratio==0.) ||
          std::abs(next.bottom-best.bottom)>.01 || std::abs(next.minimum_ratio-best.minimum_ratio)>.001) {
        current=next;return current;
      }
    }
    current=best;return current;
  }
};
} }
