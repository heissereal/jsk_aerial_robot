#pragma once
#include <hugmy/control/hybrid_locomotion_mpc.h>
#include <vector>

namespace hugmy { namespace hybrid {
// A bounded, measured-result search. No simulated prediction is recorded as
// progress. The two coordinates are early horizontal mix and force scale.
// Transition duration is set by the measured pneumatic response, not a third
// independently competing optimizer. Physical safety limits always win.
struct TrajectoryChoice { double mix_offset=0., force_scale=1.; };
struct TrajectorySample {
  unsigned choice=0; Pair bend{};
  double roll=0., pitch=0., score=0., progress=0.;
  bool completed=false, safe=false;
};
class TrajectoryAdaptation {
 public:
  static constexpr unsigned baseline_confirmations=3;
  std::vector<TrajectorySample> samples;
  unsigned current=0;
  unsigned baseline_revalidation_remaining=0;
  bool enabled=true;
  static TrajectoryChoice choice(unsigned i) {
    // Once a repeatable waveform exists, explore only its local neighbourhood.
    // The former +/-15% force and +/-0.10 mix steps were unnecessarily large
    // for an online trial on a supported body. Larger adaptation belongs to
    // the measured inner feedback, not a blind one-stroke outer jump.
    const std::array<TrajectoryChoice,7> choices{{{0.,1.},{.05,1.},{-.05,1.},
        {0.,1.05},{0.,.95},{.05,1.05},{-.05,1.05}}};
    return choices[i%choices.size()];
  }
  TrajectoryChoice selected() const {return choice(current);}
  static double contextDistance(const Pair& a,const Pair& b,double roll,double pitch,
                                const TrajectorySample& s) {
    return std::pow((a[0]-b[0])/.2,2)+std::pow((a[1]-b[1])/.2,2)+
        std::pow((roll-s.roll)/.15,2)+std::pow((pitch-s.pitch)/.2,2);
  }
  // Estimate retained displacement only from comparable, successful full
  // strokes.  This estimate is used to shorten the final commanded stride;
  // it is deliberately separate from selecting mix/force candidates.
  double estimatedProgress(const Pair& bend,double roll,double pitch) const {
    auto estimate=[&](bool selected_only) {
      double weighted_progress=0.,weight_sum=0.;
      for(const auto& s:samples) {
        const double distance=contextDistance(bend,s.bend,roll,pitch,s);
        if(!s.safe || s.progress<.001 || distance>4. ||
           (selected_only && s.choice!=current)) continue;
        const double weight=1./(.25+distance);
        weighted_progress+=weight*s.progress;weight_sum+=weight;
      }
      return weight_sum>0.?weighted_progress/weight_sum:
          std::numeric_limits<double>::quiet_NaN();
    };
    double estimate_value=estimate(true);
    if(!std::isfinite(estimate_value)) estimate_value=estimate(false);
    return std::isfinite(estimate_value)?clamp(estimate_value,.001,.05):
        estimate_value;
  }
  void record(const Pair& bend,double roll,double pitch,double progress,double seconds,
              double effort,double recovery,bool measured,bool completed,bool safe) {
    // Keep measured history even when outer candidate exploration is disabled:
    // the goal conditioner still needs a contact-specific nominal stride.
    if(!measured || !std::isfinite(progress) || !std::isfinite(seconds) || seconds<=0. ||
       !std::isfinite(effort) || effort<0. || !std::isfinite(recovery) || recovery<0. ||
       !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(bend[0]) || !std::isfinite(bend[1])) return;
    // A guarded stroke is still a real locomotion observation after a quiet,
    // deflated endpoint has proved that the displacement was retained.  Do
    // not erase useful forward travel merely because the intended waveform
    // ended early.  Truly unsafe/unmeasured motion can never earn credit.
    const double dx=safe?progress:std::min(0.,progress);
    // mm/s minus normalized effort per retained mm and full recovery time.
    // Sub-mm rocking/idle cannot be the winning stationary strategy.
    const double score=1000.*dx/seconds-.002*effort/std::max(1.,1000.*std::max(0.,dx))-
        recovery/seconds-(dx<.001?2.:0.)-(safe?(completed?0.:.25):3.);
    samples.push_back({current,bend,roll,pitch,score,dx,completed,safe});
    // A poor exploratory stroke is useful evidence, but it must not be
    // followed immediately by another unknown condition. Return to the known
    // baseline for two retained-forward strokes, then continue the local
    // search. This preserves adaptation without allowing exploratory failures
    // to accumulate into loss of progress or posture.
    if(current!=0 && (!completed || !safe || dx<.001))
      baseline_revalidation_remaining=2;
    else if(current==0 && safe && dx>=.001 &&
            baseline_revalidation_remaining)
      --baseline_revalidation_remaining;
    if(samples.size()>70) samples.erase(samples.begin());
  }
  void select(const Pair& bend,double roll,double pitch) {
    if(!enabled) {current=0;return;}
    std::array<unsigned,7> count{};std::array<double,7> sum{};
    unsigned n=0;
    for(const auto& s:samples) if(contextDistance(bend,s.bend,roll,pitch,s)<=4.) {
      ++count[s.choice];sum[s.choice]+=s.score;++n;
    }
    if(!n) {current=0;return;}
    if(baseline_revalidation_remaining) {current=0;return;}
    // First prove that one unchanged waveform is repeatable. Exploring a new
    // mix every stroke made identification itself disturb otherwise useful
    // consecutive motion. Failed/idle baseline samples have non-positive
    // progress and do not satisfy this confirmation count.
    unsigned baseline_successes=0;
    for(const auto& s:samples)
      if(s.choice==0 && s.progress>=.001 && s.safe &&
         contextDistance(bend,s.bend,roll,pitch,s)<=4.) ++baseline_successes;
    if(baseline_successes<baseline_confirmations) {current=0;return;}
    // Explore finite local candidates; then revisit using a bounded confidence
    // bonus. Updates occur only at measured regrasp boundaries, never mid-pulse.
    for(unsigned i=0;i<count.size();++i) if(!count[i]) {current=i;return;}
    double best=-std::numeric_limits<double>::infinity();
    for(unsigned i=0;i<count.size();++i) {
      const double value=sum[i]/count[i]+.2*std::sqrt(std::log(1.+n)/count[i]);
      if(value>best) {best=value;current=i;}
    }
  }
};
}}
