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
  // Distance shaping changes extension and pull together.  Retain its exact
  // scale so a shortened/strengthened stroke can update the response model
  // without being mistaken for an unscaled optimizer candidate.
  double command_scale=1.;
  bool completed=false, safe=false;
};
class TrajectoryAdaptation {
 public:
  static constexpr unsigned baseline_confirmations=2;
  std::vector<TrajectorySample> samples;
  unsigned current=0;
  unsigned baseline_revalidation_remaining=0;
  bool enabled=true;
  static TrajectoryChoice choice(unsigned i) {
    // Once a repeatable waveform exists, explore only its local neighbourhood.
    // The former +/-15% force and +/-0.10 mix steps were unnecessarily large
    // for an online trial on a supported body. Larger adaptation belongs to
    // the measured inner feedback, not a blind one-stroke outer jump.
    // Test a little more horizontal force before raising total force.  The
    // combined candidate is last, so these effects remain identifiable.
    const std::array<TrajectoryChoice,7> choices{{{0.,1.},{.05,1.},{0.,1.05},
        {.05,1.05},{-.05,1.},{0.,.95},{-.05,1.05}}};
    return choices[i%choices.size()];
  }
  TrajectoryChoice selected() const {return choice(current);}
  static double contextDistance(const Pair& a,const Pair& b,double roll,double pitch,
                                const TrajectorySample& s) {
    return std::pow((a[0]-b[0])/.2,2)+std::pow((a[1]-b[1])/.2,2)+
        std::pow((roll-s.roll)/.15,2)+std::pow((pitch-s.pitch)/.2,2);
  }
  struct ProgressEstimate {
    bool valid=false;
    double progress=std::numeric_limits<double>::quiet_NaN();
    unsigned count=0;
  };
  ProgressEstimate progressEstimate(const Pair& bend,double roll,double pitch,
                                    int choice_filter) const {
    std::vector<double> recent;
    // Use at most the three latest comparable results.  Old contact states
    // should not dominate after the arms have advanced along the cylinder.
    for(auto it=samples.rbegin();it!=samples.rend() && recent.size()<3;++it) {
      const auto& s=*it;
      if(!s.safe || s.progress<.001 || contextDistance(bend,s.bend,roll,pitch,s)>4. ||
         (choice_filter>=0 && int(s.choice)!=choice_filter) ||
         !std::isfinite(s.command_scale) ||
         s.command_scale<.2) continue;
      // Extension and pull are each changed by sqrt(scale), so their first
      // order product is scale.  Normalize before judging repeatability.
      recent.push_back(s.progress/s.command_scale);
    }
    if(recent.size()<2) return {};
    std::sort(recent.begin(),recent.end());
    const double median=recent[recent.size()/2];
    const double span=recent.back()-recent.front();
    // Two millimetres covers endpoint noise; beyond that require agreement to
    // within 30% of the median.  A broad pair such as 2 and 3.5 mm must not be
    // promoted into a stride model merely because both happened to advance.
    if(span>std::max(.002,.3*median)) return {};
    double sum=0.;for(double x:recent) sum+=x;
    return {true,clamp(sum/recent.size(),.001,.05),unsigned(recent.size())};
  }
  // Estimate the unscaled retained displacement only from repeatable,
  // comparable safe strokes. A single lucky displacement cannot strengthen
  // the next command.
  double estimatedProgress(const Pair& bend,double roll,double pitch) const {
    auto estimate=progressEstimate(bend,roll,pitch,int(current));
    // Do not average unlike mix/force candidates. Until the selected
    // candidate itself is repeatable, fall back to the repeatable unchanged
    // waveform. This keeps a horizontal-mix experiment from silently
    // changing the distance model used to scale total force.
    if(!estimate.valid && current!=0)
      estimate=progressEstimate(bend,roll,pitch,0);
    return estimate.valid?estimate.progress:std::numeric_limits<double>::quiet_NaN();
  }
  // Tile the full remaining distance with the repeatable stride.  A tiny last
  // stroke is distributed over the preceding strokes; a nearly full last
  // stroke is kept.  Ten millimetres is the preferred efficient stride from
  // the task specification, but command growth is bounded per update.
  static double planStride(double remaining,double measured,double preferred,
                           double minimum_scale,double maximum_scale) {
    if(!std::isfinite(remaining) || remaining<=0. ||
       !std::isfinite(measured) || measured<.001) return measured;
    if(remaining<.9*measured)
      return clamp(remaining,minimum_scale*measured,maximum_scale*measured);
    const double nominal=std::max(measured,std::min(preferred,remaining));
    const unsigned whole=std::max(1u,unsigned(std::floor(remaining/nominal+1e-9)));
    const double remainder=remaining-whole*nominal;
    double target=nominal;
    if(remainder>1e-6) {
      // Avoid a weak cleanup motion: spread a small residual over the already
      // planned strokes. If the residual is almost a full stroke, add one
      // slightly weaker stroke instead of making all preceding strokes large.
      target=remainder<.75*nominal?remaining/whole:
          remaining/(whole+1.);
    }
    return clamp(target,minimum_scale*measured,maximum_scale*measured);
  }
  void record(const Pair& bend,double roll,double pitch,double progress,double seconds,
              double effort,double recovery,bool measured,bool completed,bool safe,
              double command_scale=1.) {
    // Keep measured history even when outer candidate exploration is disabled:
    // the goal conditioner still needs a contact-specific nominal stride.
    if(!measured || !std::isfinite(progress) || !std::isfinite(seconds) || seconds<=0. ||
       !std::isfinite(effort) || effort<0. || !std::isfinite(recovery) || recovery<0. ||
       !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(bend[0]) ||
       !std::isfinite(bend[1]) || !std::isfinite(command_scale) ||
       command_scale<.2 || command_scale>2.) return;
    // A guarded stroke is still a real locomotion observation after a quiet,
    // deflated endpoint has proved that the displacement was retained.  Do
    // not erase useful forward travel merely because the intended waveform
    // ended early.  Truly unsafe/unmeasured motion can never earn credit.
    const double dx=safe?progress:std::min(0.,progress);
    // mm/s minus normalized effort per retained mm and full recovery time.
    // Sub-mm rocking/idle cannot be the winning stationary strategy.
    const double score=1000.*dx/seconds-.002*effort/std::max(1.,1000.*std::max(0.,dx))-
        recovery/seconds-(dx<.001?2.:0.)-(safe?(completed?0.:.25):3.);
    samples.push_back({current,bend,roll,pitch,score,dx,command_scale,completed,safe});
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
    for(const auto& s:samples) if(contextDistance(bend,s.bend,roll,pitch,s)<=4. &&
                                  std::abs(s.command_scale-1.)<=.02) {
      ++count[s.choice];sum[s.choice]+=s.score;++n;
    }
    if(!n) {current=0;return;}
    if(baseline_revalidation_remaining) {current=0;return;}
    // First prove that one unchanged waveform is repeatable. Exploring a new
    // mix every stroke made identification itself disturb otherwise useful
    // consecutive motion. Failed/idle baseline samples have non-positive
    // progress and do not satisfy this confirmation count.
    const auto baseline=progressEstimate(bend,roll,pitch,0);
    if(!baseline.valid || baseline.count<baseline_confirmations)
      {current=0;return;}
    // Repeat one positive exploratory sample once. Candidate comparison must
    // use a reproducible response rather than a one-off displacement.
    if(current!=0 && count[current]==1 && samples.back().choice==current &&
       samples.back().safe && samples.back().progress>=.001) return;
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
