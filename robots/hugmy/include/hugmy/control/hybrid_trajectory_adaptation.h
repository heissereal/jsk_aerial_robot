#pragma once
#include <hugmy/control/hybrid_locomotion_mpc.h>
#include <vector>

namespace hugmy { namespace hybrid {
// A bounded, measured-result search. No simulated prediction is recorded as
// progress. Besides horizontal mix and pull magnitude, it can request a
// slightly deeper measured unloading event and a small rear reverse assist.
// Pressure timing and arm synchronization remain in the inner feedback loops.
struct TrajectoryChoice {
  double mix_offset=0., force_scale=1., unload_scale=1., rear_reverse_n=0.;
};
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
  static constexpr unsigned choice_count=6;
  // One measured forward baseline is enough to start a bounded local probe.
  // Two mutually consistent samples are still required by progressEstimate()
  // before the result is used for goal-distance scaling.  Keeping these two
  // confidence levels separate prevents the optimizer from remaining at
  // candidate zero forever on contacts which change as the robot advances.
  static constexpr unsigned baseline_confirmations=1;
  std::vector<TrajectorySample> samples;
  unsigned current=0;
  unsigned baseline_revalidation_remaining=0;
  bool enabled=true;
  static TrajectoryChoice choice(unsigned i) {
    // The measured unloading pose commonly gives m0 ~= 0.94--1.0. Positive
    // mix offsets are then clipped and produce no distinct experiment. The
    // high-friction contact instead needs vertical support to persist while
    // pressure starts flexion, followed continuously by horizontal pull as
    // measured flexion grows. Probe physically distinct negative offsets
    // before force-only changes. The independent 6-N, bend/rate and posture
    // guards remain authoritative, and a poor probe still forces one baseline
    // revalidation before another candidate is tried.
    // Compensate the first two mix probes so their horizontal component is
    // approximately unchanged: 0.85*1.18 ~= 1 and 0.70*1.40 ~= 1. They test
    // the benefit of transient unloading, rather than accidentally testing a
    // weaker pull at the same time. Lower-force variants remain as separate
    // candidates, allowing the measured energy penalty to prefer them when
    // their retained progress is comparable.
    // Candidate zero is the measured baseline.  The next four candidates
    // alter exactly one physical mechanism (unloading depth, rear assist,
    // pull magnitude, then lift/pull mix); the last candidate combines only
    // bounded changes.  Keeping this set small is deliberate: a 20--30 s
    // physical stroke can compare all mechanisms online instead of spending
    // the entire experiment on several nearly identical mix offsets.
    // Rear reverse is capped below 1 N per arm and is released only during
    // measured pressure-driven flexion.
    const std::array<TrajectoryChoice,choice_count> choices{{
        {0.,1.,1.,0.},
        {0.,1.,1.15,0.},
        {0.,1.,1.,.35},
        {0.,1.15,1.,0.},
        {-.15,1.18,1.,0.},
        {-.15,1.18,1.20,.50}}};
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
  // A single safe, completed forward stroke is not sufficient evidence for
  // strengthening a later command, but it is sufficient evidence that a
  // *smaller* residual goal should not receive the same full stroke again.
  // Return only the latest comparable baseline observation; the caller may
  // use it for reduction, never for amplification.  This distinction avoids
  // repeating a measured 20-mm stroke when only 8--10 mm remain while
  // preserving the two-sample requirement for ordinary stride prediction.
  double provisionalProgressForReduction(const Pair& bend,double roll,
                                          double pitch) const {
    for(auto it=samples.rbegin();it!=samples.rend();++it) {
      const auto& s=*it;
      if(s.choice!=0 || !s.safe || !s.completed || s.progress<.002 ||
         !std::isfinite(s.command_scale) || s.command_scale<.2 ||
         contextDistance(bend,s.bend,roll,pitch,s)>4.) continue;
      return clamp(s.progress/s.command_scale,.002,.05);
    }
    return std::numeric_limits<double>::quiet_NaN();
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
    // Forward travel is a feasibility condition, not merely one term which a
    // low-energy backwards stroke can trade away. Sub-mm rocking/idle is also
    // always inferior to a repeatable forward stroke.
    const double reverse_penalty=dx<-.0005?20.+5000.*std::abs(dx):0.;
    const double score=1000.*dx/seconds-
        .002*effort/std::max(1.,1000.*std::max(0.,dx))-
        recovery/seconds-reverse_penalty-(dx<.001?2.:0.)-
        (safe?(completed?0.:.25):3.);
    samples.push_back({current,bend,roll,pitch,score,dx,command_scale,completed,safe});
    // A poor exploratory stroke is useful evidence, but it must not be
    // followed immediately by another unknown condition. Return once to the
    // baseline and require a safely completed support transfer before the
    // next local probe.  Requiring the baseline to move forward deadlocks the
    // search on the exact high-friction contact that needs a stronger pull:
    // the unchanged waveform can be safe and repeatable while retaining a
    // small negative displacement.
    if(current!=0 && (!completed || !safe || dx<.001))
      baseline_revalidation_remaining=1;
    else if(current==0 && safe && completed &&
            baseline_revalidation_remaining)
      --baseline_revalidation_remaining;
    if(samples.size()>70) samples.erase(samples.begin());
  }
  void select(const Pair& bend,double roll,double pitch) {
    if(!enabled) {current=0;return;}
    // Do not stack an exploratory force/mix change on an already biased
    // support state.  The unchanged waveform may still advance and supplies
    // a new contact observation; exploration resumes after regrasp/recovery
    // brings the boundary roll inside this deliberately tighter band.
    if(std::abs(roll)>8.*M_PI/180.) {current=0;return;}
    std::array<unsigned,choice_count> count{};
    std::array<double,choice_count> sum{};
    std::array<unsigned,choice_count> positive{},safe_completed{},reverse{};
    unsigned n=0;
    for(const auto& s:samples) if(contextDistance(bend,s.bend,roll,pitch,s)<=4. &&
                                  std::abs(s.command_scale-1.)<=.02) {
      ++count[s.choice];sum[s.choice]+=s.score;++n;
      if(s.safe && s.progress>=.001) ++positive[s.choice];
      if(s.safe && s.completed) ++safe_completed[s.choice];
      if(!s.safe || s.progress<-.0005) ++reverse[s.choice];
    }
    if(!n) {current=0;return;}
    if(baseline_revalidation_remaining) {current=0;return;}
    // Identification already established the unloading/flexion waveform.
    // One retained-forward baseline therefore permits a small local probe;
    // failed/idle baselines do not.  Goal-distance prediction remains more
    // conservative and still requires two repeatable samples above.
    // A retained-forward baseline may start local exploration immediately.
    // On high-friction compliant tissue, however, the unchanged waveform can
    // be safely repeatable yet move backward. Requiring positive travel in
    // that case traps the controller at the losing baseline forever. Two
    // completed safe baseline observations are enough to permit one bounded
    // local probe; unsafe/incomplete trials never unlock exploration.
    // A safe completed baseline which moved backward is already sufficient
    // evidence that repeating exactly the same trajectory is not useful at
    // this new contact. Permit one bounded local probe immediately. The
    // stricter two-consistent-sample rule remains in progressEstimate(), so a
    // single result still cannot amplify goal-distance scheduling.
    if(positive[0]<baseline_confirmations && safe_completed[0]<1)
      {current=0;return;}
    // Repeat one positive exploratory sample once. Candidate comparison must
    // use a reproducible response rather than a one-off displacement.
    if(current!=0 && count[current]==1 && samples.back().choice==current &&
       samples.back().safe && samples.back().progress>=.001) return;
    // Explore finite local candidates; then revisit using a bounded confidence
    // bonus. Updates occur only at measured regrasp boundaries, never mid-pulse.
    for(unsigned i=0;i<count.size();++i) if(!count[i]) {current=i;return;}
    // Only a candidate with measured forward travel and no retained reversal
    // can become the deployed winner.
    double best=-std::numeric_limits<double>::infinity();
    unsigned best_index=0;
    for(unsigned i=0;i<count.size();++i) {
      if(!positive[i] || reverse[i]) continue;
      const double value=sum[i]/count[i]+.2*std::sqrt(std::log(1.+n)/count[i]);
      if(value>best) {best=value;best_index=i;}
    }
    current=std::isfinite(best)?best_index:0;
  }
};
}}
