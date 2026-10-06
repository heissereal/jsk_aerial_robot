#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>

namespace hugmy {
namespace hybrid {
using Pair = std::array<double, 2>;
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double slew(double x, double y, double up, double down, double dt) {
  return x + clamp(y-x, -down*dt, up*dt);
}
inline bool bottomPressureReady(double measured, double target, double normal_tolerance) {
  return std::isfinite(measured) && std::isfinite(target) && target>=0. &&
      std::isfinite(normal_tolerance) && normal_tolerance>=0. &&
      std::abs(measured-target)<=(target<=5. ? 5. : normal_tolerance);
}
struct Config {
  double period=.10, horizon=2., step=.05, budget_ms=12.;
  double thrust_max=6., thrust_rise=1., thrust_fall=12., pressure_max=50.;
  // Measured pneumatic ripple/overshoot around the bounded command.  This is
  // not additional command authority; target pressure remains pressure_max.
  double pressure_measurement_margin=8.;
  double pressure_slew=80., pressure_rate=22., exhaust_rate=30., pressure_delay=.08;
  double bend_tau=.35, bend_rate_tau=.12, moment_gain=8., roll_damping=1.5;
  double roll_limit=.261799, roll_rate_limit=.8, pitch_change_limit=.35;
  double bend_min=.523599, bend_max=2.61799, extension_limit=.349066, bend_rate_limit=3.14;
  // Pair-level request must not silently cap two 6-N rotors at 6 N total.
  // Individual thrust and upward-force constraints remain independent.
  double mismatch_limit=.261799, target_speed=.005, forward_force_max=12., up_force_max=6.;
  double run_min=.5, run_max=3., prepare_timeout=60., regrip_timeout=20., settle=.4;
  double stroke=.02, pressure_tolerance=5.;
  double energy_weight=.02, force_weight=2., ratio_weight=12., roll_weight=40.;
  double search_initial_thrust=1., search_thrust_step=.5, search_interval=.5;
  double search_thrust_weight=2.;
  double sliding_rear_pressure=0.;
  bool valid() const {
    for(double v:{period,horizon,step,budget_ms,thrust_max,thrust_rise,thrust_fall,
        pressure_max,pressure_measurement_margin,pressure_slew,pressure_rate,exhaust_rate,pressure_delay,bend_tau,
        bend_rate_tau,moment_gain,roll_damping,roll_limit,roll_rate_limit,pitch_change_limit,
        bend_min,bend_max,extension_limit,bend_rate_limit,mismatch_limit,target_speed,
        forward_force_max,up_force_max,run_min,run_max,prepare_timeout,regrip_timeout,settle,
        stroke,pressure_tolerance,energy_weight,force_weight,ratio_weight,roll_weight,
        search_initial_thrust,search_thrust_step,search_interval,search_thrust_weight,sliding_rear_pressure})
      if(!std::isfinite(v) || v<0.) return false;
    return period>=.02 && period<=.2 && step>=.02 && step<=period && horizon>=1.2 && horizon<=3. &&
      budget_ms>0 && thrust_max>0 && thrust_max<=6 && thrust_rise>0 && thrust_fall>0 &&
      pressure_max>0 && pressure_max<=55 && pressure_measurement_margin>=0 &&
      pressure_measurement_margin<=10 && pressure_slew>0 && pressure_rate>0 && exhaust_rate>0 &&
      pressure_delay<=.8 && bend_tau>0 && bend_rate_tau>0 && roll_limit>0 &&
      roll_rate_limit>0 && bend_min<bend_max && extension_limit>0 && bend_rate_limit>0 &&
      mismatch_limit>0 && target_speed>0 && forward_force_max>0 && up_force_max>0 && run_min>0 &&
      run_max>=run_min && run_max<=5 && prepare_timeout>0 && regrip_timeout>0 &&
      settle>0 && stroke>0 && search_initial_thrust>0 &&
      search_thrust_step>0 && search_thrust_step<=1. && search_interval>=.2 &&
      search_thrust_weight>0 && sliding_rear_pressure<=6.;
  }
};
struct State {
  double now=0., roll=0., roll_rate=0., pitch=0., pitch_start=0., speed=0., elapsed=0.;
  double bottom=0., bottom_command=0.;
  Pair pressure{}, command{}, bend{}, rate{}, thrust{}, start_bend{};
  Pair floor{{.523599,.523599}};
  bool fresh=false;
};
struct Force { double forward=0., up=0., roll_moment=0., side=0., pitch_moment=0.; };
// A cylinder cannot sustain unlimited transverse force while it is sliding.
// Reduce the side producing excess lateral force; never boost either rotor.
inline Pair balanceLateral(Pair force,const Pair& side,double roll,double rate) {
  const double desired=clamp(2.*roll+.6*rate,-.5,.5);
  double excess=force[0]*side[0]+force[1]*side[1]-desired;
  for(size_t j=0;j<2;++j) if(excess*side[j]>0. && std::abs(side[j])>1e-6) {
    const double reduction=std::min(force[j],excess/side[j]);
    force[j]-=reduction;excess-=reduction*side[j];
  }
  return force;
}
// Breakaway search in per-rotor N, not a tiny fixed horizontal force request.
// Only powered, feasible planning time counts. Integrating signed velocity
// over a window avoids latching on a single mocap velocity spike.
struct ThrustSearch {
  double target=1., time=0., displacement=0.;
  void reset(const Config& c) {target=std::min(c.search_initial_thrust,c.thrust_max);newWindow();}
  void newWindow() {time=0.;displacement=0.;}
  void observe(double speed,double dt,const Config& c) {
    if(!std::isfinite(speed) || !std::isfinite(dt) || dt<=0. || dt>.25) {newWindow();return;}
    time+=dt;displacement+=speed*dt;
    if(time+1e-9<c.search_interval) return;
    if(displacement/time<c.target_speed)
      target=std::min(c.thrust_max,target+c.search_thrust_step);
    newWindow();
  }
};
struct Plan {
  bool feasible=false, budget_exhausted=false;
  Pair thrust{}, pressure{}, delay{}, pull_thrust{};
  double duration=0., score=std::numeric_limits<double>::infinity();
  double forward_impulse=0., worst_roll=0., predicted_ratio=1., compute_ms=0.;
  unsigned evaluated=0, rejected=0;
  // model/geometry, bend/rate, lift, mismatch, roll (one reason per candidate)
  std::array<unsigned,5> rejection{};
};
// Measured pressure response, separated by filling/exhausting and by shared
// pneumatic load. These are empirical intervals, NOT certified safety bounds.
struct PneumaticResponse {
  double fill=22., exhaust=30., delay=.08, error=0.;
  unsigned samples=0;
};
struct CommandSample { double time; Pair pressure; };
class Model {
 public:
  std::array<std::array<PneumaticResponse,2>,2> pneumatic;
  Pair bend_bias{}, bend_error{};
  double rate_error=0., confidence=0.;
  unsigned accepted=0;
  std::deque<CommandSample> history;
  State previous;
  bool initialized=false;
  Pair edge_time{{-1.,-1.}}, edge_base{}, edge_sign{};
  void reset(const Config& c) {
    *this=Model();
    for(auto& a:pneumatic) for(auto& p:a) {p.fill=c.pressure_rate;p.exhaust=c.exhaust_rate;p.delay=c.pressure_delay;}
  }
  void newWindow() { history.clear(); initialized=false; edge_time.fill(-1.); confidence=0.; bend_bias.fill(0.); }
  static int load(const State& s) {
    int n=s.bottom_command>s.bottom+2.;
    for(size_t j=0;j<2;++j) n+=s.command[j]>s.pressure[j]+2.;
    return n>=2 ? 1 : 0;
  }
  double pastCommand(size_t j,double time,double fallback) const {
    double value=fallback;
    for(const auto& h:history) {if(h.time>time) break; value=h.pressure[j];}
    return value;
  }
  template<class Equilibrium>
  void observe(const State& s,const Config& c,Equilibrium eq) {
    if(!s.fresh) {newWindow();return;}
    for(double v:{s.now,s.roll_rate,s.bottom,s.bottom_command})
      if(!std::isfinite(v)) {newWindow();return;}
    for(size_t j=0;j<2;++j) {
      for(double v:{s.pressure[j],s.command[j],s.bend[j],s.rate[j],s.thrust[j]})
        if(!std::isfinite(v)) {newWindow();return;}
      if(!std::isfinite(eq(j,s.pressure[j],s.thrust[j]))) {newWindow();return;}
    }
    if(!initialized) {
      previous=s; initialized=true; history.push_back({s.now,s.command}); return;
    }
    const double dt=s.now-previous.now;
    if(dt==0.) return;
    if(dt<0. || dt>.25) {newWindow();return;}
    const int bucket=load(previous);
    bool compatible=true;
    for(size_t j=0;j<2;++j) {
      auto& p=pneumatic[j][bucket];
      const double delta=s.pressure[j]-previous.pressure[j];
      const double cmd=pastCommand(j,previous.now-p.delay,previous.pressure[j]);
      const double prediction=slew(previous.pressure[j],cmd,p.fill,p.exhaust,dt);
      p.error=.95*p.error+.05*std::abs(s.pressure[j]-prediction);
      // Identify latency on real command edges; never restart it on every tick.
      const double command_delta=s.command[j]-previous.command[j];
      const double old_gap=previous.command[j]-previous.pressure[j];
      if(edge_time[j]<0. && std::abs(command_delta)>.05 &&
         std::abs(s.command[j]-s.pressure[j])>3. &&
         (std::abs(old_gap)<=3. || old_gap*command_delta<0.)) {
        edge_time[j]=s.now;edge_base[j]=s.pressure[j];
        edge_sign[j]=s.command[j]>s.pressure[j]?1.:-1.;
      }
      if(edge_time[j]>=0. && edge_sign[j]*(s.pressure[j]-edge_base[j])>.5) {
        const double observed=s.now-edge_time[j];
        if(observed<=.8) p.delay=.9*p.delay+.1*observed;
        edge_time[j]=-1.;
      }
      if(edge_time[j]>=0. && s.now-edge_time[j]>1.5) edge_time[j]=-1.;
      const double rate=delta/dt, gap=cmd-previous.pressure[j];
      if(std::abs(gap)>3. && rate*gap>0. && std::abs(rate)>1. && std::abs(rate)<100.) {
        double& estimate=rate>0?p.fill:p.exhaust;
        estimate=.95*estimate+.05*std::abs(rate); ++p.samples;
      }
      const double e0=eq(j,previous.pressure[j],previous.thrust[j]);
      const double e1=eq(j,s.pressure[j],s.thrust[j]);
      // Incremental model stays anchored to the observed contact configuration.
      const double predicted_rate=previous.rate[j]+(e1-e0)/c.bend_tau+bend_bias[j]*dt;
      const double predicted_bend=previous.bend[j]+predicted_rate*dt;
      const double residual=s.bend[j]-predicted_bend;
      bend_error[j]=.95*bend_error[j]+.05*std::abs(residual);
      if(std::isfinite(residual) && std::abs(residual)<.08 && std::abs(delta)<8.)
        bend_bias[j]=clamp(.98*bend_bias[j]+.02*(s.rate[j]-predicted_rate)/dt,-.5,.5);
      else compatible=false;
    }
    rate_error=.95*rate_error+.05*std::abs(s.roll_rate-previous.roll_rate);
    if(compatible) {++accepted;confidence=std::min(1.,confidence+.01);}
    else {confidence=0.;bend_bias.fill(0.);}
    history.push_back({s.now,s.command});
    while(history.size()>1 && history[1].time<s.now-2.) history.pop_front();
    previous=s;
  }
};

// Two pressure-switch times, two early forces, two pull forces and a transfer
// duration are searched in simulation. No compulsory stop at the horizon end.
// Goal confirmation/recovery owns stopping. Only the first bounded move is
// executed; every later move is recomputed from fresh measurements.
class Planner {
 public:
  Model model;
  ThrustSearch thrust_search;
  bool hold_thrust_search=false;
  unsigned refinement_cursor=0;
  Plan last;
  double last_time=-1., force_request=.5;
  std::array<bool,2> pull_latched{{false,false}};
  void reset(const Config& c) { *this=Planner();model.reset(c);thrust_search.reset(c); }
  void beginRun() { last_time=-1.;pull_latched.fill(false);last=Plan();thrust_search.newWindow(); }
  static bool valid(const State& s,const Config& c,bool pressure_confirmation_pending=false) {
    if(!s.fresh || !c.valid()) return false;
    for(double x:{s.now,s.roll,s.roll_rate,s.pitch,s.pitch_start,s.speed,s.elapsed,s.bottom,s.bottom_command})
      if(!std::isfinite(x)) return false;
    if(s.elapsed<0 || std::abs(s.roll)>=c.roll_limit || std::abs(s.roll_rate)>=c.roll_rate_limit ||
       std::abs(s.pitch-s.pitch_start)>=c.pitch_change_limit) return false;
    for(size_t j=0;j<2;++j) {
      for(double x:{s.pressure[j],s.command[j],s.bend[j],s.rate[j],s.thrust[j],s.start_bend[j],s.floor[j]})
        if(!std::isfinite(x)) return false;
      // Only the adapter's bounded confirmation can defer this soft ceiling.
      // MPC calls retain strict default validation.
      if((!pressure_confirmation_pending &&
             (s.pressure[j]<-5 ||
              s.pressure[j]>c.pressure_max+c.pressure_measurement_margin)) ||
         s.command[j]<0 || s.command[j]>c.pressure_max ||
         s.thrust[j]<0 || s.thrust[j]>c.thrust_max || s.bend[j]<=std::max(c.bend_min,s.floor[j]) || s.bend[j]>=c.bend_max ||
         s.start_bend[j]-s.bend[j]>=c.extension_limit || std::abs(s.rate[j])>=c.bend_rate_limit) return false;
    }
    return true;
  }
  template<class Equilibrium,class Geometry>
  Plan update(const State& s,const Config& c,double minimum_ratio,Equilibrium eq,Geometry geometry) {
    if(!valid(s,c) || !std::isfinite(minimum_ratio) || minimum_ratio<0. || minimum_ratio>1.) {
      last=Plan();last_time=-1.;return last;
    }
    model.observe(s,c,eq);
    if(last_time>=0 && s.now-last_time>=0 && s.now-last_time<c.period-1e-6) return last;
    if(last_time>=0 && (s.now<last_time || s.now-last_time>.25)) {beginRun();last=Plan();return last;}
    // Increase the search target only during feasible powered motion. Hold it
    // once mean forward speed reaches the target; never bypass the predictor.
    const double previous_request=force_request;
    const auto previous_search=thrust_search;
    if(last_time>=0. && !hold_thrust_search) thrust_search.observe(s.speed,s.now-last_time,c);
    double forward_gain=0.;
    for(size_t j=0;j<2;++j) forward_gain+=std::max(0.,geometry(j,s.bend[j],s.roll,s.pitch).forward);
    force_request=clamp(thrust_search.target*forward_gain,.3,c.forward_force_max);
    last_time=s.now;
    Plan best;
    const auto started=std::chrono::steady_clock::now();
    const int steps=static_cast<int>(std::ceil(c.horizon/c.step));
    const double dt=c.horizon/steps;
    const int bucket=Model::load(s);
    const double grasp=std::min(30.,c.pressure_max);
    // Coordinate sweeps over a full-range seed avoid spending the budget only
    // on zero-force candidates. Each trial evaluates all uncertainty scenarios.
    Plan seed=last.feasible?last:Plan();
    // Computational status belongs to this call, not the warm-start waveform.
    seed.budget_exhausted=false;
    seed.duration=clamp(seed.duration>0?seed.duration:1.2,c.run_min,c.run_max);
    if(!last.feasible) {seed.delay={{.25,.25}};seed.pressure={{grasp,grasp}};}
    if(!last.feasible) for(size_t j=0;j<2;++j) {
      seed.thrust[j]=std::min(thrust_search.target,s.thrust[j]+.5);
      seed.pull_thrust[j]=thrust_search.target;
    }
    auto evaluate=[&](Plan trial) {
      // The candidate bank must not jump to 6 N while exploring at 1 N.
      for(size_t j=0;j<2;++j) {
        trial.thrust[j]=clamp(trial.thrust[j],0.,thrust_search.target);
        trial.pull_thrust[j]=clamp(trial.pull_thrust[j],0.,thrust_search.target);
      }
      if(minimum_ratio>=.999) trial.delay.fill(0.);
      if(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()>c.budget_ms) {
        best.budget_exhausted=true;return false;
      }
      ++best.evaluated;
      double worst=-1e30, worst_roll=0., impulse=0., ratio=1.;
      for(int scenario=0;scenario<5;++scenario) {
        Pair p=s.pressure,a=s.bend,w=s.rate,f=s.thrust,pc=s.command;
        std::deque<CommandSample> future{{s.now,s.command}};
        double roll=s.roll,wr=s.roll_rate,cost=0.,ix=0.;
        const double initial_moment=geometry(0,a[0],s.roll,s.pitch).roll_moment*f[0]+
                                    geometry(1,a[1],s.roll,s.pitch).roll_moment*f[1];
        double phase_ratio=minimum_ratio;
        for(int k=0;k<steps;++k) {
          const double t=k*dt;
          const double scale=scenario==1?1.6:(scenario==2?.65:1.);
          double fx=0.,fz=0.,moment=0.;
          // Retain the selected force split throughout the powered probe.
          // The old wall-clock blend reached m=1 before the thrust ramp had
          // produced any lift, making every outer candidate another D trial.
          // Regripping supplies the pressure-driven terminal flexion.
          phase_ratio=minimum_ratio;
          Pair requested{},side{};
          for(size_t j=0;j<2;++j) {
            const bool flex=pull_latched[j] || s.elapsed+t>=trial.delay[j] || minimum_ratio>=.999;
            const double force=flex?trial.pull_thrust[j]:trial.thrust[j];
            // The same roll and relative-extension brakes exist in prediction
            // and execution. Geometry signs, not assumed rotor IDs, set roll.
            const auto g0=geometry(j,a[j],roll,s.pitch);
            side[j]=g0.side;
            const double brake=std::max(0.,(4.*roll+1.5*wr)*g0.roll_moment*10.);
            const double other_extension=s.start_bend[1-j]-a[1-j];
            const double excess=std::max(0.,s.start_bend[j]-a[j]-other_extension-.035);
            requested[j]=clamp(force-brake-3.*excess,0.,c.thrust_max);
          }
          requested=balanceLateral(requested,side,roll,wr);
          for(size_t j=0;j<2;++j) {
            const double arm_scale=scenario<3?scale:((scenario==3)==(j==0)?1.6:.65);
            const bool flex=pull_latched[j] || s.elapsed+t>=trial.delay[j] || minimum_ratio>=.999;
            const double target_p=flex?trial.pressure[j]:0.;
            f[j]=slew(f[j],requested[j],c.thrust_rise,c.thrust_fall,dt);
            pc[j]=slew(pc[j],target_p,c.pressure_slew,c.pressure_slew,dt);
            const auto& response=model.pneumatic[j][bucket];
            const double lag=response.delay*arm_scale;
            double delayed;
            delayed=model.pastCommand(j,s.now+t-lag,s.command[j]);
            for(const auto& command:future) {
              if(command.time>s.now+t-lag) break;
              delayed=command.pressure[j];
            }
            p[j]=slew(p[j],delayed,response.fill/arm_scale,response.exhaust/arm_scale,dt);
            const double base=eq(j,s.pressure[j],s.thrust[j]);
            const double equilibrium=eq(j,p[j],f[j]);
            if(!std::isfinite(base) || !std::isfinite(equilibrium)) {++best.rejected;++best.rejection[0];return true;}
            const double target=s.bend[j]+equilibrium-base;
            // The learned acceleration residual is transient. Extrapolating
            // it as bias*t over two seconds invented sustained motion from
            // pressure/gyro noise (opposite +/-0.5 biases gave a false pair
            // mismatch even for the measured-command hold candidate).
            const double residual_rate=model.bend_bias[j]*c.bend_rate_tau*
                (1.-std::exp(-t/c.bend_rate_tau));
            const double target_rate=(target-a[j])/(c.bend_tau*arm_scale)+
                s.rate[j]*std::exp(-t/.25)+residual_rate;
            w[j]+=clamp(dt/c.bend_rate_tau,0.,1.)*(target_rate-w[j]);
            a[j]+=dt*w[j];
            const double margin=.01+std::min(.05,2.*model.bend_error[j]);
            if(a[j]<=std::max(c.bend_min,s.floor[j])+margin || a[j]>=c.bend_max-margin ||
               s.start_bend[j]-a[j]>=c.extension_limit-margin || std::abs(w[j])>=c.bend_rate_limit) {
              ++best.rejected;++best.rejection[1];return true;
            }
            const auto g=geometry(j,a[j],roll,s.pitch);
            if(!std::isfinite(g.forward) || !std::isfinite(g.up) || !std::isfinite(g.roll_moment)) {++best.rejected;++best.rejection[0];return true;}
            fx+=f[j]*g.forward;fz+=f[j]*g.up;moment+=f[j]*g.roll_moment;
            cost+=dt*c.energy_weight*f[j]*f[j];
            cost+=dt*c.search_thrust_weight*
                std::pow(std::max(0.,thrust_search.target-f[j]),2);
          }
          future.push_back({s.now+t+dt,pc});
          if(fz>c.up_force_max) {++best.rejected;++best.rejection[2];return true;}
          const double mismatch=(s.start_bend[0]-a[0])-(s.start_bend[1]-a[1]);
          if(std::abs(mismatch)>c.mismatch_limit) {++best.rejected;++best.rejection[3];return true;}
          wr+=dt*(c.moment_gain*(moment-initial_moment)*scale-c.roll_damping*(wr-s.roll_rate));
          roll+=dt*wr;
          worst_roll=std::max(worst_roll,std::abs(roll));
          if(std::abs(roll)>=c.roll_limit-.01 || std::abs(wr)>=c.roll_rate_limit) {++best.rejected;++best.rejection[4];return true;}
          ratio=std::max(0.,fx)/(std::max(0.,fx)+std::max(0.,fz)+1e-6);
          ix+=dt*fx;
          const double target_x=force_request*phase_ratio;
          const double target_up=force_request*(1.-phase_ratio);
          cost+=dt*(c.force_weight*(std::pow(fx-target_x,2)+std::pow(fz-target_up,2))+
              c.ratio_weight*force_request*force_request*std::pow(ratio-phase_ratio,2)+
              c.roll_weight*roll*roll+4.*wr*wr+4.*mismatch*mismatch);
        }
        // Fixed-horizon propulsion demand prevents the trivial solution of
        // ending immediately to avoid every running/effort cost. This is a
        // commanded force integral, NOT a prediction of contact slip/progress.
        const double required_impulse=force_request*minimum_ratio*c.horizon;
        cost+=2.*wr*wr+4.*std::pow(std::max(0.,required_impulse-ix),2)+.05*trial.duration;
        worst=std::max(worst,cost);impulse=ix;
      }
      if(worst<best.score) {
        const auto evaluated=best.evaluated,rejected=best.rejected;
        const auto rejection=best.rejection;
        const bool exhausted=best.budget_exhausted;
        best=trial;best.evaluated=evaluated;best.rejected=rejected;best.feasible=true;
        best.rejection=rejection;
        best.budget_exhausted=exhausted;
        best.score=worst;best.forward_impulse=impulse;best.worst_roll=worst_roll;best.predicted_ratio=ratio;
      }
      return true;
    };
    // Warm seed and bounded future force levels, then independent timing,
    // pressure, pull force and powered-duration coordinate refinement.
    evaluate(seed);
    // Checked hold candidate: do not stop merely because a warm waveform is
    // infeasible when actual pressure/thrust can still be held safely.
    Plan hold=seed;hold.pressure=s.command;hold.thrust=s.thrust;
    hold.pull_thrust=s.thrust;hold.delay.fill(0.);
    evaluate(hold);
    // A feasible 0.1-N move must not be hidden between coarse 0.8/1.5-N
    // candidates. Increment the last feasible request (before feedback).
    // Incrementing already-braked applied force subtracted the same roll/
    // extension correction twice and could never overcome its deadband.
    // Rollouts still start at MEASURED thrust and validate the full slew.
    Plan small_move=hold;
    for(size_t j=0;j<2;++j) small_move.thrust[j]=small_move.pull_thrust[j]=
        std::min(thrust_search.target,std::max(s.thrust[j],last.feasible?seed.pull_thrust[j]:s.thrust[j])+c.thrust_rise*c.period);
    evaluate(small_move);
    for(double delta:{-1.,1.}) {
      Plan pressure_move=small_move;
      for(size_t j=0;j<2;++j) pressure_move.pressure[j]=
          clamp(s.command[j]+delta,0.,c.pressure_max);
      if(!evaluate(pressure_move)) break;
    }
    // A wall-clock budget must not starve pressure/timing forever behind the
    // thrust bank. Try a different coupled pressure proposal on each update,
    // then rotate the first refinement dimension. Every proposal still goes
    // through the identical five-scenario feasibility test.
    const unsigned cursor=refinement_cursor++;
    Plan coupled=seed;
    coupled.thrust.fill(thrust_search.target);
    coupled.pull_thrust=coupled.thrust;
    const double pressure_delta=(cursor%2?10.:-10.);
    for(size_t j=0;j<2;++j)
      coupled.pressure[j]=clamp(seed.pressure[j]+pressure_delta,0.,c.pressure_max);
    evaluate(coupled);
    const int first_dimension=static_cast<int>((4+cursor)%9);
    for(int offset=0;offset<9 && !best.budget_exhausted;++offset) {
      const int dimension=(first_dimension+offset)%9;
      const Plan center=best.feasible?best:seed;
      for(int sign:{-1,1}) {
        Plan t=center;
        if(dimension<2) t.thrust[dimension]=clamp(t.thrust[dimension]+sign*.8,0.,c.thrust_max);
        else if(dimension<4) t.pull_thrust[dimension-2]=clamp(t.pull_thrust[dimension-2]+sign*.8,0.,c.thrust_max);
        else if(dimension<6) t.pressure[dimension-4]=clamp(t.pressure[dimension-4]+sign*10.,0.,c.pressure_max);
        else if(dimension<8) t.delay[dimension-6]=clamp(t.delay[dimension-6]+sign*.25,0.,.8);
        else t.duration=clamp(t.duration+sign*.4,c.run_min,c.run_max);
        for(size_t j=0;j<2;++j) if(pull_latched[j]) t.delay[j]=0.;
        if(!evaluate(t)) break;
      }
    }
    // If the warm pressure/timing is no longer feasible, vary these together
    // before spending the budget on thrust-only changes to that same waveform.
    // These remain fully predicted candidates, not an unchecked fallback.
    if(!best.feasible) for(double level:{0.,.5,1.}) {
      Plan t=seed;t.delay.fill(0.);t.pressure=s.pressure;
      for(auto& pressure:t.pressure) pressure=clamp(pressure,0.,c.pressure_max);
      t.thrust.fill(level*thrust_search.target);t.pull_thrust=t.thrust;
      t.duration=c.run_max;
      if(!evaluate(t)) break;
    }
    for(double level:{0.,.25,.5,.75,1.}) {
      Plan t=seed;t.thrust.fill(level*c.thrust_max);t.pull_thrust=t.thrust;
      if(!evaluate(t)) break;
    }
    for(int sweep=0;sweep<2 && !best.budget_exhausted;++sweep) {
      for(int dimension=0;dimension<9 && !best.budget_exhausted;++dimension) {
        const Plan center=best.feasible?best:seed;
        for(int sign:{-1,1}) {
          Plan t=center;
          if(dimension<2) t.thrust[dimension]=clamp(t.thrust[dimension]+sign*.8,0.,c.thrust_max);
          else if(dimension<4) t.pull_thrust[dimension-2]=clamp(t.pull_thrust[dimension-2]+sign*.8,0.,c.thrust_max);
          else if(dimension<6) t.pressure[dimension-4]=clamp(t.pressure[dimension-4]+sign*10.,0.,c.pressure_max);
          else if(dimension<8) t.delay[dimension-6]=clamp(t.delay[dimension-6]+sign*.25,0.,.8);
          else t.duration=clamp(t.duration+sign*.4,c.run_min,c.run_max);
          for(size_t j=0;j<2;++j) if(pull_latched[j]) t.delay[j]=0.;
          if(!evaluate(t)) break;
        }
      }
    }
    best.compute_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    if(!best.feasible) {force_request=previous_request;thrust_search=previous_search;}
    last=best;
    return last;
  }
  // Execute the first move only, with the same brakes and actuator slew as in
  // rollout. No new unmodelled thrust boost is added after feasibility testing.
  template<class Geometry>
  Pair firstThrust(const State& s,const Config& c,const Plan& p,double dt,Geometry geometry) const {
    Pair out{};
    if(!p.feasible || !valid(s,c) || !std::isfinite(dt) || dt<=0 || dt>.1) return out;
    Pair up{},side{},requested{};
    for(size_t j=0;j<2;++j) {
      const auto g=geometry(j,s.bend[j],s.roll,s.pitch);
      if(!std::isfinite(g.up) || !std::isfinite(g.roll_moment) || !std::isfinite(g.side)) return Pair{};
      up[j]=std::max(0.,g.up);
      side[j]=g.side;
      const double brake=std::max(0.,(4.*s.roll+1.5*s.roll_rate)*g.roll_moment*10.);
      const double excess=std::max(0.,s.start_bend[j]-s.bend[j]-(s.start_bend[1-j]-s.bend[1-j])-.035);
      requested[j]=clamp((p.delay[j]<=s.elapsed||pull_latched[j]?p.pull_thrust[j]:p.thrust[j])-brake-3.*excess,0.,c.thrust_max);
    }
    requested=balanceLateral(requested,side,s.roll,s.roll_rate);
    for(size_t j=0;j<2;++j) out[j]=slew(s.thrust[j],requested[j],c.thrust_rise,c.thrust_fall,dt);
    // Measured-angle protection between replans: pressure lag must never turn
    // a planned horizontal increase into excessive actual upward force.
    const double total_up=out[0]*up[0]+out[1]*up[1];
    if(total_up>c.up_force_max)
      for(size_t j=0;j<2;++j) if(up[j]>0.) out[j]*=c.up_force_max/total_up;
    return out;
  }
};
} // namespace hybrid
} // namespace hugmy
