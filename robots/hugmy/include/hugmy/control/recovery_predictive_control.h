#ifndef HUGMY_RECOVERY_PREDICTIVE_CONTROL_H
#define HUGMY_RECOVERY_PREDICTIVE_CONTROL_H

#include <hugmy/control/continuous_roll_recovery.h>
#include <chrono>

namespace hugmy {
// Small, deterministic scenario MPC for the selected roll-recovery pair.
// Finite scenarios are engineering hypotheses, NOT certified uncertainty bounds.
// Two thrust knots and independent pressure targets; only the first move is used.
struct RecoveryPredictiveConfig {
  bool enabled = true, shadow = true, adapt = true;
  double period = 0.10, horizon = 2.0;
  double budget_ms = 8.0; // bounded search; never interrupt the 50-Hz guards indefinitely
  double pressure_min = 0.0, pressure_max = 30.0, pressure_slew = 10.0;
  double pressure_step = 2.0, thrust_step = 0.6;
  double pressure_tau = 0.6, bend_tau = 0.35;
  double moment_gain = 8.0, damping = 1.5; // local acceleration/Nm, damping /s
  double bend_band = 0.1745329252, max_rate = 0.5;
  double allowed_worsening = 0.0872664626;
};
struct RecoveryPredictiveState {
  double now = 0., error = 0., rate = 0., thrust = 0., bottom = 0.;
  std::array<double, 2> pressure{}, pressure_command{}, bend{};
};
struct RecoveryPredictivePlan {
  bool feasible = false;
  bool budget_exhausted = false;
  double thrust = 0., later_thrust = 0., score = 0., worst_terminal_error = 0.;
  std::array<double, 2> pressure{};
  unsigned evaluated = 0, rejected = 0;
};
struct RecoveryPredictiveControl {
  RecoveryPredictivePlan plan;
  RecoveryPredictiveState previous;
  bool initialized = false;
  double last_plan = -1., acceleration_bias = 0., confidence = 0.;
  double prediction_error = 0.;
  double pressure_prediction_error = 0., bend_prediction_error = 0.;
  std::array<double, 2> pressure_tau{{.6, .6}}, bend_bias{};
  unsigned accepted_observations = 0;
  void reset() { *this = RecoveryPredictiveControl(); }
  static bool validConfig(const RecoveryPredictiveConfig& c, const RollRecoveryConfig& l) {
    for(double value:{c.period,c.horizon,c.budget_ms,c.pressure_min,c.pressure_max,
        c.pressure_slew,c.pressure_step,c.thrust_step,c.pressure_tau,c.bend_tau,
        c.moment_gain,c.damping,c.bend_band,c.max_rate,c.allowed_worsening,
        l.limit,l.rise,l.fall,l.stop,l.max_target_rate,l.angle_gain})
      if(!std::isfinite(value)) return false;
    return c.period>0. && c.period<=.25 && c.horizon>=.4 && c.horizon<=3. &&
        c.budget_ms>0. && c.pressure_min>=0. && c.pressure_max>=c.pressure_min &&
        c.pressure_slew>0. && c.pressure_step>0. && c.thrust_step>0. &&
        c.pressure_tau>0. && c.bend_tau>0. && c.moment_gain>0. && c.damping>=0. &&
        c.bend_band>=0. && c.max_rate>0. && c.allowed_worsening>=0. &&
        l.limit>0. && l.rise>0. && l.fall>0. && l.stop>=0. &&
        l.max_target_rate>0. && l.angle_gain>0.;
  }
  static bool valid(const RecoveryPredictiveState& s) {
    if (!std::isfinite(s.now) || !std::isfinite(s.error) || !std::isfinite(s.rate) ||
        !std::isfinite(s.thrust) || !std::isfinite(s.bottom) || s.thrust < 0.) return false;
    for (size_t j=0; j<2; ++j)
      if (!std::isfinite(s.pressure[j]) || !std::isfinite(s.pressure_command[j]) ||
          !std::isfinite(s.bend[j]) || s.bend[j] < -0.174533 || s.bend[j] > 2.966) return false;
    return true;
  }
  template<class Equilibrium, class Lever>
  void observe(const RecoveryPredictiveState& s, const RecoveryPredictiveConfig& c,
               Equilibrium equilibrium, Lever lever) {
    if (!initialized) {
      pressure_tau.fill(c.pressure_tau); previous=s; initialized=true; return;
    }
    const double dt=s.now-previous.now;
    if (dt < c.period*.5) return;
    bool compatible=dt<=.25 && std::abs(s.bottom-previous.bottom)<3.;
    pressure_prediction_error=bend_prediction_error=0.;
    for(size_t j=0;j<2;++j)
      compatible=compatible && std::abs(s.bend[j]-previous.bend[j])<.25 &&
          std::abs(s.pressure[j]-previous.pressure[j])<5.;
    for(size_t j=0;j<2;++j) {
      const double predicted_pressure=previous.pressure[j]+(previous.pressure_command[j]-
          previous.pressure[j])*(1.-std::exp(-dt/pressure_tau[j]));
      const double predicted_bend=previous.bend[j]+(equilibrium(j,previous.pressure[j],
          .5*(previous.thrust+s.thrust))+bend_bias[j]-previous.bend[j])*
          (1.-std::exp(-dt/c.bend_tau));
      compatible=compatible && std::isfinite(predicted_bend) &&
          std::abs(predicted_pressure-s.pressure[j])<3. &&
          std::abs(predicted_bend-s.bend[j])<.20;
      pressure_prediction_error=std::max(pressure_prediction_error,std::abs(predicted_pressure-s.pressure[j]));
      bend_prediction_error=std::max(bend_prediction_error,std::abs(predicted_bend-s.bend[j]));
    }
    // Predict with actual applied input, never with an unexecuted shadow plan.
    double tau_before=0., tau_now=0.;
    for(size_t j=0;j<2;++j) {
      tau_before+=previous.thrust*lever(j, previous.bend[j]);
      tau_now+=s.thrust*lever(j, s.bend[j]);
    }
    const double nominal_accel=c.moment_gain*(tau_now-tau_before);
    const double innovation=(s.rate-previous.rate)/dt-nominal_accel-acceleration_bias;
    prediction_error=std::abs(innovation)*dt;
    compatible=compatible && std::isfinite(innovation) && std::abs(innovation)<.5;
    if (!compatible) {
      // Contact change, external disturbance or possible manual intervention:
      // discard local adaptation; do not learn a new actuation sign from it.
      confidence=0.; acceleration_bias=0.; bend_bias.fill(0.);
      pressure_tau.fill(c.pressure_tau); accepted_observations=0;
      last_plan=-1.; // never reuse a plan across an observed contact change
    } else if(c.adapt) {
      ++accepted_observations;
      confidence=std::min(1., confidence+.02);
      acceleration_bias=recoveryClamp(acceleration_bias+.05*innovation, -.3, .3);
      for(size_t j=0;j<2;++j) {
        const double dp=(s.pressure[j]-previous.pressure[j])/dt;
        const double gap=previous.pressure_command[j]-previous.pressure[j];
        if(std::abs(gap)>2. && std::abs(dp)>.5 && gap*dp>0.) {
          const double observed=recoveryClamp(gap/dp,.15,2.);
          pressure_tau[j]+=.05*(observed-pressure_tau[j]);
        }
        // Bias updates only on nearly settled arm/pressure samples. The table
        // is an equilibrium map, not a transient or contact-force sensor.
        if(std::abs(s.bend[j]-previous.bend[j])/dt<.05 && std::abs(dp)<1.) {
          const double residual=s.bend[j]-equilibrium(j,s.pressure[j],s.thrust);
          if(std::isfinite(residual)) bend_bias[j]=recoveryClamp(
              bend_bias[j]+.05*(residual-bend_bias[j]),-.5,.5);
        }
      }
    }
    previous=s;
  }
  template<class Equilibrium, class Lever>
  RecoveryPredictivePlan update(const RecoveryPredictiveState& s,
      const RecoveryPredictiveConfig& c, const RollRecoveryConfig& limits,
      Equilibrium equilibrium, Lever lever) {
    if(!valid(s) || !validConfig(c,limits) || s.thrust>limits.limit+1.e-6 ||
       (initialized && (s.now<previous.now || s.now-previous.now>.25))) {
      reset(); return plan;
    }
    observe(s,c,equilibrium,lever);
    if(last_plan>=0. && s.now-last_plan<c.period-1e-6) return plan;
    last_plan=s.now; plan=RecoveryPredictivePlan();
    // nominal, slow/weak, fast/strong, left/right asymmetric responses.
    const std::array<std::array<double,4>,5> scenarios{{
      {{1.,1.,1.,0.}}, {{1.8,1.8,.5,-.01}}, {{.5,.5,1.5,.01}},
      {{1.6,.6,.7,-.005}}, {{.6,1.6,1.3,.005}} }};
    const int steps=static_cast<int>(std::ceil(c.horizon/.10));
    const double dt=c.horizon/steps;
    double best=std::numeric_limits<double>::infinity();
    // Uncertainty limits each re-plan's change, not its absolute force.
    const double trust=c.thrust_step*(.5+.5*confidence);
    // Include reachable future extension forces, not just tiny local changes:
    // otherwise a static-friction/extension threshold creates a zero-force
    // local minimum. Only the first move is trust/slew limited below.
    const std::array<double,8> forces{{0., std::max(0.,s.thrust-trust),s.thrust,
      std::min(limits.limit,s.thrust+trust),limits.limit*.25,limits.limit*.5,
      limits.limit*.75,limits.limit}};
    const auto started=std::chrono::steady_clock::now();
    // Explore all force levels at a pressure pair before refining pressures;
    // a time-limited search must not only visit the smallest thrusts.
    for(int p0=-1;p0<=1;++p0) for(int p1=-1;p1<=1;++p1)
      for(double first:forces) for(int tail=0;tail<3;++tail) {
        if(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()>c.budget_ms) {
          plan.budget_exhausted=true;
          return plan; // only fully evaluated scenarios can have set feasible
        }
        ++plan.evaluated;
        const double second=tail==0?0.:(tail==1?first:limits.limit);
        const std::array<double,2> pc{{
          recoveryClamp(s.pressure_command[0]+p0*c.pressure_step,c.pressure_min,c.pressure_max),
          recoveryClamp(s.pressure_command[1]+p1*c.pressure_step,c.pressure_min,c.pressure_max)}};
        double worst=0., worst_error=-1e9; bool feasible=true;
        for(const auto& scenario:scenarios) {
          double e=s.error,w=s.rate,f=s.thrust,cost=0.;
          auto p=s.pressure, a=s.bend, command=s.pressure_command;
          const double initial_moment=s.thrust*(lever(0,a[0])+lever(1,a[1]));
          for(int k=0;k<steps;++k) {
            // Include the same first-move trust clamp as the executed output,
            // including on rundown; never predict faster braking than allowed.
            const double desired=k*dt<c.period-1.e-9
                ? recoveryClamp(first,std::max(0.,s.thrust-trust),std::min(limits.limit,s.thrust+trust))
                : (k<steps/2?first:second);
            f+=recoveryClamp(desired-f,-limits.fall*dt,limits.rise*dt);
            double moment=0.;
            for(size_t j=0;j<2;++j) {
              command[j]+=recoveryClamp(pc[j]-command[j],-c.pressure_slew*dt,c.pressure_slew*dt);
              const double scale=scenario[j];
              p[j]+=(command[j]-p[j])*(1.-std::exp(-dt/(pressure_tau[j]*scale)));
              const double eq=equilibrium(j,p[j],f)+bend_bias[j];
              if(!std::isfinite(eq)) {feasible=false;break;}
              a[j]+=(eq-a[j])*(1.-std::exp(-dt/(c.bend_tau*scale)));
              a[j]=recoveryClamp(a[j],0.,2.966);
              moment+=(lever(j,a[j])+scenario[3])*f;
            }
            if(!feasible || !std::isfinite(moment) || !std::isfinite(initial_moment)) {feasible=false;break;}
            // Local incremental contact model anchored at measured roll rate.
            // No full-gravity term and no inversion of a signed/tiny lever.
            const double acceleration=acceleration_bias+
              c.moment_gain*scenario[2]*(moment-initial_moment)-c.damping*(w-s.rate);
            w+=dt*acceleration; e-=dt*w;
            if(!std::isfinite(e) || !std::isfinite(w) ||
               std::abs(w)>std::max(c.max_rate,std::abs(s.rate))+.01 ||
               e>s.error+c.allowed_worsening || e < -limits.stop) {feasible=false;break;}
            const double desired_rate=std::min(limits.max_target_rate,
                limits.angle_gain*std::max(0.,e));
            const double shape=std::pow(std::max(0.,a[0]-c.bend_band),2)+
                               std::pow(std::max(0.,a[1]-c.bend_band),2);
            // Recovery/shape costs dominate effort. Zero force is a candidate,
            // not a default forced by low confidence. Shape is deliberately
            // soft so braking can override the straight-arm objective.
            cost+=dt*(20.*e*e+8.*std::pow(w-desired_rate,2)+2.*shape+.02*f*f);
          }
          if(!feasible) break;
          cost+=30.*e*e;
          worst=std::max(worst,cost);worst_error=std::max(worst_error,e);
        }
        if(!feasible) {++plan.rejected;continue;}
        worst+=.10*std::pow(first-s.thrust,2)+.002*(
          std::pow(pc[0]-s.pressure_command[0],2)+std::pow(pc[1]-s.pressure_command[1],2));
        if(worst<best) {
          best=worst;plan.feasible=true;
          plan.thrust=recoveryClamp(first,std::max(0.,s.thrust-trust),std::min(limits.limit,s.thrust+trust));
          plan.later_thrust=second;
          plan.pressure=pc;plan.score=worst;plan.worst_terminal_error=worst_error;
        }
      }
    return plan;
  }
};
} // namespace hugmy
#endif
