#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

#include "controller.h"
#include "setdef.h"

/** A standard PID controller for one dimensional action and one dimensional error variable.
  * Use this controller for a simple pid control law.
  * It is assumed that there is an control deviation in the observation vector at id \b error_id.
  * The action is calculated from the coefficients \b kp, \b ki, \b kd and placed in the action vector at id \b action_id.
  * The admissible ranges of the state and action are set by the SetDef \b xwork and \b awork, respectively.
  * A basic \b anti-windup heuristic can be activated with the parameter of the same name.
  *
  * @ingroup CONTROLLER
  * @ingroup STATIC
  **/
class StdPIDController : public Controller {
public:
    StdPIDController();
    virtual ~StdPIDController();

    virtual bool get_action (   const double    *observed_state,
                                double          *action);

    virtual bool init (         const int       observed_state_dim,
                                const int       action_dim,
                                double          deltat,
                                const char      *fname      =0,
                                const char      *chapter    =0);
    virtual void deinit();
    virtual void notify_episode_starts();
    virtual void notify_episode_stops(const double* current_observed_state);

protected:
    SetDef  xwork;                  ///< working range of observations (config option)
    SetDef  awork;                  ///< allowed range of actions (config option)
    long    episode_step_counter;   ///< counter for control steps in recent epsiode
    long    episode_counter;        ///< counter for episodes

    int     error_id;               ///< the id of the controlled error variable in the observation vector (config option)
    int     action_id;              ///< the id of the controlled action in the action vector (config option)

    double  kp;                     ///< control coeff proportional (config option)
    double  ki;                     ///< control coeff integral (config option)
    double  kd;                     ///< control coeff differential (config option)
    bool    anti_windup;            ///< if true, an anti windup heuristic is used (config option)

    double  last_error;             ///< helper variable to compute \f$\frac{\partial e}{\partial t}\f$
    double  error_integral;         ///< error integral (sum, as we are in discrete time domain)

    /** Read params from an configuration file.
      * @param fname file name of the configureation file
      * @param a subsection of the configuration file (standard: Controller)
      * @return true for success, false on error
      **/
    bool    read_options(const char* fname, const char* chapter);

    int     odim;                   ///< the observation dim
    int     adim;                   ///< the action dim
    double  dt;                     ///< length of a time step (s) of the control cycle
};

#endif
