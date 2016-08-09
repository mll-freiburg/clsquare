#ifndef _GRAPHIC_H_
#define _GRAPHIC_H_

#include "global.h"

/** @defgroup GRAPHIC Graphic Modules
  * The graphic modules for the CLS2 loop. */

/** Base class for all graphic modules in CLSquare */
class Graphic {
public:
  virtual ~Graphic(){;};

  /** Updates graphics
   * \param plant_state current plant state. 
   * \param observed_state current observed state. 
   * \param action executed action in current state.
   * \param cycle control cycle in current trial.
   * \param episode current episode number.
   * \param total_time amount of time since start of simulation loop.
   * \param episode_time amount of time since start of current trial.
   * \param total_num_of_cycles amount of control cycles since start of simulation loop.
   * \return true, for success. */
  virtual bool notify(const double *plant_state, const double *observation, const double *reference_input, 
                      const double *action, const long cycle, const long episode, const float total_time,
                      const float episode_time, const long total_num_of_cycles) =0;

  virtual void notify_episode_starts(long episode){;}

  virtual void notify_episode_stops(const double *plant_state, const double *observation, const double *reference_input,
                      const long cycle, const long episode, const float total_time,
                      const float episode_time, const long total_num_of_cycles){;}

  /** Initialize graphics.
   * \param plant_s_dim Dimension of plant state
   * \param observed_s_dim Dimension of observed  state
   * \param act_dim Dimension of action space.
   * \param delta_t Duration of one control cycle.
   * \param fname File, which contains conifiguration
   * \return true, for success. */ 
  virtual bool init(int _plant_state_dim, int _observation_dim, int _action_dim, int _reference_input_dim, double _delta_t, const char *fname=0)=0;

  /** Terminate graphics.
   * \return true for success */
  virtual bool deinit() =0; 

  /** Notifies that a command via pipe has arrived. */
  //  virtual void notfiy_command_string(char* buf){return;};
  virtual void notify_command_string(char* buf){
    IOUT("Warning: 'Got notified by pipe, but no notify command behaviour is implemented. Cmd string: "<<buf);
  }
};

#ifdef CLSQUARE
#include "registry.h"
#else
#define REGISTER_GRAPHIC(classname, desc)
#endif


#endif

