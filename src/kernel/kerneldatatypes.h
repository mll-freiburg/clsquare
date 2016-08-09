#ifndef _KERNELDATATYPES_H_
#define _KERNELDATATYPES_H_

#include <string>

/** Time information of the system for a state or transition,
* containing different time measures in episodes, cycles and total values
* taken so far up to the respective point in time.
**/
struct sys_time_struct {
    sys_time_struct();
    void        reset();                         ///< reset all values to 0 (begin of program)

    long        cycle_ctr;                       /*!< Number of time steps in recent episode [number]*/
    long        episode_ctr;                     /*!< Number of episodes processed so far.*/
    long        total_num_of_cycles;             /*!< Sum of all time steps taken so far (over all episodes) [number]*/
    double      total_time;                      /*!< Sum of all time steps taken so far (over all episodes) [s]*/
    double      episode_time;                    /*!< The value of time variable t in [s], corresponding to state x(t)*/
    long double total_real_time;                 ///< Real time since Program start [s]
    double      delta_t;                         /*!< Time between control steps in [s] (determines control frequency).*/
};

/** Dimension information for the system vectors. **/
struct sys_dim_struct {
    sys_dim_struct();
    void reset();                             ///< reset all values to (begin of program)

    int      plant_state_dim;                 /*!< The dimension of the plant state x (set by plant module)*/
    int      measurement_dim;                 /*!< The dimension of the plant measurement o (set by plant module)*/
    int      observed_state_dim;              /*!< The dimension of the observed state (set by observer)*/
    int      action_dim;                      /*!< The dimension of the action u (set by plant module)*/
    int      external_signal_dim;             /*!< The dimension of the external signal r (set by plant module)*/
};

/** The vectors for system variables concerning states. **/
struct sys_state_variables {
    double  *plant_state;                   /*!< The plant state vector. This is an internal state of the plant. */
    double  *measurement;                   /*!< The measurement vector communicated to the observer. The measurement of the plant_state, produced by plant.*/
    double  *external_signal;               /*!< External signal for plant (if plant needs one). ADVANCED USAGE to enable external disturbances or reference values for feedback control.*/
    double  *observed_state;                /*!< The observed state as input for the controller. Generated from the observer module, based on the measurements and actions (over time).  */
};

/** Status of the system */
struct sys_struct{
  sys_struct();                             /*!< Constructor (empty, not allocated) */
  bool alloc();                             /*!< Allocate the arrays.*/
  bool free();                              /*!< Free memory.*/
  void reset_sys_vars();                    /*!< Set default values for variables.*/

  sys_dim_struct        dim;                ///< the dimesnion of the system variables
  sys_state_variables   prev_state_vars;    ///< state variables for t-1
  sys_state_variables   current_state_vars; ///< state variables for t
  sys_state_variables   next_state_vars;    ///< state variables for t+1
  double                *current_action;    /*!< action a(t) choosen by controller */
  double                *prev_action;       /*!< action a(t-1) choosen by controller */
  sys_time_struct       current_time;       ///< the current time information
};


/** Specification for the task.*/
struct spec_struct {
  spec_struct();

  int  verbosity;                         /*!< The verbosity level of the mainloop */
  bool interactive_mode;                   /*!< Allow commands from pipe */
  std::string config_fname;               /*!< The name of the configuration file (given as cmdline arg)*/
  long num_episodes;                      /*!< The number of episodes to be executed.*/
  long cycles_per_episode;                /*!< The number of cycles in each episode.*/
  long call_cmd_freq;                     /*!< The number of episodes between two calls of the aux call_cmd (0), values <=0 means no execution of the aux cmd*/
  std::string call_cmd;                   /*!< The call_cmd is a command that is executed in the terminal (e.g. another clsquare with test episodes).
                                           The command is called with parameters: spec.call_cmd, sys.episode_ctr, sys.total_num_of_cycles, sys.total_time.*/
  long sleep_every_cycle;                 /*!< Number of [ms] (active) sleep to slow down simulation (e.g. for visualization) (0) \attention do not use this with real or realtime plants!!*/
  long max_initial_retries;               /*!< The number of retries on plant init state rejections. */
  long max_external_signal_retries;       /*!< The number of retries on plant external signal rejection */
  std::string plant_module_name;          /*!< Identifier string for the used plant module. */
  std::string controller_module_name;     /*!< Identifier string for the used controller module. */
  std::string reward_module_name;         ///< Identifier string for the used reward module.
  std::string graphic_module_name;        /*!< Identifier string for the used graphic module. */
  std::string statistics_module_name;     /*!< Identifier string for the used statistics module. */
  std::string observer_module_name;       /*!< Identifier string for the used observer module. */
  std::string input_module_name;          /*!< Identifier string for the used input module. */
  std::string output_module_name;         /*!< Identifier string for the used output module. */
  int reward_type;                        ///< indicates which state representation to use to query the reward module(plant_state, measurement or observed_state)
  bool do_not_start_in_terminal_states;   ///< whether or not to accept terminal states as the initial state of an episode

};

#endif
