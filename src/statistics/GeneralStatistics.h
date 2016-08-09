/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Martin Riedmiller

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef _GENERALSTATISTICS_H_
#define _GENERALSTATISTICS_H_
#define MAX_COSTS 1000

#include "setdef.h"
#include "global.h"
#include "statistics.h"

  /** Provides useful statistical information about the performance of the used controller.
   * \todo documentation for different statistical modes.
   * \todo interfaces for plant/controller rewards
   * \todo statistic about plant/controller rewards
   * \todo code is quite c-style ... not really clear ... full code revision ?
   * \todo factory concept to split up different modes ?
   *
   * @ingroup STATISTICS
   */
class GeneralStatistics : public Statistics {
 public:

  /** The statistics module is notified every control cycle about the current state of the process.   
   * Calls the update method for the statistics of current episode.
   * \param plant_state Current plant state. 
   * \param measurement Current measurement. 
   * \param observation Current observed state. 
   * \param action Executed action in current state.
   * \param cycle Control cycle in current trial.
   * \param episode Current trial.
   * \param total_time Amount of time since start of simulation loop.
   * \param episode_time Amount of time since start of current trial.
   * \param total_num_of_cycles Amount of control cycles since start of simulation loop.
   * \return true, for success. */
  void notify(const double* plant_state, const double* measurement,
              const double* observation, const double* action,
              const long cycle, const long episode,
              const float total_time, const float episode_time, const long total_num_of_cycles, double transition_rating_signal);

  /** Initializes statistics module.
   * \param plant_state_dim Dimension of plant state
   * \param measurement_dim Dimension of measurement
   * \param observation_dim Dimension of observed state
   * \param act_dim Dimension of action space.
   * \param delta_t Duration of one control cycle.
   * \param fname File, which contains conifiguration
   * \return true, for success. */ 
  bool init(int _plant_state_dim, int _measurement_dim, int _observation_dim, int _action_dim, 
            double _delta_t, long _cycles_per_episode, const char *fname=0);

  /** End statistics. */
  void deinit();

  /** Notifies, that a trial has started. 
   * Calls the method to initialize the statistics of current episode. 
   * \param episode_ctr Current episode. */
  void notify_episode_starts(const long episode_ctr);

  /** Notifies, that a trial has stopped. 
   * Collect all remaining statistical information of current episode. 
   * \param plant_state Current plant state. 
   * \param observation Current observed state. 
   * \param episode_ctr Current trial. */   
  void notify_episode_stops(const double* plant_state, const double* measurement, const double* observation,
                            const long cycle, const long episode,
                            const float total_time, const float episode_time, const long total_num_of_cycles, bool is_terminal, double terminal_rating_signal);

 protected:
  /** No legend of statistics is printed into the statistics file. */
  bool noheader;

  int header_mode;

  /** Definition of goal states. */
  SetDef XplusSet;

  /** Definition of states to avoid. */ 
  SetDef XminusSet;

  /** Definition of the working area of the process. */
  SetDef XworkSet;

  /** Statistical mode.
   * \li 0: standardized
   * \li 1: raw  
   */
  int GeneralStatistics_mode;

  /** Dimension of observation space. */
  int state_dim;
  int plant_state_dim, measurement_dim, observation_dim;
  
  /** Input type to be used. */
  enum {PlantState, Measurement, Observation} state_mode;

  /** Dimension of action space. */
  int action_dim;

  /** Duration of one control cycle in seconds. */
  float delta_t;

  /** Maximal length of a trial in control cylcles. */
  long max_episode_length;

  /** Frequency of an output (print to file) of the statistic module. */
  long averageOverNEpisodes; 

  /** Filename of statistics file. */
  char ProtFName[MAX_STR_LEN];
    
  /** Outsput stream (stream to file) of statistics.  */
  std::ostream *out;

  int  err_obs_value_id;
  long err_start_cycle;

  /** Open file with statistics. */
  bool open_file();

  void print_meaning();

  /** Reads configuration of statistics module.
      \param fname file which contains configuration of statistics module.
      \return true for success 

      Possible options:
      \li xplus: terminal states 
      \li xwork: working area
      \li average_over_n_episodes: average over a number of episodes
      \li statistics_mode: mode
      \li noheader: prints no legend */  
  bool read_options(const char * fname);

  /** Initialize statistics for current episode. */
  void init_currPerf();

  /** Initialize cummulated statistics. */
  void init_accPerf();

  /** Collect some statistical information at the end of a trial. */
  void finish_episode(const double* state);

  /** Updates statistics for current episode. */
  void update_currPerf(const double* state, const double* action, const double reward);

  /** Updates cummulated statistics. */
  void update_accPerf();

  /** Outputs cummulated statistics and resets all statistics */
  void write2file(const long episode);

  /** Outputs cummulated statistics (mode:standard benchmark) and resets all statistics. */
  void write2file_mode0(const long episode);

  /** Outputs cummulated statistics (mode:raw) and resets all statistics. */
  void write2file_mode1(const long episode);
 

  struct{
    /** touched a goal state on current episode. */
    bool touchedXplus;
    
    /** touched an "avoid" state on current episode. */
    bool touchedXminus;

    /** touched a state outside of working area on current episode. */
    bool touchedOutofXwork;

    /** current episode terminated in a goal state. */
    bool terminatedInXplus;

    /** touched an "avoid" state or a state outside the working area on current episode. */
    bool crashed;

    /** Number of control cycles outside the goal area on current episode. */
    long cyclesOutofXplus;

    /** Number of control cycles inside the goal area on current episode. */
    long cyclesInXplus;

    /** Number of control cycles until the current episode crashed. */
    long cyclesUntilCrashed;   

    /** Number of control cycles until a goal state is reached on current episode. */
    long cyclesUntilReachedXplus;

    /** Number of control cycles on current episode. */
    long num_cycles;

    /** Number of control cycles until the process stays permanently in the goal area on current episode. */
    long cyclesBeforeAsymptotic;
    
    /** Costs of a episode. */
    long costs;

    /** Rewards of a episode. */
    double reward;

    /** Final rewards of a episode. */
    double final_reward;

    double last_cycles_error_sum;
    long   last_cycles_error_ctr;
    
  } currPerf; 
  
  
  struct{
     /** Number of episodes that touched a goal state. */
    long touchedXplus;
    
    /**  Number of episodes that touched an "avoid" state. */
    long touchedXminus;

    /**  Number of episodes that touched a state outside the working area.  */
    long touchedOutofXwork;

    /**  Number of episodes that terminated in a goal state. */
    long terminatedInXplus;
   
    /** Number of episodes that crashed. */
    long crashed;

    /** Average (over all episodes) number of control cycles out of the goal area. */
    double average_cyclesOutofXplus;

    /** Average (over all episodes) number of control cycles in the goal area. */
    double average_cyclesInXplus;

    /** Average (over all episodes) number of control cycles until a episode crashed. */
    double average_cyclesUntilCrashed;   

    /** Average (over all episodes) number of control cycles until a episode reached a goal state. */
    double average_cyclesUntilReachedXplus;

    /** Maximum (over all episodes) number of control cycles until process stayed permanently in goal area. */
    double maxCyclesBeforeAsymptotic;
    
    /** Average costs over all episodes. */
    double average_costs;

    /** Average reward over all episodes. */
    double average_reward;

    /** Average final reward over all episodes. */
    double average_final_reward;

    /** Cummulated control cycles over all episodes. */
    long cyclesTotal;

    /** Number of episodes. */
    long num_episodes;

    double avg_last_cycles_err;
    double min_last_cycles_err;
    double max_last_cycles_err;

  } accPerf; 

};

#endif

