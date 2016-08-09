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

#ifndef _STATISTICS_H_
#define _STATISTICS_H_

#include "global.h"
#include "registry.h"

/** @defgroup STATISTICS Statistic Module
  * Module for computing a statistic of episodes.
  */

class Statistics {
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
   */
    virtual void notify(const double* plant_state, const double* measurement,
                        const double* observation, const double* action,
                        const long cycle, const long episode,
                        const float total_time, const float episode_time, const long total_num_of_cycles, double transition_rating_signal)=0;

  /** Initializes statistics module.
   * \param plant_state_dim Dimension of plant state
   * \param measurement_dim Dimension of measured state
   * \param observation_dim Dimension of observed state
   * \param act_dim Dimension of action space.
   * \param delta_t Duration of one control cycle [s]
   * \param fname File, which contains conifiguration
   * \return true, for success. */ 
    virtual bool init(int plant_state_dim, int measurement_dim, int observation_dim, int action_dim,
                      double delta_t, long cycles_per_episode, const char *fname=0)=0;

  /** End statistics. */
  virtual void deinit()=0;

  /** Notifies, that a trial has started. 
   * Calls the method to initialize the statistics of current episode. 
   * \param episode_ctr Current episode. */
  virtual void notify_episode_starts(const long episode_ctr)=0;

  /** Notifies, that a trial has stopped. 
   * Collect all remaining statistical information of current episode. 
   * \param plant_state Current plant state. 
   * \param measurement Current measurement. 
   * \param observation Current observed state. 
   * \param episode_ctr Current trial. */   
  virtual void notify_episode_stops(const double* plant_state, const double* measurement, const double* observation,
                             const long cycle, const long episode,
                             const float total_time, const float episode_time, const long total_num_of_cycles, bool is_terminal, double terminal_rating_signal)=0;

  virtual ~Statistics(){;}
};


class NOOPStatistics : public Statistics {
public:
    virtual void notify(const double* plant_state, const double* measurement, 
                        const double* observation, const double* action,
                        const long cycle, const long episode,
                        const float total_time, const float episode_time, const long total_num_of_cycles, double transition_rating_signal) {;}

    virtual bool init(int plant_state_dim, int measurement_dim, int observation_dim, int action_dim,
                      double delta_t, long cycles_per_episode, const char *fname=0) {return true;}

    virtual void deinit() {;}

    virtual void notify_episode_starts(const long episode_ctr){;}

    virtual void notify_episode_stops(const double* plant_state, const double* measurement, const double* observation,
                                       const long cycle, const long episode,
                                       const float total_time, const float episode_time, const long total_num_of_cycles, bool is_terminal, double terminal_rating_signal){;}

    virtual ~NOOPStatistics(){;}
};


#endif

