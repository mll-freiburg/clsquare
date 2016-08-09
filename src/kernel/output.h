/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Martin Riedmiller, Roland Hafner

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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/

#ifndef _OUTPUT_H_
#define _OUTPUT_H_

#include "global.h"
#include "registry.h"
#include "kerneldatatypes.h"

/** @defgroup OUTPUT Output Module
  * The output module logs the information of all episodes and cycles. */
class Output
{
public:
  virtual ~Output(){};
  /** Notification of the current information of the system.
    * This is an interace to the CLSquare main loop.
    * The function is called for every transition of the system.
    * Current referes to the starting state of the transition and the chosen action in this state.
    * There will be a subsequent state for which this function will be called, if the episode will proceed,
    * or the notify_episode_stops will be called if that one is the last state of this episode.
    * @note  This function will not be called for a sequence with only one state, without transitions.
    * @param current_state_vars The information abut the system state variables (plant_state, measurement, observation, external_signal).
    * @param current_action The action chosen in this state.
    * @param current_sys_time The system time information for current.
    * @param transition_rating The rating signal (direct costs) for this transition.
    **/
  virtual void notify       ( const sys_state_variables&     current_state_vars,
                              const double*                  current_action,
                              const sys_time_struct&         current_sys_time,
                              double                         transition_rating) {};

  /** Notification of the current information of the system at the end of an episode.
       * This is an interace to the CLSquare main loop.
       * The function is called once for every episode, with the information refering to the last state in the episode.
       * @note  This function will also be called for a sequence with only one state, without transitions.
       * @param current_state_vars The information abut the system state variables (plant_state, measurement, observation, external_signal).
       * @param current_sys_time The system time information for the last state in the episode.
       * @param terminal True if the last state of the episode is an terminal state.
       * @param terminal_rating If the state is an terminal state the according terminal rating is placed here.
       *        @note If terminal is false this parameter has no meaning.
       **/
  virtual void notify_episode_stops ( const sys_state_variables&     current_state_vars,
                                      const sys_time_struct&         current_sys_time,
                                      bool                           terminal,
                                      double                         terminal_rating) {};

  /** Notification of an episode start.
    * This is an interace to the CLSquare main loop.
    * The function is called for every episode.
    * @param  sys_time_info the time information at the beginning of the episode.
    *         The most important entry is the number of the episode in sys_time_struct::episode_ctr.
    *         Other information for reference purpose.
    **/
  virtual void notify_episode_starts(const sys_time_struct& sys_time_info) {};

  /** Initialization of output module.
    * This is an interace to the CLSquare main loop.
    * The function is called once when the program starts up and the module was created.
    * @param  sys_dim_info information about the dimensions of state and action variables (vectors)
    * @param  sys_time_info the time information at initialization time.
    *         The most important entry is the sys_dim_struct delta_t.
    *         The other values are initialized to the first control cycle in the first episode.
    *         From these the starting values of episode/cycle counters are communicated.
    * @return true on success, false on errors.
    *         A return value of false will quit the init phase of CLSquare and the program with an error message.
    **/
  virtual bool init(const sys_dim_struct& sys_dim_info , const sys_time_struct& sys_time_info, const char *fname=0) {return true;};

  /** Deinit the output module.
    * This is an interface to the CLSquare main loop.
    * The function is called once when the program finishes.
    **/
  virtual void deinit() {}; //terminate
};

#endif

