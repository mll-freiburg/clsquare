/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck
Copyright (c) 2011, Machine Learning Lab, Prof. Dr. Martin Riedmiller,
University of Freiburg

All rights reserved.

Author: Martin Riedmiller

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

#ifndef _DEMOCONTROLLER_H_
#define _DEMOCONTROLLER_H_

#include "controller.h"

/** A simple demo controller.
  * @author Martin Riedmiller
  * @ingroup CONTROLLER
  * @ingroup STATIC
  */

class DemoController : public Controller {
  virtual ~DemoController() {;}

  /** Computes an action, given an observation of a state.
   * \attention if the plant decides to end the recent episode, the action will not get active
   * and the state could be a final/terminal state.
   * \param observed_state current observation of state
   * \param action the computed action to be execute in the current state.
   * \return ControllerResult */
  virtual bool get_action(const double* observed_state, double* action);

  /** Initialize controller
   * \param observed_state_dim Dimension of observation space
   * \param action_dim Dimension of action space. 
   * \param deltat Duration of one control cycle in seconds.
   * \param fname File, which contains configuration 
   * \param chapter Chapter in which the configuration is located
   * \return true, for success. */
  virtual bool init(const int observed_state_dim, const int action_dim, double deltat, 
		    const char* fname=0, const char* chapter=0);

  /** Terminating control. */
  virtual void deinit(){return;}

  /** Notifies that a episode has started. */
  virtual void notify_episode_starts();

  /** Notifies that a episode has stopped. */
  virtual void notify_episode_stops(const double* current_observed_state);

  /** A function for convenience of RL conrollers
    * \param reward reward/ costs for transiton
    * \param is_terminal_state information if transition ends in terminal state
    * \param terminal_reward information about terminal_reward (only valid, if terminal state)
    */
  
  virtual void notify_transition(const double* observed_state,
                                 const double* action,
                                 const double* next_observed_state,
                                 const double reward, const bool is_terminal_state, 
				 const double terminal_reward);

  /** Checks if controller module agrees on initial state and observation of current trial.
   * If accepted true is returned.
   * If not, the module can overrides the initial state and return true, or reject the initial state or observation by returning false. 
   * Note: obsevations can't be changed. 
   * For convenience you can implement only one of the last two functions or the first or no function if you don't want to check. 
   *
   * \param initial_plant_state initial state
   * \param initial_observed_state observation of initial state
   * \param target_state target state
   * \return true if controller agrees on initial state */
  virtual bool check_initial_state(const double* initial_observed_state, const int observation_dim);

 protected:
  bool read_options(const char* fname, const char* chapter);
  int verbosity;
  double max_action;

};
#endif
