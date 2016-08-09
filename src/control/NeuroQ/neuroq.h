/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

QoffController
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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/

#ifndef _NEUROQ_H_
#define _NEUROQ_H_

#include "n++.h"
#include "controller.h"
#include "aset.h"
#include "setdef.h"

#define MAX_NUM_ACTIONS 50
#define MAX_NUM_SUBSETS 10

/** A neural controller that uses a feed-forward neural network
  * saved in n++ format to encode a Q-function.
  * The location of the file in which the network is described is
  * given by the parameter \b Qnet.
  *
  * The plant aborts an episode if it reaches the state region
  * described by \b goal_area or \b avoid_area or if it leaves
  * the region \b working_area.
  *
  * The available actions for each dimension are specified by
  * the SetDef parameter \b actions.
  *
  * Several parameters can be used to adjust the plant:
  * \li \b multi_step_param (int): number of cycles a chosen
  *     action is to be maintained
  * \li \b exploration (float): percentage with which to
  *     choose a random action in each cycle
  * \li \b separate_actionnets (bool): use a separate network
  *     for each action
  *
  * @author Martin Riedmiller
  * @ingroup CONTROLLER
  * @ingroup STATIC
  */
class NeuroQ  : public Controller {
public:
 /** Choose an action in a certain state with respect to the learned neural network.   
   * \param observation observation of current state
   * \param action action to execute in current state.
   * \return true, for success. */ 
  virtual bool get_action(const double* observation, double* action); 

  /** Initializes network
   * \param observation_dim dimension of observation space
   * \param action_dim dimension of action space. 
   * \param deltat duration of one control cycle [s]
   * \param fname file, which contains configuration of controller
   * \param chapter chapter in which the configuration is located
   * \return true, for success. */
  virtual bool init(const int observation_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  
  /** Deinit network. */
  virtual void deinit(); 
   
  /** Notifies that an episode has stopped. 
   * \param final_observation observation of final state
   * \param final_reward reward for terminal state
   * \param is_terminal_state true, if final state is a terminal state */
  virtual void notify_episode_stops(const double* final_observation);
 
  virtual ~NeuroQ(){ };

 protected:
  struct{
    double exploration;
    bool seperate_actionnets;
    bool reload_controlnet;
  } params;

  SetDef goal_area, avoid_area, working_area;

  int state_dim,action_dim;
  double delta_t;
  double *r;
  double *multi_step_action;
  int multi_step_ctr, multi_step_param;

  Aset aset;

  Net qnet;
  Net *action_net;
  bool do_random;
  bool stop_in_xplus;
  int *reduced_statevec;
  int control_state_dim;
  double *offset_action;

  // procedures depending on function approximator
  double get_Q(const double* state, const double* action);
  double get_Q(const double* state, const int action_idx);

  // procedures for general Q-learning
  bool in_avoid(const double* state);
  bool in_goal(const double* state);
  double get_greedy_action(const double* state, double* action); 
  // IO procedures
  bool read_options(const char *fname, const char *chapter); 
};

#endif
