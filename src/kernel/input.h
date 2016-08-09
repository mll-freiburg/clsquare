/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Roland Hafner

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

#ifndef _INPUT_H_
#define _INPUT_H_

#include "global.h"
#include "registry.h"

/** @defgroup INPUT Input Modules
  * The input module defines the initial plant state of an episode at time step 0, thus caring for the initial and external conditions of an episode.
  * \todo split generation of initial state and reference input into separate modules? */
class Input{
 public:
  
  /** Get the initial state of a episode, interface for mainloop.
    * This function is called at the beginning of every episode.
    * \attention the initial plant state can be rejected or changed by the plant and the controller, see PLant and Controller documentation for details.
    * \param initial_plant_state initial state of episode
  */
  virtual void get_initial_episode_plant_state(double* initial_plant_state, const int episode) = 0;

  /** Get the external signal (vector) for recent time step, interface for mainloop.
    * This function is called every time step.
    * See the CLSquare Howto for usage of reference states (if you don't know that you need them, you probably won't need them ;-) ).
    * \param external_signal the external signal vector to set values in
    * \param cycle_ctr the number of time steps in the recent episode (use cycle_ctr * delta_t to get time in seconds since episode started)
  */
  virtual void get_external_signal(double* external_signal, long cycle_ctr) = 0;

  /** Initializes input module, interface for mainloop.
   * \param x_dim dimension of state space
   * \param u_dim dimension of action space
   * \param delta_t duration of a control cycle (in seconds)
   * \param fname file which contains configuration of input module 
   * \return true for success */
  virtual bool init(const int _plant_state_dim, const int _action_dim, const int _external_signal_dim, const double _delta_t, const char *fname=0) {return true;};

  /** Deinit input module, interface for mainloop.
    * \return true on success.
    */
  virtual void deinit() {};

};

#ifdef CLSQUARE
#include "registry.h"
#else
#define REGISTER_INPUT(classname, desc)
#endif

#endif

