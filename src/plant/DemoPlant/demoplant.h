/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck
Copyright (c) 2011, Machine Learning Lab, Prof. Dr. Martin Riedmiller,
University of Freiburg

demoplant.cc: simple demo plant for demo purposes

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
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/



#ifndef _DEMO_PLANT_H_
#define _DEMO_PLANT_H_

#include "plant.h"

/** A plant for demonstration purpose.
  *
  * @ingroup PLANT
  * @ingroup SIMULATION
  **/
class DemoPlant : public Plant {
 public:

  /** Computes the next state of the plant, given a current state and an action (state transition).  
   * \param current_plant_state current state of plant.
   * \param current_action executed action in current state.
   * \param next_plant_state resulting state after executing action.
   * \return true to continue, false, if break of episode is requested */
  virtual bool get_next_plant_state(const double *current_plant_state, const double *current_action, double *next_plant_state);

  /** Computes a measurement of the plant_state. 
   * \param plant_state     current state
   * \param observed_state  write observation of current state in this array
   * \return true for success */
  virtual bool get_measurement(const double *plant_state, double *measurement);

  /** Checks, if plant agrees on initial state. If not, the plant module can override the initial state
   *  or reject the initial state by returning false. 
   * \param initial_state initial state
   * \param target_state target state 
   * \returns true, if plant agrees on initial state or overrides the initial state */
  virtual bool check_initial_state(double *initial_plant_state);

  /** Initialize plant.
   * Implement one of the following virtual bool init(...) functions for your plant.
   * For convenience you can use the function that meets your requirements most.
   * Other params are set by default as defined below.
   * \param plant_state_dim plant returns dimension of state vector for plant state.
   * \param measurenebt_dim plant returns dimension of observation vector for controller.
   * \param action_dim plant returns dimension of action space. 
   * \param delta_t plant returns duration of one control cycle in seconds.
   * \param fname File, which contains configuration of plant module
   * \return true, for success. */
  virtual bool init(int &plant_state_dim, int &measurement_dim, int &action_dim, 
		    double &delta_t, const char *fname=0, const char *chapter=0);

  /** Notifies that an episode has been started. */
  virtual void notify_episode_starts();
  
  /** Notifies that an episode has been stopped. */
  virtual void notify_episode_stops();

  /** virtual destructor is necessary since methods declared virtual */
  virtual ~DemoPlant() {}

  /** Notifies that a command via pipe has arrived. */
  //  virtual void notfiy_command_string(char* buf){return;};
  virtual void notify_command_string(const char* buf);

 protected:
  int verbosity; /** verbosity of demo plant */
  virtual bool read_options(const char *fname, const char *chapter);


};

#endif
