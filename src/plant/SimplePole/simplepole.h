/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck
Copyright (c) 2011, Machine Learning Lab, Prof. Dr. Martin Riedmiller,
University of Freiburg



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

#ifndef _SIMPLEPOLE_PLANT_H_
#define _SIMPLEPOLE_PLANT_H_

#include "plant.h"

/**
 * Realizes a plant module for a stationary simple pole system. 
 * This system has a two-dimensional state: 
 * \li angle of pole
 * \li angular velocity of pole
 *
 * It accepts a one-dimensional action:
 * \li force to apply to the pole
 *
 * The system dynamics are taken from: "An Approach to Fuzzy Control of Nonlinear Systems: Stability and Design 
 * Issues", H.O.Wang and K.Tanaka, IEEE Transactions on Fuzzy Systems, Vol. 4 No.1, 1996.
 * Uniform noise is added to all actions (see options for pole2 plant).
 * @ingroup PLANT
 * @ingroup SIMULATION
 */
class SimplePole : public Plant {
 public:
  /** Computes the next state of the environment, given a current state and an action (state transition).  
   * \param state current state of plant
   * \param action executed action in current state
   * \param reference_input an external input for non controllable variables like reference values
   * \param next_state resulting state after executing action
   * \param reward reward for state transition  
   * \returns true, if next state is not a terminal state. */   
  virtual bool get_next_plant_state(const double *state, const double *action, double *next_state);
 
  /** Initializes Pole.
   * \param state_dim plant returns dimension of state space (state vector)
   * \param observation_dim plant returns dimension of observation space (observation vector) 
   * \param action_dim plant returns dimension of action space.   
   * \param delta_t plant returns duration of one control cycle [s]
   * \param fname file, which contains configuration of plant module
   * \return true, for success. */ 
  virtual bool init(int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 
  

 protected:
  /** Mass of pole. */
  double massp;

  double mu;

  /** Length of pole. */
  double lp;
 
  /** Duration of one control cycle in seconds. */
  double delta_t;
  
  /** Reads configuration for acrobot plant. 
   * \param fname filename
   * \return true for success */
  void read_options(const char * fname, const char * chapter);
};

#endif

