/*
clsquare - closed loop simulation system
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

#ifndef _reward_h_
#define _reward_h_

#include "global.h"
#include "setdef.h"

/** @defgroup REWARD Reward Modules
  * These modules implement the evaluation of system states for learning. */

enum { 
  REWARD_INPUT_UNSPECIFIED = -1,
  REWARD_INPUT_PLANT_STATE = 0,
  REWARD_INPUT_MEASUREMENT,
  REWARD_INPUT_OBSERVED_STATE, 
};

/** Base class for all reward modules in CLSquare. There are three options for how to 
 determine the rewards. The standard method is determing the rewards based on the
 measurements of the plant's state. Alternatively, rewards can also be calculated using
 either the internal plant states or the observed states calculated by the observer.
 A reward module has to chose one of these options and must only implement the 
 corresponding set of methods. */
class Reward { 
public:
  
  virtual bool init(int plant_state_dim, int measurement_dim, int observed_state_dim, int action_dim, int* expected_input_type, const char* fname);
  
  /** returns the immediate reward for a transition from the current state to the next state
   using the specified action. This method returns the reward based on the measurements. */
  virtual double get_reward(const double *current_state_representation, const double *current_action, const double *next_state_representation) = 0;
  virtual bool is_terminal(const double *state_representation) { return false; }
  virtual double get_terminal_reward(const double *state_representation) { return 0; }

  virtual void deinit(){;}
  
  virtual ~Reward() {}
  
protected:
  Reward() {}
};

#ifdef CLSQUARE
#include "registry.h"
#else
#define REGISTER_REWARD(classname, desc)
#endif

#endif
