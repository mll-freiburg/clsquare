/*
clsquare - closed loop simulation system
Author: Sascha Lange
Copyright (c) 2011, Machine Learning Lab, University of Freiburg

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

#ifndef _standardreward_h_
#define _standardreward_h_

#include "global.h"
#include "reward.h"


// Offene Fragen: Wie umgehen, wenn Startzustand im Terminalbereich ist? Logik von Roland übernehmen?

/** Stub for the standard-rewards module.
  * @ingroup REWARD */
class StandardReward : public Reward 
{ 
public:
  
  StandardReward() : Reward() {}
  
  virtual bool init(int plant_state_dim, int measurement_dim, int observed_state_dim, int action_dim, int* expected_input_type, const char* fname);
  
  virtual double get_reward(const double *current_state_representation, const double *current_action, const double *next_state_representation);
  virtual bool   is_terminal(const double *state_representation);
  virtual double get_terminal_reward(const double *state_representation);
  
  virtual ~StandardReward() {}
  
protected:
  bool read_options(const char * fname);

  int input_type;
  int representation_dim;
  
  SetDef xplus;
  SetDef xminus;
  SetDef xwork;
  
  bool xplus_is_terminal;
  bool xminus_is_terminal;
  
  double reward_step;
  double reward_xplus;
  double reward_xminus;
  
  int measurement_dim;
  int plant_state_dim;
  int observed_state_dim;
};

#ifdef CLSQUARE
#include "registry.h"
#else
#define REGISTER_REWARD(classname, desc)
#endif

#endif
