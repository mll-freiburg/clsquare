/*
 clsquare - closed loop simulation system
 Authors: Sascha Lange
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

#include "standardreward.h"
#include "global.h"
#include "valueparser.h"
#include <cstring>


double StandardReward::get_reward(const double *current_state_representation, const double *current_action, const double *next_state_representation) 
{
  double r = reward_step;
  if (is_terminal(current_state_representation)) {
    r = 0.; // steps starting within a terminal state are free (but there may occur terminal costs)
  }
  else if (!xplus_is_terminal && xplus.isWithinSet(next_state_representation, representation_dim)) { // within the xplus set, that is not terminal
    r = reward_xplus;
  }
  else if (!xminus_is_terminal && xminus.isWithinSet(next_state_representation, representation_dim)) {
    r = reward_xminus;
  }
  return r;
}

bool StandardReward::is_terminal(const double *state_representation)
{
  return 
    (xplus_is_terminal && xplus.isWithinSet(state_representation, representation_dim)) ||
    (xminus_is_terminal && xminus.isWithinSet(state_representation, representation_dim));
}

double StandardReward::get_terminal_reward(const double *state_representation) {
  double tr = 0.;
  if (xplus_is_terminal && xplus.isWithinSet(state_representation, representation_dim)) {
    tr = reward_xplus;
  }
  else if (xminus_is_terminal && xminus.isWithinSet(state_representation, representation_dim)) {
    tr = reward_xminus;
  }  
  return tr;
}




#ifdef XCODE
#pragma mark -
#pragma mark Initialization and Options
#endif


bool StandardReward::init(int plant_state_dim, int measurement_dim, int observed_state_dim, int action_dim, int* expected_input_type, const char* fname)
{ 
  this->plant_state_dim = plant_state_dim;
  this->observed_state_dim = observed_state_dim;
  this->measurement_dim = measurement_dim;
  
  if (!read_options(fname)) {
    EOUT("Error reading options of StandardReward module.");
    return false;
  }
  
  *expected_input_type = input_type;
  
  return true;
}


bool StandardReward::read_options(const char * fname) 
{
  char paramstr[MAX_STR_LEN+1];
  
  if(fname == 0)
    return false;
  
  ValueParser vp(fname,"Reward");
  if (vp.get("input_type", paramstr, MAX_STR_LEN) >= 0) {
    if(strcmp(paramstr, "measurement") == 0) {
      input_type = REWARD_INPUT_MEASUREMENT;
    }
    else if(strcmp(paramstr, "plant_state") == 0) {
      input_type = REWARD_INPUT_PLANT_STATE;
    }    
    else if(strcmp(paramstr, "observed_state") == 0) {
      input_type = REWARD_INPUT_OBSERVED_STATE;
    }
    else {
      EOUT ("Param: input_type : invalid value " << paramstr);
      return false;
    }
  }
  else {
    input_type = REWARD_INPUT_MEASUREMENT;
  }
  
  if (input_type == REWARD_INPUT_PLANT_STATE) {
    representation_dim = plant_state_dim;
  }
  else if (input_type == REWARD_INPUT_OBSERVED_STATE) {
    representation_dim = observed_state_dim;
  }
  else {
    representation_dim = measurement_dim;
  }
  
  if(vp.get("xplus",paramstr,MAX_STR_LEN)>=0){
    xplus.parseStr(paramstr, representation_dim);
  }
  if(vp.get("xminus",paramstr,MAX_STR_LEN)>=0){
    xminus.parseStr(paramstr, representation_dim);
  }
  if(vp.get("xwork",paramstr,MAX_STR_LEN)>=0){
    xwork.parseStr(paramstr, representation_dim);
  }
  vp.get("xplus_is_terminal", xplus_is_terminal, true); 
  vp.get("xminus_is_terminal", xminus_is_terminal, true); 
  
  vp.get("reward_step", reward_step, -1.);
  vp.get("reward_xplus", reward_xplus, 0.);
  vp.get("reward_xminus", reward_xminus, -1000.);
  
  return true;
}


REGISTER_REWARD(StandardReward, "Standard Reward module. If no parameters given, always returns -1.");

