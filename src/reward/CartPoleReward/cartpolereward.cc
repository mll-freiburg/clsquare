/*
 clsquare - closed loop simulation system
 Authors: Martin Riedmiller
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

#include "cartpolereward.h"
#include "global.h"
#include "valueparser.h"
#include <cstring>
#include <math.h>



double CartPoleReward::get_reward(const double *current_state_representation, const double *current_action, const double *next_state_representation) 
{
  double costs_for_action = 0;

  if(quadratic_costs == true){
    for(int i= 0; i< u_dim; i++)
      costs_for_action += current_action[i] * Ucostdiag[i] * current_action[i];
  }
  else{ // linear costs
    for(int i= 0; i< u_dim; i++)
      costs_for_action += fabs(current_action[i]) * Ucostdiag[i];    
  }
  
  //cout <<"costs 4 action: "<<costs_for_action<<endl;

  return costs_for_action;
}

double CartPoleReward::get_terminal_reward(const double *state_representation) {
  double dx = state_representation[2];

  return (fabs(dx));
}




#ifdef XCODE
#pragma mark -
#pragma mark Initialization and Options
#endif


bool CartPoleReward::init(int plant_state_dim, int measurement_dim, int observed_state_dim, int action_dim, int* expected_input_type, const char* fname)
{ 
  //  this->measurement_dim = measurement_dim;
 
  u_dim = action_dim;
  
  Ucostdiag = new double[u_dim];
  for(int i= 0; i< u_dim; i++)
    Ucostdiag[i] = 0;

  quadratic_costs = false;
 
  if (!read_options(fname)) {
    EOUT("Error reading options of CartPoleReward module.");
    return false;
  }


  return true;
}


bool CartPoleReward::read_options(const char * fname) 
{
  //char paramstr[MAX_STR_LEN+1];
  
  if(fname == 0)
    return false;
  
  ValueParser vp(fname,"Reward");
  
  //int tmpnum = 
  vp.get("Udiag",Ucostdiag,u_dim);
  //if(ruessel3D == true && tmpnum <9){
  //  WOUT(10, "reward modules costs for action are not completely specified.");
  //}
  vp.get("quadratic_costs",quadratic_costs);


  return true;
}


REGISTER_REWARD(CartPoleReward, "CartPole Reward module. If no parameters given, always returns -1.");

