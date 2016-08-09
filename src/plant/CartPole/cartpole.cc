/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

cartpole.c is based on a previous implementation by Ralf Schoknecht and Artur Merke

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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include "valueparser.h"
#include "cartpole.h"
#include "cartpole_dynamics.h"
#include "random.h"
#include "global.h"

#define STATE_DIM 4
#define ACTION_DIM 1
#define DELTA_T 0.02
#define MAX_LINE_LEN 500

#define MC 1.0  // mass cart
#define MP 0.1 // mass pole
   //#define LP 0.225 // half length pole
#define LP 0.5 // half length pole

#define ANGLE_MAX  5000
#define POSITION_MAX  2.4
#define ANGULAR_VEL_MAX 100.
#define CART_VEL_MAX 100.0
#define FRC   0.0  
#define FRP   0.0 

/*****************************************************************
 * Cart Pole
 *****************************************************************/
bool CartPole::get_next_plant_state(const double *state, const double *action, double *next_state){
  double tmp_action[1];

  tmp_action[0] = action[0];
  // restrict action
  if (tmp_action[0] > u_max )
    tmp_action[0] = u_max;

  if ( tmp_action[0] < - u_max )
    tmp_action[0] = - u_max;


  // compute next state
  cartpole_next_state(state,tmp_action,next_state);
  
  // noise:
  const double noise_scale[4] = {.5, 2, 2, 2};

  if(noise_level > 0){ // simulate noisy plant
    for(int i= 0; i<STATE_DIM; i++){
      next_state[i] += Util::gaussian(0, noise_level * noise_scale[i]);
    }
  }

  if(fabs(next_state[0]) > angle_max)
    return false;

  if(fabs(next_state[2]) > position_max)
    return false;

  return true;
}

void CartPole::deinit(){
}

bool CartPole::init(int& plant_state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter){
  delta_t = DELTA_T;
  massp = MP;
  massc = MC;
  lp = LP;
  fricc = FRC;
  fricp = FRP;
  u_max = 1E6; // unlimited
  position_max = POSITION_MAX;
  angle_max = ANGLE_MAX;
  //angular_vel_max = ANGULAR_VEL_MAX;
  //cart_vel_max = CART_VEL_MAX; 
  integration_steps = 10;
  noise_level = 0;
  read_options(fname,chapter);
  cartpole_init(integration_steps, delta_t, massc, massp, lp, fricc, fricp);

  // return values:
  plant_state_dim = STATE_DIM;         
  measurement_dim = plant_state_dim; // default: measurement is equal to state
  action_dim = ACTION_DIM; 
  return true;
}

bool CartPole::read_options(const char * fname, const char * chapter) 
{
//  char paramstr[MAX_STR_LEN];

  if(fname == 0)
    return true;

  ValueParser vp(fname,chapter==0?"Plant":chapter);

#if 0
  // Task Specification
  if (vp.get("task",paramstr,MAX_STR_LEN) >= 0) {
    if (strcmp(paramstr,"SwingUp") == 0)
      task_mode = false;
    else if (strcmp(paramstr,"Balancing")== 0)
      task_mode = true;    
    else {
       EOUT("Unknown task : (" << paramstr << ") using task Balancing");
       task_mode = true;
    }
  }
#endif

  // Read integration_method and no_of_integration_steps
  vp.get("no_of_integration_steps",integration_steps);

  // Read CartPole specific options
  vp.get("mass_cart",massc);
  vp.get("mass_pole",massp);
  vp.get("length_pole",lp);
  vp.get("friction_cart",fricc);
  vp.get("friction_pole",fricp);
  vp.get("angle_max",angle_max);
  //vp.get("angle_vel_max",angular_vel_max);
  vp.get("position_max",position_max);
  //vp.get("cart_vel_max",cart_vel_max); 
  vp.get("delta_t",delta_t);
  vp.get("u_max",u_max);
  vp.get("noise_level",noise_level);

  // Show error-message if necessary
  if ( vp.num_of_not_accessed_entries() ) {
    cerr << "\nCartPole: not recognized options:";
    vp.show_not_accessed_entries(cerr);
    cerr << endl;
    return false;
  }
  return true;
}


REGISTER_PLANT(CartPole, "Simulation of a pole balancing cart.")

