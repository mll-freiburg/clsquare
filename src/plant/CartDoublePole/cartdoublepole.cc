/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck


All rights reserved.

The plant realizes a double inverted pendulum mounted on a cart.

The default paramters of the plant are take from 
A. Bodganov: 'Optimal Control of a Double Inverted Pendulum on a Cart'
OGI School of Engineering, OHSU
(TR, CSE 04 006)

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
#include "valueparser.h"
#include "cartdoublepole.h"
#include "cartdoublepole_dynamics.h"


#define STATE_DIM 6
#define ACTION_DIM 1
#define DELTA_T 0.02


#define ANGLE1_MAX  3.14/2.
#define ANGLE2_MAX  3.14/2.
#define POSITION_MAX  2.4
#define ANGULAR_VEL_MAX 100.
#define CART_VEL_MAX 100.0
#define FRC   0.0  
#define FRP   0.0 
#define G 9.81

#define THETA1  0   /* angle */
#define THETA1P 1   /* angular velocity */
#define THETA2  2   /* angle */
#define THETA2P 3   /* angular velocity */
#define S       4   /* distance */
#define SP      5   /* cart velocity */


bool CartDoublePole::get_next_plant_state(const double *state, const double *action, double *next_state)
{  
  // compute next state
  double spp, theta1pp, theta2pp;
   
  for(int i=0;i<STATE_DIM;i++) 
    next_state[i] = state[i];       

  const double hl_1 =L1/2.0;  // half length of pole1
  const double hl_2 =L2/2.0; // half length of pole2

  for(int t=0;t<integration_steps;t++){
    compute_LGS(&spp, &theta1pp, &theta2pp, Mc, M1, M2, hl_1, hl_2, action[0], next_state);
    /* update internal state: */
    double euler_time = delta_t / (double) integration_steps;
   
    next_state[THETA1]  += euler_time * next_state[THETA1P];
    next_state[THETA2]  += euler_time * next_state[THETA2P];
    next_state[THETA1P] += euler_time * theta1pp;
    next_state[THETA2P] += euler_time * theta2pp;
    next_state[S]       += euler_time * next_state[SP];
    next_state[SP]      += euler_time * spp;
  }
  
  // Check, if system state is in working area
  if (fabs(next_state[THETA1]) > angle1_max )  {
    return false; 
  }
  
  if (fabs(next_state[THETA1P]) > angular_vel_max ) {
    return false;
  } 
    
  if (fabs(next_state[THETA2]) > angle2_max ) {
    return false;  
  }
   
  if (fabs(next_state[THETA2P]) > angular_vel_max )  {
    return false; 
  }
  
  if(fabs(next_state[S]) > position_max) {
    return false;
  }
   
  if(fabs(next_state[SP]) > cart_vel_max) {
    return false;
  }
 
  return true;
}

bool CartDoublePole::init(int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  this->delta_t = delta_t = DELTA_T;
  integration_steps = 5;

  /* default paramters for the plant (see TR Bogdanov, 2004) */
  Mc =1.5;  // mass of the cart
  M1 = 0.5; // mass of pole 1 (lower)
  M2 =0.75;  // mass of pole 2 (upper)
  L1 =0.5; // full length of pole 1
  L2 = 0.75; // full length of pole 2

  position_max = POSITION_MAX;
  angle1_max = ANGLE1_MAX;
  angle2_max = ANGLE2_MAX;
  angular_vel_max = ANGULAR_VEL_MAX;
  cart_vel_max = CART_VEL_MAX;  
  read_options(fname,chapter);

  if (integration_steps < 1) {
    EOUT("integration_steps must be a positive number (currently " << integration_steps << ")");
    return false;
  }
  if (delta_t <= 0.0) {
    EOUT("delta_t must be a positive number (currently " << delta_t << ")");
    return false;
  }

  // return values:
  state_dim = STATE_DIM;         
  measurement_dim = state_dim; // default: observed_state is equal to state
  action_dim = ACTION_DIM;
  return true;
}

bool CartDoublePole::read_options(const char * fname, const char * chapter) 
{
  if(fname == 0)
    return true;

  ValueParser vp(fname,chapter==NULL?"Plant":chapter);

  // Read CartDoublePole specific options
  vp.get("mass_cart",Mc);
  vp.get("mass_1",M1);
  vp.get("length_1",L1);
  vp.get("mass_2",M2);
  vp.get("length_2",L2);
  vp.get("angle1_max",angle1_max);
  vp.get("angle2_max",angle2_max);
  vp.get("angle_vel_max",angular_vel_max);
  vp.get("position_max",position_max);
  vp.get("cart_vel_max",cart_vel_max); 
  vp.get("delta_t",delta_t);
  vp.get("integration_steps",integration_steps);

  // Show error-message if necessary
  if ( vp.num_of_not_accessed_entries() ) {
    cerr << "\nCartDoublePole: not recognized options:";
    vp.show_not_accessed_entries(cerr);
    cerr << endl;
    return false;
  }
  return true;
}



REGISTER_PLANT(CartDoublePole, "Simulation of a double pole balancing cart. Thanks to Bernd Weisser for providing the dynamics.")

