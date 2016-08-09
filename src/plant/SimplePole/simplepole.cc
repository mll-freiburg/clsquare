/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

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

#include <math.h>
#include <stdio.h>
#include <iostream>
#include "valueparser.h"
#include "simplepole.h"
#include "simplepole_dynamics.h"
#include "global.h"

#define STATE_DIM 2
#define ACTION_DIM 1
#define DELTA_T 0.1
#define MAX_LINE_LEN 500

#define MP 1.0 // mass pole
#define LP 1   // half length pole
#define MU 0.05

/*****************************************************************
 * Cart SimplePole
 *****************************************************************/

bool SimplePole::get_next_plant_state(const double *state, const double *action, double *next_state)
{
  //  simplepole_next_state(state,action,next_state);
  simplepole_next_state_discrete_time(state,action,next_state);
  return true;
}

bool SimplePole::init(int& state_dim, int& measurement_dim, int& action_dim, double& _delta_t, const char *fname, const char *chapter)
{
  delta_t = DELTA_T;
  massp = MP;
  lp = LP;
  mu = MU;

  read_options(fname, chapter);
  simplepole_init(delta_t, massp, lp,mu);

  _delta_t = delta_t;
  // return values:
  state_dim = STATE_DIM;
  measurement_dim = state_dim; // default: observed_state is equal to state
  action_dim = ACTION_DIM; 

  return true;
}

void SimplePole::read_options(const char * fname, const char *chapter) 
{
  if(fname == 0)
    return;

  ValueParser vp(fname,chapter==0?"Plant":chapter);
  // Read SimplePole specific options
  vp.get("mass_pole",massp);
  vp.get("length_pole",lp);
  vp.get("delta_t",delta_t);

  // Show error-message if necessary
  if ( vp.num_of_not_accessed_entries() ) {
    cerr << "\nSimplePole: not recognized options:";
    vp.show_not_accessed_entries(cerr);
    cerr << endl;   
  }

}


REGISTER_PLANT(SimplePole, "Simulation of a simple pole. Used for NFQ ECML 2005 paper.")

