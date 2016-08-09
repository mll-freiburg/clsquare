/*
clsquare - closed loop simulation system
Copyright (c) 2012, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include "valueparser.h"
#include "parallelpole.h"
#include "dynamics_gomez.h"
#include "dynamics_original.h"
#include "random.h"
#include "global.h"

/*****************************************************************
 * Cart Pole
 *****************************************************************/
bool CartParallelPole::get_next_plant_state(const double *state, const double *action, double *next_state)
{
  // restrict action
  double tmp_action[1];
  tmp_action[0] = action[0];
  if (tmp_action[0] > u_max )
    tmp_action[0] = u_max;
  if ( tmp_action[0] < - u_max )
    tmp_action[0] = - u_max;

  // compute next state
  pole->next_state(state, tmp_action, next_state);

  // simulate noisy plant
  double noise_scale[_sdim];
  for (int i=0; i<_sdim; i++)
  noise_scale[0] = noise_scale[2] = 0.5;
  noise_scale[1] = noise_scale[3] = noise_scale[_sdim-2] = noise_scale[_sdim-1] = 2;
  if (noise_level > 0)
    for (int i= 0; i<_sdim; i++)
      next_state[i] += Util::gaussian(0, noise_level * noise_scale[i]);

  if (fabs(next_state[0]) > angle_max1)
    return false;
  if (_sdim > 4 && fabs(next_state[2]) > angle_max2)
    return false;
  if (fabs(next_state[_sdim>4?4:2]) > position_max)
    return false;
  return true;
}

bool CartParallelPole::init(int& plant_state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  char paramstr[MAX_STR_LEN];
  ValueParser vp(fname,chapter==0?"Plant":chapter);

  // number of poles
  int poles;
  vp.get("poles",poles,2);
  if (poles < 1 || poles > 2) {
    EOUT("Invalid number of poles.");
    return false;
  }

  // Read integration_method and no_of_integration_steps
  int integration_steps;
  vp.get("no_of_integration_steps",integration_steps,2);

  // Read CartParallelPole specific options
  double massc, massp1, massp2, lp1, lp2, fricc, fricp1, fricp2;
  vp.get("mass_cart",massc,1.0);
  vp.get("mass_pole1",massp1,0.1);
  vp.get("mass_pole2",massp2,0.01);
  vp.get("length_pole1",lp1,0.5);
  vp.get("length_pole2",lp2,0.05);
  vp.get("friction_cart",fricc,0.0);
  vp.get("friction_pole1",fricp1,0.0);
  vp.get("friction_pole2",fricp2,0.0);
  vp.get("angle_max1",angle_max1,5000);
  vp.get("angle_max2",angle_max2,5000);
  vp.get("angle_vel_max1",angular_vel_max1,100.); // unused
  vp.get("angle_vel_max2",angular_vel_max2,100.); // unused
  vp.get("position_max",position_max,2.4);
  vp.get("cart_vel_max",cart_vel_max,100.); // unused
  vp.get("delta_t",delta_t,0.01);
  vp.get("u_max",u_max,1E6);
  vp.get("noise_level",noise_level,0);

  // Initialize pole dynamics
  int ret = vp.get("dynamics",paramstr,MAX_STR_LEN);
  if (ret > 0) {
    if (strcmp(paramstr,"Gomez") == 0 || strcmp(paramstr,"Miikkulainen") == 0 || strcmp(paramstr,"Gomez-Miikkulainen")) {
      IOUT("Using Gomez-Miikkulainen dynamics model.");
      pole = new GomezMiikkulainen();
    } else if (strcmp(paramstr,"Sutton") == 0 || strcmp(paramstr,"Anderson") == 0 || strcmp(paramstr,"Sutton-Anderson")) {
      IOUT("Using Sutton-Anderson dynamics model.");
      pole = new ParallelPoleDynamicsOR();
    } else {
      EOUT("Invalid dynamics model \"" << paramstr << "\".");
      return false;
    }
  } else {
    WOUT(10, "No dynamics model specified, using Gomez-Miikkulainen.");
    pole = new GomezMiikkulainen();
  }

  pole->set_double(poles > 1);
  pole->set_delta(delta_t);
  pole->set_length(position_max, lp1, lp2);
  pole->set_mass(massc, massp1, massp2);
  pole->set_friction(fricc, fricp1, fricp2);

  // Show error-message if necessary
  if ( vp.num_of_not_accessed_entries() ) {
    EOUT("CartParallelPole: unrecognized options:");
    vp.show_not_accessed_entries(cerr);
    cerr << endl;
    return false;
  }

  // return values
  plant_state_dim = _sdim = poles * 2 + 2;
  measurement_dim = plant_state_dim;
  action_dim = 1;
  return true;
}

REGISTER_PLANT(CartParallelPole, "Simulation of a parallel pole balancing cart.")

