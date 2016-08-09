/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Based on a previous implementation by Ralf Schoknecht and Artur Merke

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

#ifndef _CARTPARALLELPOLE_PLANT_H_
#define _CARTPARALLELPOLE_PLANT_H_

#include "plant.h"
#include "parallelpole_dynamics.h"

/** Realizes a double cart pole system where a cart with two freely rotating
  * poles of different length attached next to each other can be moved on a
  * linear track.
  *
  * This double pole has a six-dimensional state: 
  * \li angle of first pole
  * \li angular velocity of first pole
  * \li angle of second pole
  * \li angular velocity of second pole
  * \li position of cart
  * \li velocity of cart
  *
  * It accepts a one-dimensional action:
  * \li force to apply to the cart
  *
  * The default plant dynamics are taken from "Robust Non-Linear Control Through Neuroevolution",
  * F.J. Gomez and R. Miikkulainen, 2002. An adaption of the dynamics used in the CartPole plant
  * can be activated by setting the parameter \b gomez to \e false.
  *
  * The physical properties can be adjusted with the parameters
  * \b mass_cart, \b mass_pole1, \b mass_pole2, \b length_pole1, \b length_pole2,
  * \b friction_cart, \b friction_pole1, \b friction_pole2.
  * \noise_level scales the amount of gaussian noise applied to the state, with
  * a level of 1 resulting in a standard deviation of 0.5 for the pole angle and 2.0
  * for all other dimensions.
  *
  * If the state leaves the constraints defined by the parameters
  * \b angle1_max, \b angle2_max, and \b position_maxi, the episode is aborted.
  * 
  * A simulation timestep takes \b delta_t seconds, and \b number_of_integration_steps
  * calculations are performed between program cycles.
  *
  * The plant can also be used as a single-pole system by setting the parameter
  * \b poles to 1, with the state vector being reduced to four elements.
  *
  * @ingroup PLANT
  * @ingroup SIMULATION
  */
class CartParallelPole : public Plant
{
public:
  virtual bool get_next_plant_state(const double *state, const double *action, double *next_state);
  virtual bool init(int& plant_state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0);
  inline virtual bool check_initial_state (double* state) {pole->notify_episode_starts(state); return true;};
  inline virtual void deinit() {};
  inline virtual const std::string get_default_graphics() {return "CartParallelPoleGraphic";};

protected:
  ParallelPoleDynamics *pole;
  double u_max;
  double noise_level;
  float angle_max1, angle_max2;
  float angular_vel_max1, angular_vel_max2;
  float position_max;
  float cart_vel_max;
  int _sdim;
};

#endif

