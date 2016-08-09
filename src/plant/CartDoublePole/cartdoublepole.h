/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Thanks to Bernd Weisser for providing plant dynamics

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

#ifndef _CARTDOUBLEPOLE_PLANT_H_
#define _CARTDOUBLEPOLE_PLANT_H_

#include "plant.h"

/** Realizes a double cart pole system where a cart with a freely rotating
  * pole attached can be moved on a linear track, and a second pole
  * attached to the end of the first one.
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
  * The physical behavior can be adjusted by the parameters
  * \b mass_cart, \b mass_1, \b mass_2, \b length_1, \b length_2.
  *
  * If the state leaves the constraints defined by the parameters
  * \b angle1_max, \b angle2_max, \b angle_vel_max, \b position_max
  * and \b cart_vel_max, the episode is aborted.
  * 
  * A simulation timestep takes \b delta_t seconds, and \b number_of_integration_steps
  * calculations are performed between program cycles.
  *
 * @ingroup PLANT
 * @ingroup SIMULATION
 */
   class CartDoublePole : public Plant {
 public:
  virtual bool get_next_plant_state(const double *state, const double *action, double *next_state);
  virtual bool init(int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 
  virtual const std::string get_default_graphics() {return "CartDoublePoleGraphic";};
  
 protected:  
  /** half length of track (0 is origin) */
  float position_max;

  /** Maximal angle of first pole. */
  float angle1_max;

  /** Maximal angle of second pole. */
  float angle2_max;

  /** Maximal angular velocity of poles. */
  float angular_vel_max;

  /** Maximal velocity of cart. */
  float cart_vel_max;

  /** Mass of cart. */
  float Mc;
  
  /** Mass of first pole. */
  float M1;
  
  /** Mass of second pole. */
  float M2;
  
  /** Full Length of first pole. */
  float L1;

  /** Full Length of second pole. */
  float L2;
  
  /** Duration of one control cycle in seconds. */
  float delta_t;

  /** Number of integration steps for computing system dynamics */
  int integration_steps;

  /** Reads configuration for double-pole plant. 
   * \param fname filename
   * \return true for success */
  bool read_options(const char * fname, const char * chapter);
};

#endif

