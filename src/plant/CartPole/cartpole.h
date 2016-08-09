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

#ifndef _CARTPOLE_PLANT_H_
#define _CARTPOLE_PLANT_H_

#include "plant.h"

/** Realizes a classical cart pole system where a cart with a freely rotating
  * pole attached can be moved on a linear track.
  *
  * This system has a four-dimensional state: 
  * \li angle of pole
  * \li angular velocity of pole
  * \li position of cart
  * \li velocity of cart
  *
  * It accepts a one-dimensional action:
  * \li force to apply to the cart 
  *
  * For a detailed description of the system dynamics see: "Neuronlike Adaptive Elements That
  * Can Solve  Difficult Learning Control Problems", R.Sutton and C.W.Anderson, IEEE Transactions
  * on Systems, Man, and Cybernetics, Vol. SMC-13, No. 5, 1983
  *
  * The physical properties can be configured with the self-explanatory parameters
  * \b mass_cart, \b mass_pole, \b length_pole, \b friction_cart, \b friction_pole.
  * \noise_level scales the amount of gaussian noise applied to the state, with
  * a level of 1 resulting in a standard deviation of 0.5 for the pole angle and 2.0
  * for all other dimensions.
  *
  * The parameters \b angle_max and \b position_max define failure criteria that will
  * abort an episode if exceeded. Ideally, Reward or Controller module should be
  * used instead. \b u_max can be used to prevent the action from exceeding a certain
  * value.
  *
  * A simulation timestep takes \b delta_t seconds, and \b number_of_integration_steps
  * calculations are performed between program cycles.
  *
  * @ingroup PLANT
  * @ingroup SIMULATION
  */
class CartPole : public Plant {
 public:
  virtual bool get_next_plant_state(const double *state, const double *action, double *next_state);
  virtual bool init(int& plant_state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 
  virtual void deinit();
  virtual const std::string get_default_graphics() {return "CartPoleGraphic";};

 protected:

  /** task mode (swing up or balancing). */
  bool task_mode;

  /** maximal force applied to cart. */
  double u_max;

  /** noise level. */
  double noise_level;
  
  /** Maximal angle of pole. */
  float angle_max;

  /* Maximal angular velocity of poles. */
  //float angular_vel_max;

  /** half length of track (0 is origin) */
  float position_max;

  /* Maximal velocity of cart. */
  //float cart_vel_max;
  
  /** Length of pole. */
  float lp;
  
  /** Mass of pole. */
  float massp;

  /** Mass of cart. */
  float massc;

  /** Friction of cart. */
  float fricc;

  /** Friction of pole. */
  float fricp;
  
  /** Number of integration steps for computing system dynamics */
  int integration_steps;

  /** Duration of one control cycle in seconds. */
  float delta_t;
  
  /** Reads configuration for cart pole plant. 
   * \param fname filename
   * \return true for success */
  bool read_options(const char * fname, const char * chapter);
};

#endif

