/*
clsquare - closed loop simulation system
Author: Martin Riedmiller
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

#ifndef _cartpolereward_h_
#define _cartpolereward_h_

#include "global.h"
#include "reward.h"


/** Computes a weighted sum of action values.
  * The total reward is calculated as
  * \f$ \Sigma_{i=0}^{udim} w_i|u_i^x| \f$,
  * with \f$\vec{w}\f$ being defined by the config parameter
  * \b Udiag and \f$x\f$ being 1 by default or 2 if
  * \b quadratic_costs has been set to \true.
  *
  * @ingroup REWARD */
class CartPoleReward : public Reward 
{ 
public:
  
  CartPoleReward() : Reward() {}
  
  virtual bool init(int plant_state_dim, int measurement_dim, int observed_state_dim, int action_dim, int* expected_input_type, const char* fname);
  
  virtual double get_reward(const double *current_state_representation, const double *current_action, const double *next_state_representation);
  virtual double get_terminal_reward(const double *state_representation);
  
  virtual ~CartPoleReward() {}
  
protected:
  bool read_options(const char * fname);
  bool quadratic_costs;

  int u_dim;
  double* Ucostdiag;
  //bool ruessel3D;
};

#ifdef CLSQUARE
#include "registry.h"
#else
#define REGISTER_REWARD(classname, desc)
#endif

#endif
