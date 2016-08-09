/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

All rights reserved.

Author: Roland Hafner

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

#ifndef _STATECONTROLLER_H_
#define _STATECONTROLLER_H_

#include "controller.h"
#include "setdef.h"
#include "global.h"
#include "funcgen1d.h"

/** Generates an action from a linear combination of the
  * state dimensions or from a set of linear interpolation
  * functions.
  *
  * The basic behavior is determined by the parameter \b mode.
  * \li If set to 0, a one-dimensional  action will be calculated
  *     as \f$a = o + \Sigma_{i=0}^{xdim} g_ix_i \f$, with
  *     \f$o\f$ defined by \b offsetAction and \f$\vec{g}\f$
  *     by \b gain. The result is capped within the interval
  *     described by \b minAction and \b maxAction. If the
  *     state lies outside the region defined by the SetDef
  *     \b xwork, the episode is aborted.
  * \li If set to 1, the n-dimensional action will be calculated
  *     as \f$ a_i=f_i(t) \f$, with \f$t\f$ being the current time
  *     (cycle \f$\cdot\f$ \b delta_t) and \f$f_i(t)\f$ being
  *     a linear interpolation function of class InterpolLinFun1D,
  *     with samples defined in the file \b ref_inp_fun_i.
  *
  * @author Roland Hafner
  * @ingroup CONTROLLER   
  * @ingroup STATIC */

class StateController : public Controller {
 public:
  virtual ~StateController(){;}
  virtual bool get_action(const double* observed_state, double* action);
  virtual bool init(const int observated_state_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  virtual void deinit();

  virtual void notify_episode_stops(const double* current_observed_state);
  
 protected:
  int xdim;
  int udim;
  double delta_t;

  int      mode;
  int      steps;

  double   maxAction;
  double   minAction;
  double   offsetAction;
  double*  gain;
  SetDef   xwork;
  SetDef   xplus;

  double   cummulated_reward;
  long     trajctr;

  std::vector< InterpolLinFun1D* > ref_inp_fun;

  bool read_options(const char* fname, const char* chapter);
};
#endif
