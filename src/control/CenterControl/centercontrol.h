/*
clsquare - closed loop simulation system
Copyright (c) 2010-2012 Machine Learning Lab, 
Prof. Dr. Martin Riedmiller, University of Freiburg

Author: Thomas Lampe

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

#ifndef _CENTERCONTROL_H_
#define _CENTERCONTROL_H_

#include "controller.h"
#include "valueparser.h"
#include <cmath>

/**
  * Interface for a CenterControl variant.
  **/
struct CenterControlModule
{
  virtual double get_action (const double state, const int dim) = 0;
  virtual bool init (ValueParser &vp, const int adim) = 0;
};

/**
  * Implements a number of simple controllers, all of which will attempt to generate the state vector specified
  * by the paramete \b targets, or the null vector by default, by generating adjustments based on the current state
  * on a per-state-dimension basis.
  *
  * The most important setting is the parameter \b mode, which toggles the type of controller used.
  * \li \e BangBang: gives +1 if the state is lower than the target, -1 if higher, or 0 if exactly there
  * \li \e Proportional: simple proportional control; takes an additional parameter \b scaling that the state will
  *     be divided by to yield the action (default: 1); \b scaling should be a list of the same length as the action
  *     dimension, but will be padded with the first entry if shorter
  *     \note This is \e not a classical P-controller, and does not generate a signal relative to an error, but to
  *           the state; to emulate a P-controller, just use a negative scaling.
  * \li \e ThreePoint: like BangBang, but with a histeresis margin around the goal in which the action 0 is chosen,
  *     defined by the parameter \b margin (default: 0.5); the list of values will be padded in the same manner as
  *     \b scaling above
  * \li \e FivePoint: like ThreePoint, but with two different zones in which different actions are chosen; is within
  *     the area defined by \b inner, it will be 0, if outside \b inner but within \b outer, then 0.5, and 1 if
  *     outside \b outer; the lists of values will be padded in the same manner as \b scaling above
  * \li \e Static: always returns the same action, defined by \b value; the lists of values will be padded in the
  *     same manner as \b scaling above
  *
  * All controllers generate an action in \f$[-1:1]\f$ byx default, unless modified through additional parameters.
  * \li \b negative <bool>: if set to false, output actions will be scaled between 0 and 1
  *     (default: \e false)
  * \li \b multi <\f$a\times\f$ bool>: multiplier that will be applied to the controller-generated 
  *     action (default: \f$[a\times 0]\f$)
  * \li \b limit <bool>: if set to false, actions will no longer be limited to the range \f$[-1:1]\f$, but use
  *     the absolute output of the module; (default: \e true for MixControlStatic, \e false for all other types)
  *
  * If the action is of a lower dimensionality than the state, state dimensions must be assigned to action
  * dimensions through the parameter \b assignment <\f$a\times\f$ double> (default: [0 1 2...]).
  *
  * @ingroup CONTROLLER
  * @ingroup STATIC
  * @author Thomas Lampe
  **/
class CenterControl : public Controller
{
 public:
  bool get_action (const double* state, double* action);
  bool init (const int observed_state_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  void deinit() {};

  CenterControl() {};
  ~CenterControl() {};

 protected:

  CenterControlModule *_module;
  bool _negative, _limit;
  int _adim, _sdim, _i, *_ass;
  double *_target, *_multi, _fbl;
};

struct CenterControlBangBang : public CenterControlModule
{
  double get_action (const double state, const int dim) {
    return state == 0. ? 0. : state < 0. ? -1. : 1.;
  };
  virtual bool init (ValueParser &vp, const int adim);
};

struct CenterControlProportional : public CenterControlModule
{
  inline double get_action (const double state, const int dim) {
    return state / scaling[dim];
  };
  virtual bool init (ValueParser &vp, const int adim);
  double *scaling;
};

struct CenterControlThreePoint : public CenterControlModule
{
  inline double get_action (const double state, const int dim) {
    return fabs(state) < margin[dim] ? 0. : state < 0. ? -1. : 1.;
  };
  virtual bool init (ValueParser &vp, const int adim);
  double *margin;
};

struct CenterControlFivePoint : public CenterControlModule
{
  inline double get_action (const double state, const int dim) {
    _tmp_act = fabs(state) < margin[0][dim] ? 0. : fabs(state) < margin[1][dim] ? 0.5 : 1.;
    return state < 0. ? -_tmp_act : _tmp_act;
  };
  virtual bool init (ValueParser &vp, const int adim);
  double *margin[2];
  double _tmp_act;
};

struct CenterControlStatic : public CenterControlModule
{
  inline double get_action (const double state, const int dim) {
    return value[dim];
  };
  virtual bool init (ValueParser &vp, const int adim);
  double *value;
};


#endif
