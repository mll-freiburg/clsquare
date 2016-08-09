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

#include "centercontrol.h"
#include "valueparser.h"
#include <string.h>
#include <stdio.h>
#include <cmath>

bool CenterControl::get_action (const double *state, double *action)
{
  for (_i=0; _i<_adim; _i++) {
    action[_i] = _ass[_i] < 0 || _ass[_i] >= _sdim ? 0. : _module->get_action(state[_ass[_i]] - _target[_i], _i);
    action[_i] *= _multi[_i];
    if (_limit) {
      if (action[_i] < -1.) action[_i] = -1.;
      else if (action[_i] > 1.) action[_i] = 1.;
    }
    if (!_negative) {
      action[_i] += 1.;
      action[_i] /= 2.;
    }
    if (fabs(action[_i]) < _fbl) return false;
  }

  return true;
}

bool CenterControl::init (const int observed_state_dim, const int action_dim, double delta_t, const char *fname, const char* chapter)
{
  // set dimensions
  _adim = action_dim;
  _sdim = observed_state_dim;
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);

  vp.get("negative", _negative, true);
  vp.get("limit", _limit, true);
  vp.get("false_below", _fbl, -1.);

  // set centering targets
  _target = new double[_adim];
  for (_i=0; _i<_adim; _i++) _target[_i] = 0.;
  vp.get("targets", &_target[0], _adim);

  // set state-to-action assignment; use first (action_dim) states by default
  _ass = new int[_adim];
  for (_i=0; _i<_adim; _i++)
    _ass[_i] = _i < _sdim ? _i : -1.;
  vp.get("assignment", &_ass[0], _adim);

  // set multiplier
  _multi = new double[_adim];
  for (_i=0; _i<_adim; _i++)
    _multi[_i] = 1.;
  vp.get("multi", &_multi[0], _adim);

  // determine controller mode
  char paramstr[255];
  _limit = true;
  if (vp.get("mode",paramstr,255) >= 0) {
    if (strcmp(paramstr,"Proportional") == 0) {
      _module = new CenterControlProportional();
    } else if (strcmp(paramstr,"ThreePoint") == 0) {
      _module = new CenterControlThreePoint();
    } else if (strcmp(paramstr,"FivePoint") == 0) {
      _module = new CenterControlFivePoint();
    } else if (strcmp(paramstr,"Static") == 0) {
      _module = new CenterControlStatic();
      _limit = false;
    } else {
      if (strcmp(paramstr,"BangBang") != 0) {
        WOUT(10, "Unknown mode: " << paramstr << "; using \"BangBang\" instead");
      }
      _module = new CenterControlBangBang();
    }
  } else {
    WOUT(10, "No controller mode specified; using \"BangBang\" instead");
    _module = new CenterControlBangBang();
  }
  _module->init(vp, _adim);
  vp.get("limit", _limit);

  // output
  std::stringstream ss;
  ss << "Parameters: ";
  ss << "state->action assignment [";
  for (_i=0; _i<_adim; _i++) {
    if (_i > 0) ss << " ";
    ss << _ass[_i];
  }
  ss << "], targets [";
  for (_i=0; _i<_adim; _i++) {
    if (_i > 0) ss << " ";
    ss << _target[_i];
  }
  ss << "]";
  IOUT(ss.str());

  return true;
}

bool CenterControlBangBang::init (ValueParser &vp, const int adim)
{
  IOUT("Using bang-bang controller");
  return true;
}

// if a parameter array is not completely full, copy first entry
#define FILL_ARRAY(_arr_,_def_,_cfg_) \
  _arr_ = new double[adim]; \
  _arr_[0] = _def_; \
  ent = vp.get(_cfg_, _arr_, adim); \
  for (int a=ent; a<adim; a++) \
    _arr_[a] = _arr_[0];

bool CenterControlProportional::init (ValueParser &vp, const int adim)
{
  int ent;
  FILL_ARRAY(scaling, 1., "scaling");

  stringstream ss;
  for (int i=0; i<adim; i++)
    ss << scaling[i] << " ";
  IOUT("Using Proportional controller with scaling=" << ss.str());
  return true;
}

bool CenterControlThreePoint::init (ValueParser &vp, const int adim)
{
  int ent;
  FILL_ARRAY(margin, 0.5, "margin");

  stringstream ss;
  for (int i=0; i<adim; i++)
    ss << margin[i] << " ";
  IOUT("Using three-point controller with margin=" << ss.str());
  return true;
}

bool CenterControlFivePoint::init (ValueParser &vp, const int adim)
{
  // legacy
  double tmp[2] = {.33, .66};
  vp.get("margin", tmp, 2);

  int ent;
  FILL_ARRAY(margin[0], tmp[0], "inner");
  FILL_ARRAY(margin[1], tmp[1], "outer");

  stringstream ss;
  for (int i=0; i<adim; i++)
    ss << margin[0][i] << " ";
  ss << "and ";
  for (int i=0; i<adim; i++)
    ss << margin[1][i] << " ";
  IOUT("Using five-point controller with margins=" << ss.str());
  return true;
}

bool CenterControlStatic::init (ValueParser &vp, const int adim)
{
  int ent;
  FILL_ARRAY(value, 0., "value");

  stringstream ss;
  for (int i=0; i<adim; i++)
    ss << value[i] << " ";
  IOUT("Using static output with value=" << ss.str());
  return true;
}

REGISTER_CONTROLLER(CenterControl, "Controller which tries to center the state around zero.")
