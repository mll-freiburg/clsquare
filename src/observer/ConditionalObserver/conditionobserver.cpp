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

#include "conditionobserver.h"
#include "valueparser.h"
#include <fstream>

#define SSSET(xxx) { ss.str(""); ss << xxx; };

void ConditionalObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  for (_i=0; _i<_mdim; _i++)
    observed_state[_i] = current_measurement[_i];

  if (_release.size() < 1)
    _choice = 0;

  if (_choice == 0) {
    for (_i=0; (unsigned int)_i<_activate.size(); _i++)
      if (_activate[_i].isWithinSet(current_measurement, _mdim)) _choice = 1;
  } else {
    for (_i=0; (unsigned int)_i<_release.size(); _i++)
      if (_release[_i].isWithinSet(current_measurement, _mdim)) _choice = 0;
  }

  for (_i=0; _i<_cdim; _i++)
    observed_state[_i+_mdim] = current_measurement[_assign[_i+_choice*_mdim]];
}

bool ConditionalObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);
  std::stringstream ss;
  _mdim = measurement_dim;

  // dimensions to use
  _assign = new int[2*_mdim];
  _cdim = vp.get("default", &_assign[0], _mdim);
  if (_cdim < 1) {
    EOUT("No measurement dimensions to use specified.");
    return false;
  }
  if (vp.get("substitute", &_assign[_mdim], _mdim) != _cdim) {
    EOUT("Lengths of default and substitute specification do not match.");
    return false;
  }
  SSSET("Using dimensions ");
  for (_i=0; _i<_cdim; _i++) ss << _assign[_i] << " ";
  ss << "as default, ";
  for (_i=0; _i<_cdim; _i++) ss << _assign[_i+_mdim] << " ";
  ss << "as substitute.";
  IOUT(ss.str().c_str());
  observed_state_dim = _mdim + _cdim;

  // activation conditions
  for (_i=0; ; _i++) {
    char *tmp = new char[255];
    SSSET("activate_" << _i);
    if (vp.get(ss.str().c_str(), tmp, 255) < 1) break;

    SetDef set;
    if (!set.parseStr(tmp, _mdim)) break;

    IOUT("Activation condition " << _i << ":" << tmp);
    _activate.push_back(set);
  }

  if (_activate.size() < 1)
    WOUT(10, "No activation conditions specified.");

  // release conditions
  for (_i=0; ; _i++) {
    char *tmp = new char[255];
    SSSET("release_" << _i);
    if (vp.get(ss.str().c_str(), tmp, 255) < 1) break;

    SetDef set;
    if (!set.parseStr(tmp, _mdim)) break;

    IOUT("Release condition " << _i << ":" << tmp);
    _release.push_back(set);
  }

  if (_release.size() < 1)
    IOUT("No release conditions specified. Will release subsitute action as soon as activation criteria are not met anymore.");

  return true;
}

REGISTER_OBSERVER(ConditionalObserver, "Observer to pick state dimension based on conditions.");

