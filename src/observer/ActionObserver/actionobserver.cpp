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

#include "actionobserver.h"
#include "valueparser.h"

bool ActionObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);

  // dimensions to record
  _dims = new int[action_dim];
  _adim = vp.get("assignment", _dims, action_dim);
  if (_adim < 1) {
    _adim = action_dim;
    for (_i=0; _i<_adim; _i++)
      _dims[_i] = _i;
  }

  // default action
  vp.get("null_action", _null, 0.);

  // sanity check
  for (_i=0; _i<_adim; _i++)
    if (_dims[_i] < 0 || _dims[_i] >= action_dim) {
      EOUT("Action index " << _dims[_i] << " out of bounds.");
      return false;
  }

  _mdim = measurement_dim;
  observed_state_dim = _mdim + _adim;
  IOUT("Adding " << _adim << " actions to observation.");
  return true;
}


void ActionObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  for (_i=0; _i<_mdim; _i++)
    observed_state[_i] = current_measurement[_i];
  for (_i=0; _i<_adim; _i++)
    observed_state[_mdim+_i] = cycle_ctr < 2 ? _null : prev_action[_dims[_i]];
}

REGISTER_OBSERVER(ActionObserver, "Observer to add the action.");

