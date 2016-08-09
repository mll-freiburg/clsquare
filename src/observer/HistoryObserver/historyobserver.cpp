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

#include "historyobserver.h"
#include "valueparser.h"

bool HistoryObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);

  // number of steps
  vp.get("memory", _mem, 1);

  // static mode
  vp.get("static", _static, false);
  if (_static) _mem = 1;

  // integral mode
  vp.get("integrate", _add, false);
  
  // dimensions to record
  _dim = vp.get("assignment", _dims, HISTOBS_MAX_DIM);
  if (_dim < 1) WOUT(10, "No dimensions to generate history for specified.");
  for (_i=0; _i<_dim; _i++)
    if (_dims[_i] >= measurement_dim) {
      EOUT("History index " << _dims[_i] << " exceeds measurement dimension " << measurement_dim << "!");
      return false;
  }

  _hdim = _mem * _dim;
  _history = new double[_hdim];
  observed_state_dim = measurement_dim + _hdim;
  _mdim = measurement_dim;
  IOUT("Adding history of " << _mem << " steps for " << _dim << " state dimensions.");
  return true;
}

void HistoryObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  // assume system has been within starting state all the time
  if (cycle_ctr < 2) {
    for (_i=0; _i<_dim; _i++)
      for (_k=0; _k<_mem; _k++)
        _history[_i*_mem+_k] = current_measurement[_dims[_i]];
  }

  // update history
  else if (!_static) {
    for (_i=_hdim-1; _i>0; _i--)
      _history[_i] = _history[_i-1];
    for (_i=0; _i<_dim; _i++)
      if (_add)
        _history[_i*_mem] += prev_measurement[_dims[_i]];
      else
        _history[_i*_mem] = prev_measurement[_dims[_i]];
  }

  // copy to observation
  for (_i=0; _i<_mdim; _i++)
    observed_state[_i] = current_measurement[_i];
  for (_i=0; _i<_hdim; _i++)
    observed_state[_mdim+_i] = _history[_i];
}

REGISTER_OBSERVER(HistoryObserver, "Observer to add a history.");

