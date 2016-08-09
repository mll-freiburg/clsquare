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

#include "intervalobserver.h"
#include "valueparser.h"
#include <cstring>

void IntervalObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  for (_i=0; _i<_mdim; _i++)
    observed_state[_i] = current_measurement[_i];

  _tmp = current_measurement[_index];
  switch (_mode) {
    case cut:
      _tmp = _tmp < _target[0] ? _target[0] : _tmp > _target[1] ? _target[1] : _tmp;
      break;
    case shift:
      while (_tmp < _target[0]) _tmp += _step;
      while (_tmp > _target[1]) _tmp -= _step;
      break;
    default:
      EOUT("Invalid mode!");
  }
  observed_state[_mdim] = _tmp;
}

bool IntervalObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);
  _mdim = measurement_dim;
  observed_state_dim = _mdim + 1;

  // dimension to use
  vp.get("measurement", _index, -1);
  if (_index < 0 || _index >= _mdim) {
    EOUT("Invalid measurement index specified.");
    return false;
  }

  // target region
  if (vp.get("target", _target, 2) < 2) {
    EOUT("No valid target region specified.");
    return false;
  }
  if (_target[1] == _target[0]) {
    EOUT("Target region has no size.");
    return false;
  }
  if (_target[1] < _target[0]) {
    WOUT(10, "Region boundaries are reversed, switching them.");
    double tmpd = _target[0];
    _target[0] = _target[1];
    _target[1] = tmpd;
  }

  // mode
  char tmp[255];
  vp.get("mode", tmp, 255);
  _mode = cut;
  if (strcmp(tmp, "shift") == 0) {
    _mode = shift;
    vp.get("step", _step, 1.);
    if (_step > _target[1]-_target[0]) {
      _step = _target[1] - _target[0];
      WOUT(10, "Specified step size exceeds target area size, reducing it to " << _step << ".");
    } 
  } else if (strcmp(tmp, "cut") != 0) {
    WOUT(10, "No valid mode specified (" << tmp << "), using \"cut\" instead.");
  }

  return true;
}

REGISTER_OBSERVER(IntervalObserver, "Observer to pick state dimension based on conditions.");

