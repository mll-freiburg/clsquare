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

#include "filterobserver.h"
#include "valueparser.h"
#include <cstring>
#include <cstdio>
#include <cmath>

using namespace FilterCollection;

bool FilterObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);
  vp.get("initial", _initial, 0);

  // determine dimensions
  vp.get("append", _append, false);
  _mdim = measurement_dim;
  _inputs = new int[_mdim];
  _cdim = vp.get("inputs", _inputs, _mdim);
  if (_cdim < 1) {
    EOUT("No input features to filter specified.");
    return false;
  }
  observed_state_dim = _append ? _mdim + _cdim : _mdim;

  // choose filter type
  char mode[255];
  vp.get("mode", mode, 255);
  FilterCollection::Filter *filter;
  if (strcmp(mode, "highpass") == 0) {
    filter = new HighpassFilter();
  } else if (strcmp(mode, "semipass") == 0) {
    filter = new SemipassFilter();
  } else if (strcmp(mode, "lowpass") == 0) {
    filter = new LowpassFilter();
  } else if (strcmp(mode, "outlier") == 0) {
    filter = new OutlierFilter();
  } else if (strcmp(mode, "average") == 0) {
    filter = new AveragerFilter();
  } else if (strcmp(mode, "hysteresis_lock") == 0) {
    filter = new HysteresisLockFilter();
  } else if (strcmp(mode, "change") == 0) {
    filter = new ChangeFilter();
  } else {
    EOUT("Unknown filter mode \"" << mode << "\".");
    return false;
  }
  if (!filter->init(fname, chapter==NULL ? "Controller" : chapter)) {
    EOUT("Could not initialize filter.");
    return false;
  }

  for (_i=0; _i<_cdim; _i++)
    _filters.push_back(filter->clone());

  return true;
}

void FilterObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  // initialize filters
  for (; _initial>0; _initial--)
    for (_i=0; _i<_cdim; _i++)
      _filters[_i]->process(current_measurement[_inputs[_i]]);
  

  // keep original state?
  if (_append)
    for (_i=0; _i<_mdim; _i++)
      observed_state[_i] = current_measurement[_i];

  // filter measurement
  for (_i=0; _i<_cdim; _i++)
    observed_state[_append ? _mdim+_i : _inputs[_i]] = _filters[_i]->process(current_measurement[_inputs[_i]]);
}

REGISTER_OBSERVER(FilterObserver, "Applies a variety of DSP filters to the observation.");

