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

#include "filter.h"
#include "valueparser.h"
#include <cstring>
#include <cstdio>
#include <cmath>

using namespace FilterCollection;

#define GET_REQUIRED_FILTER_PARAMETER(_param_,_string_) \
  if (vp.get(_string_,_param_) < 1) { \
    EOUT("Required filter parameter \"" << _string_ << "\" not specified."); \
    return false; \
  }
#define CHECK_POSITIVE_FILTER_PARAMETER(_param_,_string_) \
  if (_param_ <= 0) { \
    EOUT("Filter parameter \"" << _string_ << "\" must be positive."); \
    return false; \
  }

bool HighpassFilter::init (const char *fname, const char* chapter)
{
  ValueParser vp(fname, chapter);

  double dt, fc;
  GET_REQUIRED_FILTER_PARAMETER(dt, "delta_t");
  GET_REQUIRED_FILTER_PARAMETER(fc, "cutoff_frequency");
  double RC = 1. / (2. * M_PI * fc);
  _alpha = RC / (RC + dt);

  _first = true;
  return true;
}

int HighpassFilter::process (const double raw)
{
  if (_first) {
    _last_raw = raw;
    _last_filtered = raw;
    _first = false;
    return raw;
  }

  _last_filtered = _alpha * (_last_filtered + raw - _last_raw);
  _last_raw = raw;
  return _last_filtered;
}

bool SemipassFilter::init (const char *fname, const char* chapter)
{
  ValueParser vp(fname, chapter);

  double dt, fc;
  GET_REQUIRED_FILTER_PARAMETER(dt, "delta_t");
  GET_REQUIRED_FILTER_PARAMETER(fc, "cutoff_frequency");
  double RC = 1. / (2. * M_PI * fc);
  _alpha = dt / (RC + dt);

  _first = true;
  return true;
}

bool LowpassFilter::init (const char *fname, const char* chapter)
{
  ValueParser vp(fname, chapter);

  double dt, fc;
  GET_REQUIRED_FILTER_PARAMETER(dt, "delta_t");
  GET_REQUIRED_FILTER_PARAMETER(fc, "cutoff_frequency");
  double RC = 1. / (2. * M_PI * fc);
  _alpha = dt / (RC + dt);

  _first = true;
  return true;
}

int LowpassFilter::process (const double raw)
{
  if (_first) {
    _last = raw;
    _first = false;
    return raw;
  }
  _last = _alpha * raw + (1. - _alpha) * _last;
  return _last;
}

bool AveragerFilter::init (const char *fname, const char* chapter)
{
  ValueParser vp(fname, chapter);

  GET_REQUIRED_FILTER_PARAMETER(_window, "window_size");
  CHECK_POSITIVE_FILTER_PARAMETER(_window, "window_size");
  _buffer = new double[_window];

  _current = 0;
  _first = true;
  return true;
}

int AveragerFilter::process (const double raw)
{
  if (_first) {
    for (int i=0; i<_window; i++)
      _buffer[i] = raw;
    _total = raw * _window;
    _first = false;
    return raw;
  }

  _total -= _buffer[_current];
  _buffer[_current] = raw;
  _total += _buffer[_current];

  _current++;
  _current %= _window;

  return _total / _window;
}

bool OutlierFilter::init (const char *fname, const char* chapter)
{
  ValueParser vp(fname, chapter);

  GET_REQUIRED_FILTER_PARAMETER(_max_dev, "max_deviation");
  CHECK_POSITIVE_FILTER_PARAMETER(_max_dev, "max_deviation");

  _first = true;
  return true;
}

int OutlierFilter::process (const double raw)
{
  if (_first) {
    _last[0] = _last[1] = raw;
    _first = false;
    return raw;
  }

  _outlier = fabs(raw-_last[0]) > _max_dev && fabs(raw-_last[1]) > _max_dev;
  _last[0] = _last[1];
  _last[1] = raw;
  return _outlier ? _last[0] : raw;
}

bool HysteresisLockFilter::init (const char *fname, const char* chapter)
{
  ValueParser vp(fname, chapter);

  GET_REQUIRED_FILTER_PARAMETER(_activation_thresh, "activation_threshold");
  GET_REQUIRED_FILTER_PARAMETER(_activation_steps, "activation_steps");
  CHECK_POSITIVE_FILTER_PARAMETER(_activation_steps, "activation_steps");
  _activation_buffer = new double[_activation_steps];
  _activation_index = 0;

  GET_REQUIRED_FILTER_PARAMETER(_return_thresh, "release_threshold");
  GET_REQUIRED_FILTER_PARAMETER(_return_steps, "release_steps");
  CHECK_POSITIVE_FILTER_PARAMETER(_return_steps, "release_steps");
  _return_buffer = new double[_return_steps];
  _return_index = 0;

  _current = 0.;
  return true;
}

#define DUAL_THRESHOLD_STEP(_side_) \
  _side_##_buffer[_side_##_index] = raw; \
  _side_##_index++; \
  _side_##_index %= _side_##_steps;

int HysteresisLockFilter::process (const double raw)
{
  DUAL_THRESHOLD_STEP(_activation);
  DUAL_THRESHOLD_STEP(_return);

  _change = true;
  if (_current != 0.) {
    for (_i=0; _i<_return_steps; _i++)
      _change &= fabs(_return_buffer[_i]) <= _return_thresh; // TODO using fabs here is very EOG-specific
    if (_change) _current = 0.;
  } else {
    _negative = _activation_buffer[0] < _activation_thresh;
    if (!_negative)
      for (_i=0; _i<_activation_steps; _i++)
        _change &= _activation_buffer[_i] >= _activation_thresh;
    else
      for (_i=0; _i<_activation_steps; _i++)
        _change &= _activation_buffer[_i] <= -_activation_thresh;
    if (_change) {
      _current = _activation_thresh;
      if (_negative) _current *= -1;
    }
  }

  //return _active || raw > _activation_thresh ? _activation_thresh : raw;
  return _current;
}

bool ChangeFilter::init (const char *fname, const char* chapter)
{
  _last = 0.;
  return true;
}

int ChangeFilter::process (const double raw)
{
  if (raw != _last) {
    _last = raw;
    return raw;
  }
  return 0;
}

