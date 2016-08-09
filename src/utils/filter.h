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

#ifndef _FILTER_COLLECTION_H_
#define _FILTER_COLLECTION_H_

#include "global.h"
#include <vector>

namespace FilterCollection
{

class Filter
{
 public:
  virtual bool init (const char *fname, const char* chapter) = 0;
  virtual int process (const double raw) = 0;
  virtual Filter* clone () = 0;
};

/** Standard high-pass filter.
  * Filters out signals below the frequency set by \b cutoff_frequency. 
  * Assumes the sampling time in seconds is equal to \b delta_t. */
class HighpassFilter : public FilterCollection::Filter
{
 public:
  bool init (const char *fname, const char* chapter);
  int process (const double raw);
  Filter* clone () {return new HighpassFilter(*this);};
 protected:
  double _alpha, _last_raw, _last_filtered;
  bool _first;
};

/** Modified high-pass filter using low-pass alpha. */
class SemipassFilter : public HighpassFilter
{
 public:
  bool init (const char *fname, const char* chapter);
  Filter* clone () {return new SemipassFilter(*this);};
};

/** Standard low-pass filter.
  * Filters out signals above the frequency set by \b cutoff_frequency. 
  * Assumes the sampling time in seconds is equal to \b delta_t. */
class LowpassFilter : public FilterCollection::Filter
{
 public:
  bool init (const char *fname, const char* chapter);
  int process (const double raw);
  Filter* clone () {return new LowpassFilter(*this);};
 protected:
  double _alpha, _last;
  bool _first;
};

/** Removes values that differ too much from their
  * two predecessors. Any value that differs by more than
  * \b max_deviation is replaced by the previous one. */
class OutlierFilter : public FilterCollection::Filter
{
 public:
  bool init (const char *fname, const char* chapter);
  int process (const double raw);
  Filter* clone () {return new OutlierFilter(*this);};
 protected:
  double _max_dev, _last[2];
  bool _outlier, _first;
};

/** Standard uniform window averager.
  * The value returned equals the average across the
  * last \b window_size steps. */
class AveragerFilter : public FilterCollection::Filter
{
 public:
  bool init (const char *fname, const char* chapter);
  int process (const double raw);
  Filter* clone () {return new AveragerFilter(*this);};
 protected:
  int _window, _current;
  double _total, *_buffer;
  bool _first;
};

/** Locks the signal at a predefined threshold when it is exceeded,
  * until it falls below a second threshold.
  * A lock occurs is the \b activation_threshold has been exceeded
  * for the duration \b activation_steps. The lock is only released
  * if the signal remains below \b release_threshold for \b release_steps
  * number of steps. */
class HysteresisLockFilter : public FilterCollection::Filter
{
 public:
  bool init (const char *fname, const char* chapter);
  int process (const double raw);
  Filter* clone () {return new HysteresisLockFilter(*this);};
 protected:
  int _i, _activation_steps, _activation_index, _return_steps, _return_index;
  double _activation_thresh, *_activation_buffer, _return_thresh, *_return_buffer, _current;
  bool _change, _negative;
};

/** Returns the input value only if it differs from the previous one,
  * and 0 otherwise. */
class ChangeFilter : public FilterCollection::Filter
{
 public:
  bool init (const char *fname, const char* chapter);
  int process (const double raw);
  Filter* clone () {return new ChangeFilter(*this);};
 protected:
  double _last;
};

};

#endif
