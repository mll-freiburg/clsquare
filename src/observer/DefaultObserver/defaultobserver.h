/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck
Copyright (c) 2011, Machine Learning Lab, Prof. Dr. Martin Riedmiller,
University of Freiburg

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

#ifndef _default_observer_h_
#define _default_observer_h_

#include "observer.h"
#include <sstream>

/** Basic implementation of the Observer class.
  * Simply returns the unchanged plant measurement.
  *
  * @ingroup OBSERVER */
class DefaultObserver : public Observer
{
public:
  /** init gets plant_state_dim, measurement_dim
   * \return observed_state_dim
   */
  bool init(const int plant_state_dim, const int measurement_dim, const int action_dim,
            int &observed_state_dim, const char *fname=0, const char *chapter=0);

  /** compute observed state out of measurement
   *  prev. information might be used to build up history
   *  to check, if history is valid, the cycle_ctr is also communicated
   * \return observed_state
   */
  void get_observed_state(const double *prev_measurement, const double* prev_action,
                          const double *current_measurement, const int cycle_ctr,
                          double *observed_state);

  /** Notifies that an episode has been started. */
  inline void notify_episode_starts() {
    return;
  }

  /** Notifies that an episode has been stopped. */
  inline void notify_episode_stops() {
    return;
  }

  inline void deinit() {
    ;
  }

  DefaultObserver() {__verbose = false;};

  ~DefaultObserver() {};

protected:
  int __observed_state_dim;

  int __verbose;

  std::stringstream __ss;
};

#endif
