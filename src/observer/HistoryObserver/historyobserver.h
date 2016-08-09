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

#ifndef _HISTORY_OBSERVER_H_
#define _HISTORY_OBSERVER_H_

#include "observer.h"

#define HISTOBS_MAX_DIM 100

/** Observer class that extends the state with a history.
  * The parameter \b memory controls the number of past states to be included, while
  * \b assignment chooses the measurement indices for which to generate a history.
  * Entries are ordered first by measurement index, then by time, i.e. the state
  * sequence (0,a) -> (1,b) will generate the observation [0 1 a b].
  *
  * Furthermore, there are a number of parameters that can be set to achieve several
  * alternate behaviours:
  * \li \b integrate: if \e true, will integrate measurements rather than replacing
  *     previous ones with newer ones.
  * \li \b static: if \e true, causes the history to never be updated after the initial
  *     measurement of each episode, which makes it possible to remember initialization
  *     states. If in static mode, \b memory will always be 1, and integration will
  *     be disabled.
  *
  * @author Thomas Lampe
  * @ingroup OBSERVER */
class HistoryObserver : public Observer {
  public:
    HistoryObserver () {};
    ~HistoryObserver () {};
    bool init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname=0, const char *chapter=0);
    void get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state);

  protected:
    int _i, _k, _hdim, _mdim, _mem, _dim, _dims[HISTOBS_MAX_DIM];
    double *_history;
    bool _static, _add;
};

#endif
