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

#ifndef _CONDITIONAL_OBSERVER_H_
#define _CONDITIONAL_OBSERVER_H_

#include "observer.h"
#include <vector>
#include "setdef.h"

/** Observer class that picks specified state dimensions under certain conditions.
  * The list \b substitute describes the measurement indices to be used when the activation conditions are met,
  * \b default those used otherwise. The substitute assignment will be activated as soon as the state is within any
  * of the StateDef sets described by the config parameters \b activate_x, where \e x is a continuous index starting
  * at zero. The controller supports hysteresis; once the substitute has been activated, it will be used until any
  * condition described by the parameters \b release_x is met. If no release condition is specified, the default
  * measurements will be resumed as soon as the activation criteria are not met anymore. If activation and release
  * areas overlap, a change of choice is preferred, leading to an oscillatory behaviour.
  *
  * @author Thomas Lampe
  * @ingroup OBSERVER */
class ConditionalObserver : public Observer {
  public:
    ConditionalObserver () {};
    ~ConditionalObserver () {};
    bool init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname=0, const char *chapter=0);
    void get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state);
    inline void notify_episode_starts () {_choice=0;};

  protected:
    int _i, _mdim, _cdim, _choice, *_assign;
    std::vector<SetDef> _activate, _release;
};

#endif
