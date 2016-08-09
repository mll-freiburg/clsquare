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

#ifndef _FILTER_COLLECTION_OBSERVER_H_
#define _FILTER_COLLECTION_OBSERVER_H_

#include "observer.h"
#include "filter.h"
#include <vector>

/** Applies a variety of DSP filters to the observation.
  * Available filters are ones from the FilterCollection
  * utility group. The filter to be used is set by the
  * parameter \b mode, with options being:
  * \li highpass: FilterCollection::HighpassFilter
  * \li lowpass: FilterCollection::LowpassFilter
  * \li semipass: FilterCollection::SemipassFilter
  * \li outlier: FilterCollection::OutlierFilter
  * \li average: FilterCollection::AveragerFilter
  * \li hysteresis_lock: FilterCollection::HysteresisLockFilter
  * \li change: FilterCollection::ChangeFilter
  *
  * The observation dimensions to apply the filter to are
  * specified by the list \b inputs. If the parameter
  * \b append is set to \e true, the resulting filtered
  * values are added to the end of the observation vector;
  * otherwise the original values are overwritten.
  *
  * Lastly, the parameter \b initial can be specified
  * to call the filter for a certain number of times
  * when the first input value is being received,
  * which can be useful to stabilize slow-acting
  * filters.
  *
  * \todo Does not reset filters on new episode.
  *
  * @author Thomas Lampe
  * @ingroup OBSERVER */
class FilterObserver : public Observer {
  public:
    FilterObserver () {};
    ~FilterObserver () {};
    bool init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname=0, const char *chapter=0);
    void get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state);

  protected:
    std::vector<FilterCollection::Filter*> _filters;
    int _i, _mdim, _cdim, *_inputs, _initial;
    bool _append;
};

#endif
