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

#include "chainobserver.h"
#include "valueparser.h"
#include <stdio.h>
#include <cstdlib>
#include <cstring>

#define SSSET(xxx) { ss.str(""); ss << xxx; };

bool ChainObserver::init (const int plant_state_dim, const int measurement_dim, const int action_dim, int &observed_state_dim, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==NULL ? "Observer" : chapter);
  vp.get("verbose", _verb, false);
  std::stringstream ss;
  observed_state_dim = measurement_dim;

  int len, space, maxdim=measurement_dim;
  for (_i=0; ; _i++) {
    ChainObserverSlave slave;

    // try to get next controller name
    slave.name = new char[255];
    SSSET("observer_" << _i);
    len = vp.get(ss.str().c_str(), slave.name, 255);
    if (len < 1) break;

    // if there's a space, treat next segment as chapter
    space = -1;
    for (_k=0; _k<len; _k++)
      if (slave.name[_k] == ' ') {
        space = _k;
        slave.name[_k] = '\0';
    }

    // create observer
    slave.base = ObserverFactory::getTheObserverFactory()->create(slave.name);
    if (slave.base == NULL) {
      EOUT("ChainObserver could not create an observer of type " << slave.name << "!");
      return false;
    }

    // determine config chapter
    if (space < 0) {
      SSSET("Observer_" << _i);
      slave.chapter = new char[ss.str().size() + 1];
      std::strcpy(slave.chapter, ss.str().c_str());
    } else {
      slave.chapter = &slave.name[space+1];
    }

    // initialize module
    slave.mdim = observed_state_dim;
    slave.prev = new double[slave.mdim];
    for (_k=0; _k<slave.mdim; _k++) slave.prev[_k] = 0.;
    if (!slave.base->init(plant_state_dim, slave.mdim, action_dim, slave.odim, fname, slave.chapter) || slave.odim < 1) {
      EOUT("ChainObserver failed to initialize observer " << slave.name << " !");
      return false;
    }
    if (slave.odim > maxdim) maxdim = slave.odim;
    IOUT("Initializing observer " << slave.name << " in chapter [" << slave.chapter << "] changed observation dimensionality by " << (slave.odim-slave.mdim) << ".");
    _chain.push_back(slave);
    observed_state_dim = slave.odim;
  }

  _meas_dim = measurement_dim;
  _obs_dim = observed_state_dim;
  _mbuf = new double[maxdim];
  _obuf = new double[maxdim];
  IOUT("Total observation dimensionality: " << observed_state_dim);
  return true;
}

void ChainObserver::get_observed_state (const double *prev_measurement, const double* prev_action, const double *current_measurement, const int cycle_ctr, double *observed_state)
{
  if (_verb) {
    std::cout << "Measurement: ";
    for (_i=0; _i<_meas_dim; _i++) std::cout << current_measurement[_i] << " ";
    std::cout << std::endl;
  }

  // if no slaves present, pass measurement
  if (_chain.size() < 1) {
    for (_i=0; _i<_meas_dim; _i++) observed_state[_i] = current_measurement[_i];
    return;
  }

  // first slave will get plant measurement
  for (_k=0; _k<_meas_dim; _k++)
    _mbuf[_k] = current_measurement[_k];

  // pass measurement to slave
  for (_i=0; _i<(int)_chain.size(); _i++) {

    // need to set prev properly first time
    if (cycle_ctr == 0)
    for (_k=0; _k<_chain[_i].mdim; _k++)
      _chain[_i].prev[_k] = _mbuf[_k];

    _chain[_i].base->get_observed_state(_chain[_i].prev, prev_action, _mbuf, cycle_ctr, _obuf);

    // save slave's measurement for next cycle
    for (_k=0; _k<_chain[_i].mdim; _k++)
      _chain[_i].prev[_k] = _mbuf[_k];

    // this slave's observation is the next slave's measurement
    for (_k=0; _k<_chain[_i].odim; _k++)
      _mbuf[_k] = _obuf[_k];
  }

  // last output is the one to be passed on
  for (_k=0; _k<_obs_dim; _k++)
    observed_state[_k] = _obuf[_k];

  if (_verb) {
    std::cout << "Observation: ";
    for (_i=0; _i<_obs_dim; _i++) std::cout << observed_state[_i] << " ";
    std::cout << std::endl;
  }
}

void ChainObserver::notify_episode_starts ()
{
  for (_i=0; _i<(int)_chain.size(); _i++) _chain[_i].base->notify_episode_starts();
}

void ChainObserver::notify_episode_stops ()
{
  for (_i=0; _i<(int)_chain.size(); _i++) _chain[_i].base->notify_episode_stops();
}

void ChainObserver::deinit ()
{
  for (_i=0; _i<(int)_chain.size(); _i++) _chain[_i].base->deinit();
}

REGISTER_OBSERVER(ChainObserver, "Observer to combine different sub-observers.");

