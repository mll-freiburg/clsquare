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

#include "mixcontrol.h"
#include "valueparser.h"
#include "registry.h"
#include <stdio.h>
#include <cmath>
#include <cstring>

#define SSSET(xxx) { ss.str(""); ss << xxx; };

#define MAX_NUM_SLAVES 20

bool MixControl::get_action(const double *state, double *action)
{
  // create reduced temporary state and generate action
  for (_i=0; _i<_slaves.size(); _i++) {
    _slaves[_i].reduce_state(state);
    _slaves[_i].success = _slaves[_i].base->get_action(_slaves[_i].state, _slaves[_i].action);
  }

  // map controller action to composite action
  for (_i=0; _i<_assignment.size(); _i++) {

    // check if any alternative applies
    for (_k=1; _k<_assignment[_i].size(); _k++)
      if (_assignment[_i][_k].condition.isWithinSet(state, _state_dim))
        break;

    // use default
    if (_k >= _assignment[_i].size())
      _k = 0;

    if (!_slaves[_assignment[_i][_k].controller].success) {
      //IOUT("Slave controller " << _assignment[_i][_k].controller << " (" << _slaves[_assignment[_i][_k].controller].name << " in [" << _slaves[_assignment[_i][_k].controller].chapter << "]) failed to generate an action.");
      return false;
    }

    // assign action
    action[_i] = _assignment[_i][_k].multi * _slaves[_assignment[_i][_k].controller].action[_assignment[_i][_k].action] + _assignment[_i][_k].add;
  }

	return true;
}

bool MixControl::init (const int observed_state_dim, const int action_dim, double delta_t, const char *fname, const char* chapter)
{
  if (chapter==NULL) chapter = "Controller";
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);
  _state_dim = observed_state_dim;
  std::stringstream ss;

  for (_i=0; ; _i++) {
    MixControlSlave slave;

    // try to get next controller name
    int len, space = -1;
    slave.name = new char[255];
    SSSET("controller_" << _i);
    len = vp.get(ss.str().c_str(), slave.name, 255);
    if (len < 0) break;

    // if there's a space, treat next segment as chapter
    for (_k=0; _k<(unsigned int)len-1; _k++)
      if (slave.name[_k] == ' ') {
        space = _k;
        slave.name[_k] = '\0';
    }

    // create controller
    slave.base = ControllerFactory::getTheControllerFactory()->create(slave.name);
    if (slave.base == NULL) {
      EOUT("MixControl could not create a controller of type " << slave.name << "!");
      return false;
    }

    // check for different action dimension
    SSSET("action_dim_" << _i);
    vp.get(ss.str().c_str(), slave.adim, action_dim);
    if (slave.adim < 1) {
      EOUT("Controller action dimensionality must be positive, is " << slave.adim);
      return false;
    }
    slave.action = new double[slave.adim];

    // check for different state assignment
    SSSET("state_dim_" << _i);
    vp.get(ss.str().c_str(), slave.sdim, observed_state_dim);
    if (slave.sdim < 1) {
      EOUT("Controller state dimensionality must be positive, is " << slave.sdim);
      return false;
    }
    slave.state = new double[slave.sdim];
    if (slave.sdim > observed_state_dim) {
      EOUT("Slave dimension " << slave.sdim << " exceeds observation dimension " << observed_state_dim << "!");
      return false;
    }
    slave.input = new int[slave.sdim];
    for (_k=0; (int)_k<slave.sdim; _k++)
      slave.input[_k] = _k;
    SSSET("state_assign_" << _i);
    vp.get(ss.str().c_str(), slave.input, slave.sdim);

    // determine config chapter
    SSSET("Controller_" << _i);
    if (space < 0) {
      slave.chapter = new char[ss.str().size() + 1];
      std::strcpy(slave.chapter, ss.str().c_str());
    } else {
      slave.chapter = &slave.name[space+1];
    }

    // try to initialize controller
    SSSET("Initializing slave controller " << slave.name << " in chapter [" << slave.chapter << "] with action_dim=" << slave.adim << " and state dimensions [" << slave.input[0]);
    for (_k=1; (int)_k<slave.sdim; _k++)
      ss << " " << slave.input[_k];
    ss << "]";
    if (!slave.base->init(slave.sdim, slave.adim, delta_t, fname, slave.chapter)) {
      EOUT("MixControl failed to initialize controller " << slave.name << " !");
      return false;
    } else
      IOUT(ss.str());

    // if controller is MixControl specific, initialize second stage
    if (strstr(slave.name, "MixControl")!=0 && strcmp(slave.name, "MixControl")!=0) {
      if (!((MixControlOperation*)(slave.base))->init(&_slaves, fname, slave.chapter, _slaves.size())) {
        EOUT("MixControl failed to initialize operator " << slave.name << " !");
        return false;
      }
    }
    _slaves.push_back(slave);
  }

  // get assignment
  char *tmp_ass = new char[255];
  if (vp.get("assignment", tmp_ass, 255) < 0) {
    EOUT("Missing parameter " << chapter << "::assignment!");
    return false;
  }
  SetDef ass;
  ass.parseStr(tmp_ass, action_dim);

  // get multipliers
  double tmp_mod[2*action_dim];
  for (_i=0; _i<(unsigned int)action_dim; _i++) {
    tmp_mod[_i] = 1.;
    tmp_mod[_i+action_dim] = 0.;
  }
  vp.get("multi", &tmp_mod[0], action_dim);
  vp.get("add", &tmp_mod[action_dim], action_dim);

  // build assignment
  IOUT("MixControl action assignment:");
  for (_i=0; _i<(unsigned int)action_dim; _i++) {

    // cope data
    MixControlAction action;
    action.multi = tmp_mod[_i];
    action.add = tmp_mod[_i+action_dim];
    action.controller = ass.subsets[0][_i].min;
    action.action = ass.subsets[0][_i].max;

    // check sanity
    if (action.controller < 0 || action.controller >= (int)_slaves.size()) {
      EOUT("No controller[" << action.controller << "] (action dim " << _i << ")");
      return false;
    }
    if (action.action < 0 || action.action >= _slaves[action.controller].adim) {
      EOUT("Controller " << action.controller << " does not have an action[" << action.action << "]");
      return false;
    }
    
    std::vector<MixControlAction> tmpv;
    tmpv.push_back(action);
    IOUT("-- Output " << _i << ": action " << action.action << " of controller " << action.controller << " (" << _slaves[action.controller].name << ", configured in section [" << _slaves[action.controller].chapter << "])");

    // check for alternatives
    double tmpd[4];
    for (_k=0; ; _k++) {

      // try to get next alternative
      SSSET("action_" << _i << "_" << _k << "_params");
      if (vp.get(ss.str().c_str(), tmpd, 4) != 4)
        break;

      // set parameters
      MixControlAction tmpa;
      tmpa.controller = tmpd[0];
      tmpa.action = tmpd[1];
      tmpa.multi = tmpd[2];
      tmpa.add = tmpd[3];

      // parse condition
      tmp_ass = new char[255];
      SSSET("action_" << _i << "_" << _k << "_condition");
      vp.get(ss.str().c_str(), tmp_ass, 255);
      if (!tmpa.condition.parseStr(tmp_ass, observed_state_dim))
        break;

      IOUT("-- Output " << _i << ": action " << tmpa.action << " of controller " << tmpa.controller << " (" << _slaves[tmpa.controller].name << ", configured in section [" << _slaves[tmpa.controller].chapter << "]) in state " << tmp_ass);
      tmpv.push_back(tmpa);
    }

    if (tmpv.size() > 1)
      WOUT(10, "Using deprecated inline conditions in MixControl. Please use controller Type MixControlCondition instead.");

    _assignment.push_back(tmpv);
  }

  if ((int)_assignment.size() != action_dim) {
    EOUT("Assignment size " << _assignment.size() << " does not match action dimension " << action_dim << "!");
    return false;
  }

  // prepare observation buffer
  int smax = observed_state_dim;
  for (_i=0; _i<_slaves.size(); _i++)
    if (_slaves[_i].sdim > smax) smax = _slaves[_i].sdim;
  _tmp_observation = new double[smax];

	return true;
}

void MixControl::deinit ()
{
  for (_i=0; _i<_slaves.size(); _i++)
    _slaves[_i].base->deinit();
}

void MixControl::notify_episode_starts ()
{
  for (_i=0; _i<_slaves.size(); _i++)
    _slaves[_i].base->notify_episode_starts();
}

void MixControl::notify_episode_stops (const double* current_observed_state)
{
  for (_i=0; _i<_slaves.size(); _i++) {
    _slaves[_i].reduce_state(current_observed_state);
    _slaves[_i].base->notify_episode_stops(_slaves[_i].state);
  }
}

void MixControl::notify_command_string (const char* buf)
{
  for (_i=0; _i<_slaves.size(); _i++)
    _slaves[_i].base->notify_command_string(buf);
}

void MixControl::notify_transition (const double* observed_state, const double* action, const double* next_observed_state, const double reward, const bool is_terminal_state, const double terminal_reward)
{
  for (_i=0; _i<_slaves.size(); _i++) {

    // reduce old observation and copy, since only one buffer
    _slaves[_i].reduce_state(observed_state);
    for (_j=0; _j<(unsigned int)_slaves[_i].sdim; _j++)
      _tmp_observation[_j] = _slaves[_i].state[_j];

    // reduce new observation
    _slaves[_i].reduce_state(next_observed_state);

    // now we have to trust that it still remembers its last action, since we may have dropped parts during the assignment,
    // so reconstruction may be impossible; should be reasonable, since next get_next_state() will be called only after
    // this function, and slave.action is not modified anywhere else
    // could only be a problem if we are called by another metacontroller that uses notify_transition() to read old data
    _slaves[_i].base->notify_transition(_tmp_observation, _slaves[_i].action, _slaves[_i].state, reward, is_terminal_state, terminal_reward);
  }
}

bool MixControl::check_initial_state (const double* initial_observed_state, const int observation_dim)
{
  bool success = true;
  for (_i=0; _i<_slaves.size(); _i++) {
    _slaves[_i].reduce_state(initial_observed_state);
    success &= _slaves[_i].base->check_initial_state(_slaves[_i].state, _slaves[_i].sdim);
  }
  return success;
}

REGISTER_CONTROLLER(MixControl, "Metacontroller which generates an action from several sub-controllers.")

bool MixControlOperation::init (const int observed_state_dim, const int action_dim, double deltat, const char* fname, const char* chapter)
{
  _init = false;
  _adim = action_dim;
  _sdim = observed_state_dim;

  bool variable = _slavenum < 1;
  if (variable) _slavenum = MAX_NUM_SLAVES;

  _acts = new int*[_slavenum];
  _cons = new int[_slavenum];

  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);
  std::stringstream ss;

  for (_i=0; _i<_slavenum; _i++) {
    SSSET("controller_" << _i);
    vp.get(ss.str().c_str(), _cons[_i], -1);
    if (_cons[_i] < 0) {
      if (variable) {
        _slavenum = _i;
        return true;
      } else {
        EOUT("Slave controller " << _i << " not specified (" << _cons[_i] << ") in chapter [" << chapter << "]");
        return false;
      }
    }

    _acts[_i] = new int[_adim];
    for (int k=0; k<_adim; k++)
      _acts[_i][k] = k;
    SSSET("actions_" << _i);
    vp.get(ss.str().c_str(), _acts[_i], _adim);
  }

  return true;
}

bool MixControlOperation::check_params (const std::vector<MixControlSlave> *slaves, const int id)
{
  for (_i=0; _i<_slavenum; _i++) {
    if (_cons[_i] >= id) {
      EOUT("Slave controller ID " << _cons[_i] << " surpasses operator ID " << id << ".");
      return false;
    }
  }

  _slaves = slaves;
  _init = true;

  return true;
}
