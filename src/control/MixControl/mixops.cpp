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

#include "mixops.h"
#include "valueparser.h"
#include <cstdio>
#include <cmath>
#include <cstring>

#define INITCHECK { if (!_init) {EOUT("Slave controller references not initialized!"); return false;} };
#define SSSET(xxx) { ss.str(""); ss << xxx; };

/* *********** Conditional ************* */

bool MixControlCondition::get_action (const double* state, double* action)
{
  INITCHECK;

  // automatically release
  if (_default_release)
    _choice = 0;
  for (_i=0; (unsigned int)_i<_activate.size(); _i++)
    if (_activate[_i].isWithinSet(state, _sdim))
      _choice = 1;
  if (_choice == 1 && !_never_release)
    for (_i=0; (unsigned int)_i<_release.size(); _i++)
      if (_release[_i].isWithinSet(state, _sdim))
        _choice = 0;

  for (_i=0; _i<_adim; _i++)
    action[_i] = _slaves->at(_cons[_choice]).action[_acts[_choice][_i]];

  return _slaves->at(_cons[_choice]).success;
}

bool MixControlCondition::init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, int own_id)
{
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);
  std::stringstream ss;

  _i = 0;
  while (true) {
    char *tmp = new char[255];
    SSSET("activate_" << _i);
    if (vp.get(ss.str().c_str(), tmp, 255) < 1) break;

    SetDef set;
    if (!set.parseStr(tmp, _sdim)) break;

    SSSET("Activation condition " << _i << ": " << tmp);
    IOUT(ss.str());
    _activate.push_back(set);
    _i++;
  }

  if (_activate.size() < 1)
    WOUT(10, "No activation conditions specified.");

  _i = 0;

  vp.get("never_release", _never_release, false);
  _default_release = false;
  if (_never_release) {
    IOUT("Will never release secondary controller.");
  } else {
    while (true) {
      char *tmp = new char[255];
      SSSET("release_" << _i);
      if (vp.get(ss.str().c_str(), tmp, 255) < 1) break;

      SetDef set;
      if (!set.parseStr(tmp, _sdim)) break;

      SSSET("Release condition " << _i << ": " << tmp);

      IOUT(ss.str());
      _release.push_back(set);
      _i++;
    }

    if (_release.size() < 1) {
      IOUT("No release conditions specified. Will release subsitute action as soon as activation criteria are not met anymore.");
      _default_release = true;
    }
  }

  return MixControlOperation::check_params(slaves, own_id);
}

REGISTER_CONTROLLER(MixControlCondition, "MixControl operator for choosing different controllers in different states.")

/* *********** Arithmetics ************* */

bool MixControlCombine::get_action (const double* state, double* action)
{
  INITCHECK;
  for (_i=0; _i<_adim; _i++) {
    if (_mode[1] < 0. && _slaves->at(_cons[1]).action[_acts[1][_i]] == 0.) {
      EOUT("Division by zero.");
      return false;
    }
    if (_pow)
      action[_i] = pow(_slaves->at(_cons[0]).action[_acts[0][_i]], _slaves->at(_cons[1]).action[_acts[1][_i]]);
    else
      action[_i] = _slaves->at(_cons[0]).action[_acts[0][_i]] * pow(_slaves->at(_cons[1]).action[_acts[1][_i]], _mode[1]) + _mode[0] * _slaves->at(_cons[1]).action[_acts[1][_i]];
  }
  return true;
}

bool MixControlCombine::init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, int own_id)
{
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);

  char op;
  vp.get("operator", op, ' ');
  _mode[0] = 0.;
  _mode[1] = 0.;
  _pow = false;
  switch (op) {
    case '+':
      _mode[0] = 1.;
      break;
    case '-':
      _mode[0] = -1.;
      break;
    case '*':
      _mode[1] = 1.;
      break;
    case '/':
      _mode[1] = -1.;
      break;
    case '^':
      _pow = true;
      break;
    default:
      EOUT("Unknown combination operator: '" << op << "'");
      return false;
  }

  std::stringstream ss;
  ss << "MixControlCombine assignment:";
  for (_i=0; _i<_adim; _i++)
    ss << " [" << _cons[0] << "][" << _acts[0][_i] << "]" << op << "[" << _cons[1] << "][" << _acts[1][_i] << "] ";
  IOUT(ss.str());

  return MixControlOperation::check_params(slaves, own_id);
}

REGISTER_CONTROLLER(MixControlCombine, "MixControl operator to perform basic arithmetic operations on two actions.")

/* *********** Weighted ************* */

bool MixControlWeight::get_action (const double* state, double* action)
{
  INITCHECK;
  for (_i=0; _i<_adim; _i++)
    action[_i] = _slaves->at(_cons[0]).action[_acts[0][_i]] * _factor[_i] + _slaves->at(_cons[1]).action[_acts[1][_i]] * (1. - _factor[_i]);

  return true;
}

bool MixControlWeight::init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, int own_id)
{
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);

  _factor = new double[_adim];
  for (_i=0; _i<_adim; _i++) _factor[_i] = 0.5;
  vp.get("factors", _factor, _adim);

  std::stringstream ss;
  ss << "MixControlWeight assignment:";
  for (_i=0; _i<_adim; _i++)
    ss << " " << _factor[_i];
  IOUT(ss.str());

  return MixControlOperation::check_params(slaves, own_id);
}

REGISTER_CONTROLLER(MixControlWeight, "MixControl operator to compute a weighted sum of two actions.")

/* *********** Pipe ************* */

bool MixControlPipe::get_action (const double* state, double* action)
{
  INITCHECK;
  if (_current == -1) return false;
  for (_i=0; _i<_adim; _i++)
    action[_i] = _slaves->at(_cons[_current]).action[_acts[_current][_i]];

  return true;
}

bool MixControlPipe::init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, int own_id)
{
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);

  char param[64], command[255];
  for (_i=0; _i<_slavenum; _i++) {
    sprintf(param, "command_%d", _i);
    if (vp.get(param, command, 255) < 1) {
      EOUT("No pipe command specified for controller " << _i);
      return false;
    }
    _commands.push_back(std::string(command));
  }

  return MixControlOperation::check_params(slaves, own_id);
}

void MixControlPipe::notify_episode_starts ()
{
  _current = 0;
}

void MixControlPipe::notify_command_string (const char* buf)
{
  for (_i=0; _i<_slavenum; _i++)
    if (strncmp(_commands.at(_i).c_str(), buf, _commands.at(_i).size()) == 0) {
      _current = _i;
      IOUT("Will switch to controller #" << _i << " next cycle.");
      return;
  }
  IOUT("Unknown command \"" << buf << "\"; will abort episode.");
  _current = -1;
  return;
}

REGISTER_CONTROLLER(MixControlPipe, "MixControl operator to select a subcontroller based on pipe input.")

/* *********** Toggle ************* */

bool MixControlToggle::get_action (const double* state, double* action)
{
  INITCHECK;

  if (!_toggled) {
    if (_tset.isWithinSet(state, _sdim)) {
      _toggled = true;
      _current++;
      if (_current < _slavenum || _rewind) {
        _current %= _slavenum;
        IOUT("Switching to controller " << _current);
      } else {
        IOUT("Finished last controller.");
        return false;
      }
    }
  } else {
    if (!_tset.isWithinSet(state, _sdim))
      _toggled = false;
  }

  for (_i=0; _i<_adim; _i++)
    action[_i] = _slaves->at(_cons[_current]).action[_acts[_current][_i]];

  return true;
}

bool MixControlToggle::init (const std::vector<MixControlSlave> *slaves, const char* fname, const char* chapter, int own_id)
{
  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);

  char temp[1024];
  if (vp.get("condition", temp, 1024) < 1) {
    EOUT("No toggling condition specified.");
    return false;
  }
  _tset.parseStr(temp, _sdim);

  vp.get("rewind", _rewind, false);

  return MixControlOperation::check_params(slaves, own_id);
}

void MixControlToggle::notify_episode_starts ()
{
  _current = 0;
  _toggled = false;
}

REGISTER_CONTROLLER(MixControlToggle, "MixControl operator to toggle through a list of subcontrollers whan a condition is met.")


