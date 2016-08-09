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

#include "jacobase.h"
#include "valueparser.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>

#define FINGER_OPEN 4
#define FINGER_CLOSED 60

#define SETSTRING(target,source) { target = new char[strlen(source)+1]; strncpy(target, source, strlen(source)); target[strlen(source)]='\0'; }
#define ABORT(xxx,yyy) {EOUT(xxx); yyy;}

// execute a mono method
#define INVOKEMETHOD(method, args, fail) \
  mono_runtime_invoke(method, NULL, args, exception); \
  if (exception) { \
    EOUT("Exception thrown by mono runtime when invoking Jaco" << #method); \
    if (!_continue_at_exception) fail; \
    exception = NULL; \
  }

// get data from the robot
#define GETDATA(target) \
  result = INVOKEMETHOD(_get, NULL, return false); \
  if (!result) ABORT("No data received.", return false); \
  for (_j=0; _j<_soff; _j++) \
    target[_j] = mono_array_get((MonoArray*)result, float, _j);

// send a position command to the robot
#define GOTOPOS(target, error) \
  for (_j=0; _j<9; _j++) { \
    _cmd[_j] = target[_j]; \
    _pargs[_j+1] = &_cmd[_j]; \
  } \
  _pargs[0] = &_cartesian; \
  INVOKEMETHOD(_pos, _pargs, error);

#define GOTOPOSSPEED(target, error) \
  for (_j=0; _j<12; _j++) { \
    _cmd[_j] = target[_j]; \
    _pargs[_j+1] = &_cmd[_j]; \
  } \
  _pargs[0] = &_cartesian; \
  INVOKEMETHOD(_posspeed, _pargs, error);

float JacoBasePlant_init[18] = {0.200487, -0.235204, 0.435699, -1.51857, 0.471082, -3.17681, 3.75, 3.75, 3.25, 282.515, 154.471, 43.7498, 230.081, 83.235, 77.791, 3.75, 3.75, 3.25};

bool JacoBasePlant::get_next_plant_state (const double *state, const double *action, double *next)
{
  if (_abort) return false;

  // if API control has been lost, there is no point in continuing
  if (state[15] < 0.5 && !_manual) {
    EOUT("API has no control over robot.");
    return false;
  }

  // link targets to mono buffer
  ///< \todo should check validity of target in the future, but the API doesn't offer the means at present
  for (_j=0; _j<_adim; _j++) {
    _cmd[_j] = action[_j];
    _cargs[_j+1] = &_cmd[_j];
  }
  _cargs[0] = &_cartesian;

  // modify command and send to robot
  switch (_amode) {

  case Direction:
  {
    // simulate direction by setting delta
    // IMPORTANT: can't just set movement limits, since endless turner joints may go wrong way then
    for (_j=0; _j<6; _j++)
      _cmd[_j] = state[_j+_offset] + (fabs(_cmd[_j])<=_dir_thresh ? 0. : _cmd[_j]<0. ? -_increment : _increment);
    for (_j=6; _j<9; _j++)
      _cmd[_j] = _cmd[_j]==0 ? state[_j+6] : _cmd[_j]<0. ? FINGER_OPEN : FINGER_CLOSED;
  }
  case Position:
  {
    // check if target within limits
    if (_chktarget)
      for (_j=0; _j<9; _j++) {
        if (_cmd[_j] < _hmin[_j]) _cmd[_j] = _hmin[_j];
        if (_cmd[_j] > _hmax[_j]) _cmd[_j] = _hmax[_j];
    }
    INVOKEMETHOD(_stop, NULL, return false);
    if (_adim == 12) {
      INVOKEMETHOD(_posspeed, _cargs, return false);
    } else {
      INVOKEMETHOD(_pos, _cargs, return false);
    }
    break;
  }
  case Joystick:
  {
    // if past limit, move no further
    if (_chktarget)
      for (_j=0; _j<9; _j++)
        if ((state[_j+(_j<6?_offset:6)]<_hmin[_j] && _cmd[_j]<0.) || (state[_j+(_j<6?_offset:6)]>_hmax[_j] && _cmd[_j]>0.)) _cmd[_j] = 0.;

    // decide whether to move fingers or TCP
    _fingers = true;
    for (_j=0; _j<6; _j++)
      _fingers &= _cmd[_j] == 0.;

    if (_fingers) {
      // simulate direction for fingers
      if (!_finger_warning) {
        WOUT(10, "Using fingers in joystick mode; may not function properly.");
        _finger_warning = true;
      }
      for (_j=6; _j<9; _j++)
        _cmd[_j] = _cmd[_j]==0 ? state[_j+6] : _cmd[_j]<0. ? FINGER_OPEN : FINGER_CLOSED;
      INVOKEMETHOD(_finger, &_cargs[7], return false);
    }
  }
  default:
    INVOKEMETHOD(_set, _cargs, return false);
  }

  // wait for data
  usleep(_delta_t);
  GETDATA(next);

  // calculate speed
  for (_j=0; _j<15; _j++)
    next[_j+_soff] = next[_j] - state[_j];

  return true;
}

bool JacoBasePlant::get_measurement (const double *state, double *measurement)
{
  for (_i=0; _i<_sdim; _i++)
    measurement[_i] = state[_i];
  return true;
}

#define MAXWAIT 100
bool JacoBasePlant::wait ()
{
  float *data = new float[_sdim];
  float prev[9] = {1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000.};
  bool there = false;
  int time = 0;
  while (!there && time < MAXWAIT) {
    time++;
    GETDATA(data);
    usleep(10*_delta_t);
    there = true;
    for (_j=0; _j<9; _j++) {
      there &= fabs(data[_j+6]-prev[_j]) < 1.;
      prev[_j] = data[_j+6];
    }
  }
  return time < MAXWAIT;
}

bool JacoBasePlant::check_initial_state (double *state)
{
  if (_reclaim_control) {
    INVOKEMETHOD(_reclaim, NULL, return false);
  }
  INVOKEMETHOD(_stop, NULL, return false);

  if (!_in_init_region) {
    double preinit[9], min, max;
    int set;
    IOUT("Performing initialization sequence...");
    for (_i=0; _i<(int)_preinit.size(); _i++) {

      // determine target position
      set = _episode % _preinit[_i].numSubsets();
      for (int j=0; j<9; j++) {
        min = _preinit[_i].subsets[set][j].min;
        max = _preinit[_i].subsets[set][j].max;
        if (min == min) {
          preinit[j] = min;
          if (min != max)
            min += (max - min) * ((float)rand() / RAND_MAX);
        } else if (_i==0) {
          EOUT("Undefined range in first target!");
          return false;
        }
      }

      GOTOPOS(preinit, return false);
      JacoBasePlant::wait();
      INVOKEMETHOD(_stop, NULL, return false);
    }
    _in_init_region = true;
    IOUT("Done!");
  }

  // override parts of inital position when specified
  for (_i=0; _i<6; _i++)
    if (state[_i+_offset] == state[_i+_offset])
      _init[_i] = state[_i+_offset];
  for (_i=0; _i<3; _i++)
    if (state[_i+12] == state[_i+12])
      _init[_i+6] = state[_i+12];

  // wait until not moving anymore
  GOTOPOS(_init, return false);
  stringstream ss;
  ss << "Moving to initial position [";
  for (_i=0; _i<6; _i++) ss << " " << state[_i];
  ss << " ]...";
  IOUT(ss.str());
  JacoBasePlant::wait();
  IOUT("Done!");

  // check if target and real position differ too much
  GETDATA(state);
  for (_i=0; _i<6; _i++)
    if (fabs(_init[_i]-state[_i+_offset]) > _wait_thresh[_i]) {
      EOUT("Could not reach initial position; DOF " << _i << " is " << state[_i+_offset] << ", should be " << _init[_i]);
      return false;
  }

  // initial speed is zero
  for (_j=0; _j<15; _j++)
    state[_j+_soff] = 0.;

  return true;
}

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
bool JacoBasePlant::init (int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  ValueParser vp(fname, chapter==0?"Plant":chapter);
  vp.get("reclaim", _reclaim_control, false);
  vp.get("manual_mode", _manual, false);
  vp.get("cartesian", _cartesian, true);
  _increment = _cartesian ? 1.0 : 90.0;

  vp.get("continue_at_exception", _continue_at_exception, false);
  if (_continue_at_exception)
    WOUT(10, "Will force continued program execution in event of an exception. Proceed with caution!");

  // CLS internals
  action_dim = 9;
  _soff = 37;
  state_dim = measurement_dim = _sdim = _soff + 15;
  vp.get("delta_t", _delta_t, 1000);
  delta_t = _delta_t;
  _episode++;

  // basic init pose
  int ioffset = _cartesian ? 0 : 9;
  for (_i=0; _i<9; _i++)
    _init[_i] = JacoBasePlant_init[_i+ioffset];

  // end-of-episode gripper opening
  _drop = false;
  vp.get("drop", _gpos, -1.);
  if (_gpos != -1) {
    _gargs[0] = &_gpos;
    _drop = true;
  }
  // maximum deviation of starting position
  for (_i=0; _i<6; _i++)
    _wait_thresh[_i] = _cartesian ? 0.02 : 1.;
  vp.get("wait_threshold", _wait_thresh, 6);

  // index of relevant state dimensions
  _offset = _cartesian ? 0 : 6;

  // action mode
  char amode[255];
  if (vp.get("action_mode", amode, 255) > 0) {
    if (strcmp(amode, "joystick") == 0) {
      _amode = Joystick;
    } else if (strcmp(amode, "position") == 0) {
      _amode = Position;
    } else if (strcmp(amode, "position_with_speed") == 0) {
      _amode = Position;
      action_dim += 3;
    } else if (strcmp(amode, "direction") == 0) {
      _amode = Direction;
      vp.get("dir_threshold", _dir_thresh, 0.01);
    } else if (strcmp(amode, "direction_with_speed") == 0) {
      _amode = Direction;
      action_dim += 3;
      vp.get("dir_threshold", _dir_thresh, 0.01);
    } else {
      EOUT("Invalid action mode \"" << amode << "\" specified.");
      return false;
    }
  } else {
    WOUT(10, "Parameter \"action_mode\" has not been set. Using legacy position mode instead.");
    bool pmode;
    vp.get("position_mode", pmode, false);
    _amode = pmode ? Position : Joystick;
  }

  // get movement limits
  _chktarget = vp.get("pos_min", _hmin, 9) > 8 && vp.get("pos_max", _hmax, 9) > 8;

  // setup
  _domain = mono_jit_init("Jaco.exe");
  std::stringstream ss;
  ss << TOSTRING(CLS_LIB_DIR) << "/Jaco.exe";
  MonoAssembly *assembly = mono_domain_assembly_open(_domain, ss.str().c_str());
  if (!assembly) EOUT("Failed to create assembly");
  MonoImage* image = mono_assembly_get_image(assembly);

  mono_config_parse(NULL);
  char *arg[10];
  SETSTRING(arg[0], "./Jaco.exe");

  // reinitialization sequence
  char target[255];
  for (_i=0; ; _i++) {
    ss.str("");
    ss << "precheck_" << _i;
    if (vp.get(ss.str().c_str(), target, 255) < 1) break;
    SetDef set;
    if (!set.parseStr(target, 9)) {
      EOUT("Invalid set definition.");
      return false;
    }
    _preinit.push_back(set);
  }

  // initialization help (legacy)
  if (_preinit.size() < 1) {
    float pos[9];
    for (_i=0; ; _i++) {
      ss.str("");
      ss << "preinit_" << _i;
      if (vp.get(ss.str().c_str(), pos, 9) < 9) break;
      SetDef set;
      set.subsets.push_back(SetDef::Bounds());
      for (int j=0; j<9; j++) {
        SetDef::MinMax mm;
        mm.min = mm.max = pos[j];
        set.subsets[0].push_back(mm);
      }
      _preinit.push_back(set);
    }
  }

  IOUT("Found " << _preinit.size() << " waypoints for initialization.");

  // protection zones
  char profile[255];
  if (vp.get("profile", profile, 255) > 0) {
    SETSTRING(arg[1], "zone");
    SETSTRING(arg[2], "load");
    SETSTRING(arg[3], profile);
    mono_jit_exec(_domain, assembly, 4, arg);
    std::cout << "Reboot the robot and move it to its home position using the remote, then press enter." << std::endl;
    std::cout << "Alternatively press enter without rebooting to keep using the previous zone configuration." << std::endl;
    std::cin.ignore();
  }

  // run main in dll mode
  SETSTRING(arg[1], "dll");
  mono_jit_exec(_domain, assembly, 2, arg);

  // main oddly seems to return before all commands are finished;
  // therefore we need to wait, otherwise might override the mode of the first command
  usleep(1000000);

  // prepare mono methods
  MonoMethodDesc *desc;
#define GETMETHOD(method, handle) \
  desc = mono_method_desc_new(method, true); \
  if (!desc) ABORT("Failed to create description for " << method, return false); \
  handle = mono_method_desc_search_in_image(desc, image); \
  if (!handle) ABORT("Failed to find method for " << method, return false);
  _mono_ready = true;
  GETMETHOD("Jaco:Stop", _stop);
  GETMETHOD("Jaco:Deinit", _deinit);
  GETMETHOD("Jaco:SetPosition", _pos);
  GETMETHOD("Jaco:SetPositionWithSpeed", _posspeed);
  GETMETHOD("Jaco:SetDirection", _set);
  GETMETHOD("Jaco:GetData", _get);
  GETMETHOD("Jaco:SetGripper", _grip);
  GETMETHOD("Jaco:SetFingers", _finger);
  GETMETHOD("Jaco:GetControl", _reclaim);

  // check if everything okay
  exception = NULL;
  mono_runtime_invoke(_stop, NULL, NULL, exception);
  if (exception) {
    WOUT(10, "Exception thrown by mono runtime during first invocation. Will retry once."); 
    exception = NULL;
  }
  INVOKEMETHOD(_stop, NULL, return false);
  if (!_continue_at_exception)
    IOUT("Mono method invocation seems to work okay.");

  _adim = action_dim;
  _finger_warning = false;
  _pause = 0;
  return true;
}

void JacoBasePlant::notify_episode_stops ()
{
  INVOKEMETHOD(_stop, NULL, );
  if (_drop) {
    INVOKEMETHOD(_grip, _gargs, );
    sleep(2);
  }
  _in_init_region = false;

  if (_pause > 0) {
    IOUT("Sleeping for " << _pause << " seconds...");
    sleep(_pause);
    _pause = 0;
  }

  _episode++;
}

void JacoBasePlant::notify_episode_starts ()
{
  _abort = false;
}

void JacoBasePlant::deinit()
{
  if (!_mono_ready) return;
  INVOKEMETHOD(_deinit, NULL, );
  mono_jit_cleanup(_domain);
}

void JacoBasePlant::notify_command_string (const char* buf)
{
  if (strcmp(buf, "plant_cmd pause") == 0) {
    INVOKEMETHOD(_stop, NULL, );
    WOUT(10, "Never pause a dynamic plant! Will abort current episode to prevent faulty data.");
    _abort = true;
  } else if (strncmp(buf, "plant_cmd wait", 14) == 0) {
    _pause = atoi(&buf[15]);
    IOUT("Will wait for " << _pause << " seconds before beginning next episode.");
  }
}

JacoBasePlant::JacoBasePlant ()
{
  _in_init_region = false;
}

JacoBasePlant::~JacoBasePlant ()
{
}

REGISTER_PLANT(JacoBasePlant, "General-purpose plant for a Kinova Jaco robotic arm.");
