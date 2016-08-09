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

#include "jacohead.h"
#include "valueparser.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#define RNGCRP(xxx,yyy,zzz) if (xxx < yyy) xxx = yyy; if (xxx > zzz) xxx = zzz;

bool JacoFacePlant::get_next_plant_state (const double *state, const double *action, double *next)
{
  gettimeofday(&_starttime, NULL);

  // calculate pos and speed from action
  if (_hmode == PWM) {
    for (_i=0; _i<_servos; _i++) {
      _hpos[_i] = int(1000 + action);
      RNGCRP(_hpos[_i],0,2000);
      if (state[_i+_visionplant.sdim] < _smin[_i] || state[_i+_visionplant.sdim] > _smax[_i])
        _hpos[_i] = 1000;
    }
    _base->set_param(_servos, _id, Morpheus::PWM, _hpos);
  } else {
    for (_i=0; _i<_servos; _i++) {
      parse_action(action[_i+_visionplant.adim], state[_i+_visionplant.sdim], _hpos[_i], _hspeed[_i]);
      RNGCRP(_hpos[_i],_smin[_i],_smax[_i]);
    }
    _base->set_pos(_servos, _id, _hpos, _hspeed);
  }

  // communicate with Jaco
  if (!JacoVisionPlant::get_next_plant_state(state, action, next)) return false;

  // sleep rest of cycle
  gettimeofday(&_stoptime, NULL);
  _remaining = 1000 * _delta_s - (1000000 * (_stoptime.tv_sec - _starttime.tv_sec) + (_stoptime.tv_usec - _starttime.tv_usec));
  if (_remaining > 0)
    usleep(_remaining);
  else
    WOUT(10, "Maximum cycle duration exceeded by " << -_remaining << "ms!");

  // determine servo positions
  _base->get_pos(_servos, _id, _hpos, _hspeed);
  for (_i=0; _i<_servos; _i++)
    next[_i+_visionplant.sdim] = _hpos[_i] < 1024 ? _hpos[_i] : state[_i];

  return true;
}

void JacoFacePlant::parse_action (const double action, const double state, int& pos, int& speed)
{
  pos = state;
  speed = 0;
  switch (_hmode) {
    case PWM:
      break;
    case Position:
      _tmp = action;
      RNGCRP(_tmp,0,1023);
      pos = _tmp;
      speed = _hsdef;
      break;
    case Increment:
      _tmp = action;
      if (fabs(_tmp)<3) _tmp = 0.;
      pos = state + _tmp;
      RNGCRP(pos,-1023,1023);
      speed = _hsdef;
      break;
    case Direction:
      _tmp = action;
      RNGCRP(_tmp,-1.,1.);
      pos = _tmp > 0. ? 1023 : 0;
      speed = fabs(_tmp * _hsdef);
      // since we use Morpheus firmware, avoid speed=0
      if (speed == 0) {
        pos = state;
        speed = 1;
      }
      break;
    default:
      EOUT("Unknown command mode!");
  }
}

bool JacoFacePlant::get_measurement (const double *state, double *measurement)
{
  if (!JacoVisionPlant::get_measurement(state, measurement)) return false;
  for (_i=0; _i<_servos; _i++)
    measurement[_visionplant.mdim+_i] = state[_visionplant.sdim+_i];
  return true;
}

bool JacoFacePlant::check_initial_state (double *state)
{
  // move to target position
  int initpos[_servos];
  int initspeed[4] = {5,5,5,5};
  for (_i=0; _i<_servos; _i++) {
    if (state[_i+_visionplant.sdim] != state[_i+_visionplant.sdim]) {
      EOUT("Incomplete servo position definition.");
      return false;
    }
    initpos[_i] = state[_i+_visionplant.sdim];
  }

  // wait to reach position
  bool there = false;
  for (int t=10; t>0 && !there; t--) {
    _base->set_pos(_servos, _id, initpos, initspeed);
    there = true;
    _base->get_pos(_servos, _id, _hpos, _hspeed);
    for (_i=0; _i<_servos; _i++)
      there &= abs(_hpos[_i]-initpos[_i]) < _htolerance;
    usleep(500000);
  }
  if (!there) {
    EOUT("Could not reach initial servo position, rejecting as initial state.");
    return false;
  }

  // have Jaco move second, since it checks the camera
  if (!JacoVisionPlant::check_initial_state(state)) return false;

  // sleep a little more to avoid swinging from init
  //sleep(1);

  return true;
}

bool JacoFacePlant::init (int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  if (!JacoVisionPlant::init(state_dim, measurement_dim, action_dim, delta_t, fname, chapter)) return false;
  _visionplant.sdim = state_dim;
  _visionplant.adim = action_dim;
  _visionplant.mdim = measurement_dim;

  ValueParser vp(fname, chapter==0?"Plant":chapter);
  vp.get("servo_delta", _delta_s, 10);
  vp.get("servo_tolerance", _htolerance, 10);

  // command modes
  char mode[255];
  vp.get("servo_mode", mode, 255);
  _hmode = Increment;
  if (strcmp(mode, "pwm") == 0)
    _hmode = PWM;
  else if (strcmp(mode, "position") == 0)
    _hmode = Position;
  else if (strcmp(mode, "direction") == 0)
    _hmode = Direction;
  else if (strcmp(mode, "incremental") != 0) {
    EOUT("No valid command mode specified (" << mode << ")!");
    return false;
  }

  // servo ids
  int tmp[255];
  _servos = vp.get("servo_ids", tmp, 255);
  _id = new unsigned char[_servos];
  for (_i=0; _i<_servos; _i++)
    _id[_i] = tmp[_i];
  _hpos = new int[_servos];
  _hspeed = new int[_servos];

  // movement limits
#define GET_LIMITS(_dir_) \
  _s##_dir_ = new int[_servos]; \
  if (vp.get("servo_"#_dir_, _s##_dir_, _servos) < _servos) { \
    EOUT("Insufficient number of "#_dir_"imal movement limits specified."); \
    return false; \
  }
  GET_LIMITS(min);
  GET_LIMITS(max);

  // default speed
  vp.get("servo_speed", _hsdef, 10);

  // determine protocol
  long int baud;
  bool ftdxx;
  vp.get("servo_baudrate", baud, 1000000);
  vp.get("servo_ftd2xx", ftdxx, true);
  _base = new MorpheusDevice();
  _base->set_param(DynamixelDevice::COMTYPE, ftdxx ? FTDevice::FTD2XX : FTDevice::FTDI);
  _base->set_param(DynamixelDevice::VERBOSITY, 3);
  _base->set_param(DynamixelDevice::BAUDRATE, baud);

  // connect
  _base->connect();
  usleep(30000);

  // adjust dimensions
  state_dim       += _servos;
  measurement_dim += _servos;
  action_dim      += _servos;
  return true;
}

void JacoFacePlant::deinit ()
{
  _base->disconnect();
  JacoVisionPlant::deinit();
}

REGISTER_PLANT(JacoFacePlant, "Visual servoing plant for a Kinova Jaco robotic arm with dynamixel servos.");
