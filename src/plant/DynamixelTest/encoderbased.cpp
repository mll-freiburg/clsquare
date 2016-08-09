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

#include "global.h"
#include "encoderbased.h"
#include "valueparser.h"
#include <string>
#include <cmath>
#include <cstdlib>
#include "robotis.h"
#include "morpheus.h"

using namespace std;

bool DynamixelEncoderTest::get_next_plant_state(const double *plant_state, const double *action, double *next_plant_state)
{
  gettimeofday(&_start,NULL);

  // translate actions into command
  std::cout << "Sending action";
  for (_i=0; _i<_servos; _i++) {
    _tmpa = _scale[_i] * fabs(action[_i]);
    if (_tmpa < 1) _tmpa = 1;
    _speed[_i] = _tmpa;
    _pos[_i] = action[_i] > 0 ? 1011 : 12;
    std::cout << " (" << _pos[_i] << ":" << _speed[_i] << ")";
  }
  std::cout << ", ";

  // send command
  _base->set_pos(_servos, &_id[0], &_pos[0], &_speed[0]);
  _base->get_pos(_servos, &_id[0], &_pos[0], &_speed[0]);

  // determine new state
  std::cout << "servos report ";
  for (_i=0; _i<_servos; _i++) {
    std::cout << "(" << _pos[_i] << ":" << _speed[_i] << ") ";
    next_plant_state[_i] = _pos[_i] < 1024 ? _pos[_i] : plant_state[_i];
  }
  std::cout << "after action ";
  for (_i=0; _i<_servos; _i++)
    std::cout << action[_i] << " ";
  std::cout << "in state ";
  for (_i=0; _i<_servos; _i++)
    std::cout << plant_state[_i] << " ";
  std::cout << std::endl;

  gettimeofday(&_stop,NULL);
  _remaining = 1000 * _delta_t - (1000000 * (_stop.tv_sec - _start.tv_sec) + (_stop.tv_usec - _start.tv_usec)); 
  if (_remaining > 0) {
    //IOUT("Communication took " << 1.*_delta_t - 1.*_remaining/1000. << " milliseconds.");
    usleep(_remaining);
  } else
    WOUT(10, "Warning: communication took " << (1.*_remaining / -1000.) << " milliseconds too long.");

  return true;
}

bool DynamixelEncoderTest::get_measurement(const double *plant_state, double *observation)
{
  for (_i=0; _i<_servos; _i++)
    observation[_i] = plant_state[_i];
  return true;
}

bool DynamixelEncoderTest::check_initial_state(double *initial_plant_state)
{
  return true;
}

bool DynamixelEncoderTest::init (int& plant_state_dim, int& observation_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  ValueParser vr(fname,chapter==0?"Plant":chapter);

  _delta_t = 100;
  vr.get("delta_t",_delta_t);	

  int servonum;
  vr.get("servos", servonum, 1);
  if (servonum < 1)
    return false;
  _servos = servonum;

  long int baud;
  vr.get("baudrate", baud, 1000000);

  int tmp[_servos];
  vr.get("ids", tmp, _servos);
  for (_i=0; _i<_servos; _i++)
    _id[_i] = tmp[_i];

  for (_i=0; _i<_servos; _i++)
    _torque[_i] = 100;
  vr.get("torque", _torque, _servos);

  for (_i=0; _i<_servos; _i++)
    _null[_i] = 512;
  vr.get("start", _null, _servos);

  for (_i=0; _i<_servos; _i++)
    _scale[_i] = 100;
  vr.get("scaling", _scale, _servos);

  // determine protocol
  bool ftdxx;
  vr.get("use_ftd2xx", ftdxx, false);
  vr.get("custom_firmware", _morph, false);
  if (_morph)
    _base = new MorpheusDevice();
  else
    _base = new RobotisDevice();
  _base->set_param(DynamixelDevice::COMTYPE, ftdxx ? FTDevice::FTD2XX : FTDevice::FTDI);
  _base->set_param(DynamixelDevice::VERBOSITY, 3);
  _base->set_param(DynamixelDevice::BAUDRATE, baud);

  // connect
  _base->connect();
  usleep(30000);
  
  if (!_morph) for (_i=0; _i<_servos; _i++) {
    ((RobotisDevice*)_base)->set_param(_id[_i], Robotis::TORQUE_LIMIT, _torque[_i]);
  }

  plant_state_dim = action_dim = observation_dim = _servos;
  return true;
}

void DynamixelEncoderTest::deinit()
{
  if (!_morph)
    for (_i=0; _i<_servos; _i++)
      ((RobotisDevice*)_base)->set_param(_id[_i], Robotis::TORQUE_LIMIT, _torque[_i]);
  else   
    for (_i=0; _i<_servos; _i++)
      ((MorpheusDevice*)_base)->set_param(_id[_i], Morpheus::PWM, 1000);
  _base->disconnect();
}

void DynamixelEncoderTest::notify_episode_starts ()
{
  std::cout << "Reinitializing to ";
  for (_i=0; _i<_servos; _i++) {
    std::cout << _null[_i] << " ";
    _speed[_i] = 512;
  }
  std::cout << std::endl;
  _base->set_pos(_servos, &_id[0], &_null[0], &_speed[0]);
  sleep(10);
}

DynamixelEncoderTest::DynamixelEncoderTest ()
{
}

DynamixelEncoderTest::~DynamixelEncoderTest ()
{
  _base->disconnect();
  delete _base;
}

REGISTER_PLANT(DynamixelEncoderTest, "Test for FTD devices, encoder-based.");
