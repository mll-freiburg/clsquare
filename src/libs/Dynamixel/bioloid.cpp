/*
libDynamixel - Robotis Dynamixel servo interface
Copyright (c) 2010-2012 Machine Learning Lab, 
Thomas Lampe

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

#include "bioloid.h"

// adjustments for servo positions; interface = real - _cali
// not really needed anymore for some reason
const int _cali[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/************** Setup **************/

Bioloid::Bioloid () throw ()
{
  _id_cam = 100;
  _id_imu = 120;
  _default_speed = 255;
  _verbosity = 100;
  _delta_t = 500;
  _base = new RobotisDevice();
  _base->set_param(DynamixelDevice::COMTYPE, FTDevice::FTDI);
  gettimeofday(&_start, NULL);
}

bool Bioloid::init () throw ()
{
  _speedbuffer = new int[20];
  for (_i=0; _i<20; _i++)
    _speedbuffer[_i] = _default_speed;

  bool success = _base->connect();

  for (_i=1; _i<19; _i++) success &= _base->test_response(_i);
  success &= _base->test_response(_id_cam);
  if (_id_imu >= 0 && !get_imu(_tmp_pos,_tmp_pos,_tmp_pos,_tmp_pos,_tmp_pos,_tmp_pos)) {
    LWARNING("IMU " << _id_imu << " not responsive!");
    success = false;
  }
  if (!success) LWARNING("not all hardware components operational");
  return success;
}

bool Bioloid::set_param (Bioloid::param param, int value) throw ()
{
  switch (param) {
  case Bioloid::CAM_ID:
    _id_cam = value;
    break;
  case Bioloid::IMU_ID:
    _id_imu = value;
    break;
  default:
    LWARNING("unknown parameter");
    return false;
  }
  return true;
}

bool Bioloid::step () throw ()
{
  gettimeofday(&_stop, NULL);

  _remaining = 1000 * _delta_t - (1000000 * (_stop.tv_sec - _start.tv_sec) + (_stop.tv_usec - _start.tv_usec));
  if (_remaining > 0) {
    usleep(_remaining);
  } else
    LWARNING("communication took " << (1.*_remaining / -1000.) << " milliseconds too long.");

  gettimeofday(&_start, NULL);
  return true;
}

bool Bioloid::reset () throw ()
{
  return true;
}

Bioloid::~Bioloid () throw ()
{
  _base->disconnect();
}

/************** READ_DATA **************/

bool Bioloid::get_pos (unsigned char id, int& pos, int& speed) throw ()
{
  if (!_base->get_pos(id,pos,speed)) return false;
  if (id < 20) pos -= _cali[id];
  return true;
}

bool Bioloid::get_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ()
{
  if (!_base->get_pos(motors,id,pos,speed)) return false;
  for (_i=0; _i<motors; _i++)
    if (id[_i]<20) pos[_i] -= _cali[id[_i]];
  return true;
}

bool Bioloid::get_imu (int* data) throw ()
{
  return get_imu(data[0],data[1],data[2],data[3],data[4],data[5]);
}

bool Bioloid::get_imu (int& roll, int& pitch, int& yaw, int& vel1, int& vel2, int& vel3) throw ()
{
  if (_id_imu < 0) {
    LERROR("no ID assigned for IMU.");
    return false;
  }
  return _base->get_imu(_id_imu, roll, pitch, yaw, vel1, vel2, vel3);
}

/************** WRITE_DATA **************/

bool Bioloid::set_pos (unsigned char id, int pos) throw ()
{
  return set_pos(id, pos, _default_speed);
}

bool Bioloid::set_pos (unsigned char id, int pos, int speed) throw ()
{
  if (id < 20) pos += _cali[id];
  return _base->set_pos(id, pos, speed);
}

/************** WRITE_DATA_SYNC **************/

bool Bioloid::set_pos (unsigned char motors, unsigned char* id, int* pos) throw ()
{
  return set_pos(motors, id, pos, _speedbuffer);
}

bool Bioloid::set_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ()
{
  for (_i=0; _i<motors; _i++) if (id[_i] < 20) pos[_i] += _cali[id[_i]];
  return _base->set_pos(motors, id, pos, speed);
}

