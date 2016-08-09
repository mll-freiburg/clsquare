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

#include "robotis.h"

/************** Setup **************/

RobotisDevice::RobotisDevice () throw ()
{
  _packet[0] = 0xFF;
  _packet[1] = 0xFF;
}

RobotisDevice::RobotisDevice (DynamixelDevice* dev) throw ()
{
  _packet[0] = 0xFF;
  _packet[1] = 0xFF;
  _com = dev->get_base();
}

/************** READ_DATA **************/

bool RobotisDevice::get_pos (unsigned char id, int& pos, int& speed) throw ()
{
  _packet[5] = 0x24; // address
  _packet[6] = 0x04; // number of bytes to be read
  if (communicate(id, 0x04, 0x02, false, 4)) {
    pos   = _buffer[5] + _buffer[6] * 256;
    speed = _buffer[7] + _buffer[8] * 256;
    if (speed > 1023) {
      speed -= 1023;
      speed *= -1;
    }
    return true;
  }
  return false;
}

bool RobotisDevice::get_imu (int id, int* data) throw ()
{
  return get_imu(id,data[0],data[1],data[2],data[3],data[4],data[5]);
}

bool RobotisDevice::get_imu (int id, int& roll, int& pitch, int& yaw, int& sac, int& fac, int& vac) throw ()
{
  _packet[5] = 0x1A; // address
  _packet[6] = 12; // number of bytes to be read
  if (communicate(id, 2, 0x02, false, 12)) {
    sac   = _buffer[5]  + 256 * _buffer[6];
    fac   = _buffer[7]  + 256 * _buffer[8];
    vac   = _buffer[9]  + 256 * _buffer[8];
    pitch = _buffer[11] + 256 * _buffer[12];
    roll  = _buffer[13] + 256 * _buffer[14];
    yaw   = _buffer[15] + 256 * _buffer[16];
    return true;
  }
  return false;
}

bool RobotisDevice::get_param (unsigned char id, Robotis::param param, int& value) throw ()
{
  _packet[5] = (unsigned char)param;
  if (_packet[5] > 0x4f) {
    _packet[5] -= 0x50;
    _packet[6] = 2;
    if (communicate(id, 2, 0x02, false, 2)) {
      value = _buffer[5] + _buffer[6] * 256;
      return true;
    }
  } else {
    _packet[6] = 1;
    if (communicate(id, 2, 0x02, false, 1)) {
      value = _buffer[5];
      return true;
    }
  }
  return false;
}

/************** READ_DATA_SYNC **************/

bool RobotisDevice::get_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw ()
{
  // since the Robotis firmware does not support synchronous read, query all separately
  for (_i=0; _i<motors; _i++)
    if (!get_pos(id[_i], pos[_i], vel[_i]))
      return false;
  return true;
}

bool RobotisDevice::update_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw ()
{
  return set_pos(motors, id, pos, vel) && get_pos(motors, id, pos, vel);
}

/************** WRITE_DATA **************/

bool RobotisDevice::set_pos (unsigned char id, int pos, int speed) throw ()
{
  _packet[5] = 0x1E; // address
  _packet[6] = LOWER(pos);
  _packet[7] = UPPER(pos);
  _packet[8] = LOWER(speed);
  _packet[9] = UPPER(speed);
  return communicate(id, 5, 0x03, true, 0);
  ///< \todo better than checking length: look for pattern
}

bool RobotisDevice::set_pos (unsigned char id, int pos) throw ()
{
  _packet[5] = 0x1E; // address
  _packet[6] = LOWER(pos);
  _packet[7] = UPPER(pos);
  return communicate(id, 3, 0x03, true, 0);
}

bool RobotisDevice::set_id (unsigned char old_id, unsigned char new_id) throw ()
{
  if (!test_response(old_id)) {
    DYNAWARNING("old ID does not exist");
    return false;
  }
  if ((_tmp_suc |= test_response(new_id))) {
    DYNAWARNING("new ID already taken");
    return false;
  }
  set_param(old_id,Robotis::SERVO_ID,new_id);
  if (!test_response(new_id)) {
    DYNAWARNING("new ID not responsive");
    return false;
  }
  return true;
}

bool RobotisDevice::set_param (unsigned char id, Robotis::param param, int value) throw ()
{
  switch (param) {
  case Robotis::MODEL:
  case Robotis::FIRMWARE:
  case Robotis::PRESENT_POSITION:
  case Robotis::PRESENT_SPEED:
  case Robotis::PRESENT_LOAD:
  case Robotis::PRESENT_VOLTAGE:
  case Robotis::PRESENT_TEMPERATURE:
  case Robotis::MOVING:
    DYNAERROR("Cannot write read-only data.");
    return false;
  default:
    _packet[5] = (unsigned char)param;
    if (_packet[5] > 0x4f) {
      _packet[5] -= 0x50;
      _packet[6] = LOWER(value);
      _packet[7] = UPPER(value);
      return communicate(id, 3, 0x03, true, 0);
    } else {
      _packet[6] = value;
      return communicate(id, 2, 0x03, true, 0);
    }
  }
}

/************** WRITE_DATA_SYNC **************/

bool RobotisDevice::set_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ()
{
  if (motors < 1) return false;
  _packet[5] = 0x1E; // address
  _packet[6] = 0x04; // bytes written per ID
  for (_i=0; _i<motors; _i++) {
    _packet[5*_i+7] = id[_i];
    _packet[5*_i+8] = LOWER(pos[_i]);
    _packet[5*_i+9] = UPPER(pos[_i]);
    _packet[5*_i+10] = LOWER(speed[_i]);
    _packet[5*_i+11] = UPPER(speed[_i]);
  }
  return communicate(0xfe, motors*5+2, 0x83, true, 0);
}

bool RobotisDevice::set_pos (unsigned char motors, unsigned char* id, int* pos) throw ()
{
  if (motors < 1) return false;
  _packet[5] = 0x1E; // address
  _packet[6] = 0x02; // bytes written per ID
  for (_i=0; _i<motors; _i++) {
    _packet[3*_i+7] = id[_i];
    _packet[3*_i+8] = LOWER(pos[_i]);
    _packet[3*_i+9] = UPPER(pos[_i]);
  }
  return communicate(0xfe, motors*3+2, 0x83, true, 0);
}

/************** RESET **************/

bool RobotisDevice::reset (unsigned char id) throw ()
{
  if (!test_response(id)) DYNAWARNING("ID does not exist");
  if (test_response(0)) DYNAWARNING("ID 0 already taken");
  return communicate(id, 0, 0x06, true, 0);
}

/************** Helpers **************/

bool RobotisDevice::validate (unsigned char* packet) throw ()
{
  _tmp_csum = packet[packet[3]+3];
  checksum(&packet[0]);
  return _tmp_csum == packet[packet[3]+3];
}

void RobotisDevice::checksum (unsigned char* packet) throw ()
{
  packet[packet[3]+3] = 0xFF;
  for (_t=2; _t<packet[3]+3; _t++)
    packet[packet[3]+3] -= packet[_t];
}

bool RobotisDevice::communicate (unsigned char id, unsigned char length, unsigned char command, bool send_twice, int expected_bytes) throw ()
{
  _packet[2] = id;
  _packet[3] = length + 2;
  _packet[4] = command;

  for (_t=0; _t<100; _t++)
    _buffer[_t] = 0;

  checksum(&_packet[0]);

  _ret = _com->communicate(&_packet[0], &_buffer[0], length+6, expected_bytes+6, send_twice);
  if (_ret > 5) parse_error(_buffer[4]);

  if (expected_bytes > 0 && _ret < expected_bytes+6) {
    DYNAERROR("communication failure: too little data received: " << _ret << " instead of " << expected_bytes+6);
    return false;
  }
  if (_ret > 5 && _validate_response && !validate(&_buffer[0])) {
    DYNAERROR("communication failure: checksum of response packet is " << (int)_tmp_csum << ", should be " << (int)_buffer[_buffer[3]+3]);
    return false;
  }

  return true;
}

void RobotisDevice::parse_error (unsigned char code) throw ()
{
  if (code == 0) return;

  if (code % 2 == 1)
    DYNAWARNING("Input Voltage Error: voltage out of range.");

  code >>= 1;
  if (code % 2 == 1)
    DYNAWARNING("Angle Limit Error: goal position out of range.");

  code >>= 1;
  if (code % 2 == 1)
    DYNAWARNING("Overheating Error: internal temperature above limit.");

  code >>= 1;
  if (code % 2 == 1)
    DYNAWARNING("Range Error: sent instruction out of range.");

  code >>= 1;
  if (code % 2 == 1)
    DYNAWARNING("Checksumn Error: checksum of instruction packet incorrect.");

  code >>= 1;
  if (code % 2 == 1)
    DYNAWARNING("Overload Error: maximum torque cannot control applied load.");

  code >>= 1;
  if (code % 2 == 1)
    DYNAWARNING("Instruction Error: undefined instruction or instruction without reg_write.");
}

