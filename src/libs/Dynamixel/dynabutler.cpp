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

#include "dynabutler.h"
#include <cmath>

#define MMSWAP(xxx,yyy) if (xxx > yyy) {int tmp = xxx; xxx = yyy; yyy = tmp;}

/** Servo configuration:
  * The first is the shoulder; the second and third are the secondary arm motors;
  * the next five are the primary arm motors; the last is the gripper. */

Dynabutler::Dynabutler ()
{
  // create default lists
  _joints = 6;
  for (_i=0; _i<_joints+2; _i++)
    _ids[_i] = AX12Arm::ids[_i];
  for (_i=0; _i<_joints; _i++) {
    _spd[_i] = 2;
    _min[_i] = AX12Arm::min[_i];
    _max[_i] = AX12Arm::max[_i];
  }

  // link to device
  _arm = new MorpheusDevice();
  _arm->set_param(DynamixelDevice::COMTYPE, FTDevice::FTD2XX);
  _arm->connect();
  _shoulder = new RobotisDevice(_arm);
  _crop = false;
}

Dynabutler::~Dynabutler ()
{
  Dynabutler::free();
  _arm->disconnect();
}

void Dynabutler::free () throw ()
{
  std::cout << "Loosening joints..." << std::endl;;

  // PWM for Morpheus firmware
  int p[_joints+1];
  for (_i=0; _i<_joints+1; _i++)
    p[_i] = 1001; // must be 1001 rather than 1000, ignored otherwise (only in broadcast)
  _arm->set_param(_joints+1, &_ids[1], Morpheus::PWM, &p[0]);

  // torque for Robotis firmware
  _shoulder->set_param(_ids[0], Robotis::TORQUE_ENABLE, 0);
  sleep(1);
}

bool Dynabutler::check_range (int& target, const int joint) throw ()
{
  if (!_crop) {
    if (target < _min[joint] || target > _max[joint]) {
      printf("Error: target encoder out of range (joint %i, target %i, should be in [%i:%i])\n",joint,target,_min[joint],_max[joint]);
      return false;
    }
  } else {
    if (target < _min[joint]) target = _min[joint];
    else if (target > _max[joint]) target = _max[joint];
  }
  return true;
}

bool Dynabutler::set_pos (unsigned char joint, int goal, int speed) throw ()
{
  // sanity checks
  if (joint >= _joints || joint < 0) {
    std::cerr << "Joint " << joint << " does not exist." << std::endl;
    return false;
  }
  check_range(goal, joint);

  // MX-28 has different value ranges
  if (joint == 0) {
    _shoulder->set_pos(_ids[0], goal*4, speed*100);
  }

  // mirror command for double joints
  else if (joint == 1 || joint == 2) {
    _tmp_ids[0] = _ids[joint+2];
    _tmp_ids[1] = _ids[joint];
    _pos[0] = goal;
    _pos[1] = 1023 - goal;
    _vel[0] = _vel[1] = speed;
    return _arm->set_pos(2, &_tmp_ids[0], &_pos[0], &_vel[0]);
  }

  // just send for normal joints
  else {
    return _arm->set_pos(_ids[joint+2], goal, speed);
  }
  return _success;
}

bool Dynabutler::set_pos (int* goal) throw ()
{
  return set_pos(goal, &_spd[0]);
}

bool Dynabutler::set_pos (int* goal, int* speed) throw ()
{
  // check input range
  for (_i=0; _i<_joints; _i++) {
    check_range(goal[_i], _i);
    _pos[_i+2] = goal[_i];
    _vel[_i+2] = speed[_i];
  }

  // move shoulder to front
  _pos[0] = _pos[2];
  _vel[0] = _vel[2];

  // mirror secondary motors
  _pos[1] = 1023 - _pos[3];
  _pos[2] = 1023 - _pos[4];
  _vel[1] = speed[3];
  _vel[2] = speed[4];

  // send to robot
  return _arm->set_pos(_joints+1, &_ids[1], &_pos[1], &_vel[1])
      && _shoulder->set_pos(_ids[0], _pos[0]*4, _vel[0]*100);
}

bool Dynabutler::get_pos (unsigned char joint, int& position) throw ()
{
  // sanity check
  if (joint >= _joints || joint < 0) {
    std::cerr << "Joint " << joint << " does not exist." << std::endl;
    return false;
  }

  // get data
  if (joint == 0) {
    if (!_shoulder->get_pos(_ids[0], position, _vel[0])) return false;
    position /= 4;
    return true;
  } else
    return _arm->get_pos(_ids[joint+2], position, _vel[0]);
}

bool Dynabutler::get_pos (int* buffer) throw ()
{
  // get and modify shoulder
  if (!_shoulder->get_pos(_ids[0], buffer[0], _vel[0])) return false;
  buffer[0] /= 4;

  // get arm; IDs to read start at 3, since secondaries not needed
  return _arm->get_pos(_joints-1, &_ids[3], &buffer[1], &_vel[1]);
}

bool Dynabutler::add_joints (int num, const unsigned char* ids, const int* min, const int* max) throw ()
{
  // check if room left
  if (_joints + num > DBMAXJOINTS) {
    std::cerr << "Failed to add " << num << " joints: maximum of " << DBMAXJOINTS << " would be exceeded with " << _joints << " present already." << std::endl;
    return false;
  }

  for (_i=0; _i<num; _i++) {
    _ids[_joints+_i+2] = ids[_i];
    _spd[_joints+_i]   = 2;
    _min[_joints+_i]   = min[_i];
    _max[_joints+_i]   = max[_i];
    MMSWAP(_min[_i],_max[_i]);
  }
  _joints += num;
  return true;
}

bool Dynabutler::wait (const int* goal, int tolerance, int timeout) throw ()
{
  bool there = false;
  int pos[_joints];
  for (int t=10; t>0; t--) {
    there = true;
    Dynabutler::get_pos(&pos[0]);
    for (_i=0; _i<_joints; _i++)
      there &= abs(pos[_i]-goal[_i]) < tolerance;
    if (there) return true;
    usleep(500000);
  }
  std::cerr << "Wait for action times out." << std::endl;
  return false;
}

bool Dynabutler::add_joint (const unsigned char id, const int min, const int max) throw ()
{
  if (_joints == DBMAXJOINTS) {
    std::cerr << "Failed to add joint: maximum of " << DBMAXJOINTS << " reached." << std::endl;
    return false;
  }

  _ids[_joints+2] = id;
  _spd[_joints]   = 2;
  _min[_joints]   = min;
  _max[_joints]   = max;
  MMSWAP(_min[_joints],_max[_joints]);
  _joints++;
  return true;
}

