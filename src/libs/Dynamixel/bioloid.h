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

#ifndef _BIOLOID_H_
#define _BIOLOID_H_

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "loid.h"
#include "robotis.h"

/** Implementation of a real Loid-type humaniod robot, with communication
  * realized by an FTDIDevice. */
class Bioloid : public Loid
{
public:

  /** Specific parameters that apply only to a real Loid. */
  enum param {
    /// Servo ID of the HaViMo camera.
    CAM_ID,
    /// Servo ID of the IMU.
    IMU_ID
  };

  Bioloid () throw ();
  ~Bioloid () throw ();

  /** Sets a Bioloid-specific internal parameter.
    \param param identifier of the parameter to be set
    \param value desired value
    \return false if identifier not known */
  bool set_param (Bioloid::param param, int value) throw ();
  inline bool set_param (Loid::param param, int value) throw () {
    return Loid::set_param(param, value);
  };

  inline bool set_param (DynamixelDevice::param param, int value) throw () {
    return _base->set_param(param, value);
  };

  bool init () throw ();
  bool reset () throw ();
  bool step () throw ();

  // specific output as per interface
  bool get_pos (unsigned char id, int& pos, int& speed) throw ();
  bool get_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ();
  bool get_imu (int& roll, int& pitch, int& yaw, int& vel1, int& vel2, int& vel3) throw ();
  bool get_imu (int* data) throw ();

  // specific input as per interface
  bool set_pos (unsigned char id, int pos) throw ();
  bool set_pos (unsigned char id, int pos, int speed) throw ();
  bool set_pos (unsigned char motors, unsigned char* id, int* pos) throw ();
  bool set_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ();

protected:
  RobotisDevice *_base;
  int _i, _id_cam, _id_imu, _tmp_pos, _tmp_speed, *_speedbuffer;
  bool _tmp_suc;
  timeval _start, _stop;
  long int _remaining;
};

#endif

