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

#ifndef _ROBOTISDEVICE_H_
#define _ROBOTISDEVICE_H_

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "dynamixel.h"
#include "robotis_defs.h"

/** Implements a device made from Dynamixel servos running the official
  * Robotis firmware. */
class RobotisDevice : public DynamixelDevice
{
 public:

  RobotisDevice () throw ();
  /** Creates a new device that attaches to another DynamixelDevice's FTDDevice,
    * to allow using both Robotis and Morpheus servos on the same bus. */
  RobotisDevice (DynamixelDevice* dev) throw ();
  ~RobotisDevice () throw () {
    _com->disconnect();
  };

  inline bool post_connect () throw () {
    return true;
  };

  /** Reads the data of the IMU.
    * \param id servo ID of the IMU
    * \param data pointer to an array of length 6 which will hold the determined roll, pitch and yaw,
    *        as well as sideways, forward and vertical velocities, in that order
    * \return false if data could not be read */
  bool get_imu (int id, int* data) throw ();

  /** Reads the data of the IMU.
    * \param id servo ID of the IMU
    * \param roll return variable for the IMU's roll rate
    * \param pitch return variable for the IMU's pitch rate
    * \param yaw return variable for the IMU's yaw rate
    * \param sac return variable for the IMU's sideway acceleration
    * \param fac return variable for the IMU's forward acceleration
    * \param vac return variable for the IMU's horizontal acceleration
    * \return false if data could not be read */
  bool get_imu (int id, int& roll, int& pitch, int& yaw, int& sac, int& fac, int& vac) throw ();

  bool get_pos (unsigned char id, int& pos, int& vel) throw ();
  bool get_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw ();
  bool set_pos (unsigned char id, int pos, int vel) throw ();
  bool set_pos (unsigned char id, int pos) throw ();
  bool set_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw ();
  bool set_pos (unsigned char motors, unsigned char* id, int* pos) throw ();
  bool update_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw ();
  bool reset (unsigned char id) throw ();
  bool set_id (unsigned char old_id, unsigned char new_id) throw ();

  bool set_param (DynamixelDevice::param param, int value) throw () {
    return DynamixelDevice::set_param(param, value);
  }

  /** Reads a parameter from a single servo.
    * \param id ID of the servo
    * \param param identifier of the parameter to be read
    * \param value return variable for the parameter's value
    * \return false if parameter could not be set */
  bool get_param (unsigned char id, Robotis::param param, int& value) throw ();

  /** Sets a parameter on a single AX-12 servo.
    * \param id ID of the servo
    * \param param identifier of the parameter to be read
    * \param value value the parameter is to be set to
    * \return false if parameter could not be set */
  bool set_param (unsigned char id, Robotis::param param, int value) throw ();

 protected:

  bool _tmp_suc;
  unsigned char _buffer[100], _packet[110], _tmp_csum;
  int _i, _t, _tmp_pos, _tmp_speed, _ret;

  /** Finalizes a packet and forwards it to the communication device.
    * \param id ID of the servo that the packet id intended for
    * \param length number of extra bytes included in the packet
    * \param command code of the command to be executed by the servo
    * \param send_twice send packet twice to ensure delivery; may cause double response
    * \param expected_bytes number of bytes that should be returned
    * \return false if the connection timed out or the response packet was malformed */
  bool communicate (unsigned char id, unsigned char length, unsigned char command, bool send_twice, int expected_bytes) throw ();

  /** Determines whether the chacksum of a response packet matches expectations and updates it with a correct one.
    * \param packet packet to be tested
    * \return true if received and calculated checksums are identical */
  bool validate (unsigned char* packet) throw ();

  /** Calculates the checksum for a dynamixel communication packet and writes it to the address specified within.
    * \param packet packet into which to insert the checksum */
  void checksum (unsigned char* packet) throw ();

  /** Interprets the error code in a status packet.
    * \param code error byte to be interpreted */
  void parse_error (unsigned char code) throw ();
};

#endif

