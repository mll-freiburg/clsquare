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

#ifndef _MORPHEUSDEVICE_H_
#define _MORPHEUSDEVICE_H_

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "dynamixel.h"
#include "morpheus_defs.h"

#define MORPHEUS_BUFFER_SIZE 2048

/** Implements a device made from Dynamixel servos running the official
  * Morpheus firmware. */
class MorpheusDevice : public DynamixelDevice
{
 public:

  MorpheusDevice () throw ();
  /** Creates a new device that attaches to another DynamixelDevice's FTDDevice,
    * to allow using both Robotis and Morpheus servos on the same bus. */
  MorpheusDevice (DynamixelDevice* dev) throw ();
  ~MorpheusDevice () throw () {
    _com->disconnect();
  };

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

  inline bool post_connect () throw () {
    _motors = 0;
    return start(_motors, _ids);
  };

  /** Reads a parameter from a single servo.
    * \param id ID of the servo
    * \param param identifier of the parameter to be read
    * \param value return variable for the parameter's value
    * \return false if parameter could not be set */
  bool get_param (unsigned char id, Morpheus::param param, int& value) throw ();

  /** Sets a parameter on a single AX-12 servo.
    * \param id ID of the servo
    * \param param identifier of the parameter to be read
    * \param value value the parameter is to be set to
    * \return false if parameter could not be set */
  bool set_param (unsigned char id, Morpheus::param param, int value) throw ();

  /** Sets parameters on several AX-12 servos at once.
    * \param motors number of motors to be written to
    * \param id IDs of the servos
    * \param param identifier of the parameter to be read
    * \param value values the parameters are to be set to
    * \return false if parameter could not be set */
  bool set_param (unsigned char motors, unsigned char* ids, Morpheus::param param, int* value) throw ();

  /** Initializes the response cycle to allow subsequent SYNC_READ commands.
    * \param motors the number of servos that make up the device
    * \param ids buffer that will hold the ids of all servos that have answered
    * \return false if cycle could not be started */
  bool start (unsigned char& motors, unsigned char* ids) throw ();

 protected:

  bool _tmp_suc, _started;
  unsigned char _buffer[MORPHEUS_BUFFER_SIZE], _packet[MORPHEUS_BUFFER_SIZE], _tmp_csum, _motors, _ids[256], _leftovers[MORPHEUS_BUFFER_SIZE];
  int _i, _k, _l, _t, _tmp_pos, _tmp_speed, _ret, _leftover, _missing, _packetnum, _exp, _plength, _packetlist[256], _left;

  /** Finalizes a packet and forwards it to the communication device.
    * \param id ID of the servo that the packet id intended for
    * \param length number of data bytes included in the outgoing packet
    * \param command code of the command to be executed by the servo
    * \param twice send packet twice to ensure delivery; may cause double response
    * \param expected number of data bytes that should be in the incoming packet
    * \param extra number of additional bytes to send if different from length parameter
    * \return false if the connection timed out or the response packet was malformed */
  bool communicate (unsigned char id, unsigned char length, unsigned char command, int expected=0, bool twice=false, int extra=0) throw ();

  /** Calculates the checksum for a dynamixel communication packet and writes it to the address specified within.
    * \param packet packet into which to insert the checksum */
  void checksum (unsigned char* packet) throw ();
};

#endif

