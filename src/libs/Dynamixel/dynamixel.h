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

#ifndef _DYNAMIXELDEVICE_H_
#define _DYNAMIXELDEVICE_H_

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "ftdevice.h"

#define DYNAERROR(x) {if (_verbosity > 0) std::cerr << "\033[1;31m#Dynamixel Error\033[0m:   " << x << std::endl;}
#define DYNAWARNING(x) {if (_verbosity > 1) std::cerr << "\033[1;33m#Dynamixel Warning\033[0m: " << x << std::endl;}
#define DYNALOG(x) {if (_verbosity > 2) std::cout << "#Dynamixel Info:    " << x << std::endl;}
#define UPPER(x) (x) / 256
#define LOWER(x) (x) % 256

/** Sub class that implements the communication with a Dynamixel device.
  * This includes AX-12 servos, for which it was originally intended, but 
  * can also be used for other Dynamixel models, including RS485 devices 
  * such as the RX-64. */
class DynamixelDevice
{
 public:

  /** Defines parameters of Dynamixel devices that apply to all servos
    * it includes. May be set with DynamixelDevice::set_param(DynamixelDevice::param,int). */
  enum param {
    /// Communication latency.
    LATENCY,
    /// Maximum number of communication attempts per operation.
    TIMEOUT,
    /// Toggles validation of the checksum of return packets.
    VALIDATE,
    /// Verbosity level of the interface (1 for errors, 2 for warnings, 3 for everything).
    VERBOSITY,
    /// Baudrate of the FTD bus.
    BAUDRATE,
    /// Type of communication device used
    COMTYPE
  };

  DynamixelDevice () throw ();
  virtual ~DynamixelDevice () throw () {
    _com->disconnect();
  };

  /** Opens a connection to the FTD bus.
    * \return false if connection could not be opened. */
  inline bool connect () throw () {
    return _com->connect(_baudrate) && post_connect();
  };
  virtual bool post_connect () throw () = 0;

  /** Closes the connection to the servo bus.
    * \return false if connection could not be closed or is not open. */
  inline bool disconnect () throw () {
    return _com->disconnect();
  };

  /** Sets an AX12Base-specific internal parameter.
    * \param param identifier of the parameter to be set
    * \param value desired value
    * \return false if identifier not known */
  bool set_param (DynamixelDevice::param param, int value) throw ();

  /** Reads the position and active velocity of a single AX-12 servo.
    * \param id ID of the servo
    * \param pos return variable for current position
    * \param vel return variable for current active velocity
    * \return false if information could not be read */
  virtual bool get_pos (unsigned char id, int& pos, int& vel) throw () = 0;

  /** Reads the current position from all servos.
    * \param motors the number of servos that make up the device
    * \param ids buffer that will hold the ids of all servos that have answered
    * \param pos positions of the servos, ordered as in ids
    * \param vel velocities of the servos, ordered as in ids
    * \return false if communication was incomplete */
  virtual bool get_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw () = 0;

  /** Sets the target position and speed of a single servo.
    * \param id ID of the servo
    * \param pos target position
    * \param vel target speed
    * \return false if target values could not be set */
  virtual bool set_pos (unsigned char id, int pos, int vel) throw () = 0;

  /** Sets the target position of a single servo.
    * \param id ID of the servo
    * \param pos target position
    * \return false if target values could not be set */
  virtual bool set_pos (unsigned char id, int pos) throw () = 0;

  /** Sets the target position and speed of multiple servos simultaneously.
    * \param motors number of servos to be set
    * \param id array containing IDs of the servos
    * \param pos array containing target positions, ordered as id parameter
    * \param vel array containing target speeds, ordered as id parameter
    * \return false if target values could not be set */
  virtual bool set_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw () = 0;

  /** Sets the target position of multiple servos simultaneously.
    * \param motors number of servos to be set
    * \param id array containing IDs of the servos
    * \param pos array containing target positions, ordered as id parameter
    * \return false if target values could not be set */
  virtual bool set_pos (unsigned char motors, unsigned char* id, int* pos) throw () = 0;

  /** Resets a single servo to factory specifications. Note: this will also reset its ID to 0.
    * \param id ID of the servo to be reset
    * \return false if servo could not be reset */
  virtual bool reset (unsigned char id) throw () = 0;

  /** Changes the ID of a single servo.
    * \param old_id ID of the servo whose number is to be changed
    * \param new_id desired new ID
    * \return false if ID could not be changed */
  virtual bool set_id (unsigned char old_id, unsigned char new_id) throw () = 0;

  /** Sets the target position and speed of multiple servos simultaneously and returns their new values.
    * \param motors number of servos to be set
    * \param id array containing IDs of the servos
    * \param pos array containing target positions, ordered as id parameter; will hold the new positions
    * \param vel array containing target speeds, ordered as id parameter; will hold the new speeds
    * \return false if target values could not be set */
  virtual bool update_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw () = 0;

  /** Checks whether a servo is available or not.
    * Response is tested by simply trying to read its position.
    * \param id dynamixel ID of the servo to be tested
    * \return true if the position byte of the servo can be read */
  bool test_response (int id) throw ();

  /** Passes a handle to the device's communication base.
    * \return handle to the device's communication base */
  FTDevice* get_base () throw () {
    return _com;
  };

 protected:

  // parameters
  int _tmpp, _tmps;
  int _validate_response, _verbosity, _latency, _timeout;
  long int _baudrate;
  FTDevice *_com;

};

#endif

