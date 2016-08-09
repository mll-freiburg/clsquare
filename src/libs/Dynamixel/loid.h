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

#ifndef _LOID_H_
#define _LOID_H_

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#define LERROR(x) {if (_verbosity > 0) std::cerr << "Error: " << x << std::endl;}
#define LWARNING(x) {if (_verbosity > 1) std::cerr << "Warning: " << x << std::endl;}
#define LLOG(x) {if (_verbosity > 2) std::cout << x << std::endl;}

/** Provides the basis for a small humanoid robot composed of AX-12 servos.
  * This abstract class only includes elements that relate to the robot's
  * form factor, servo layout and basic function, and can be extended to both
  * a real and a simulated version. */
class Loid
{
public:

  /** Defines parameters that apply to any kind of Loid robot.
    * To be set through Loid::set_param(Loid::param,int). */
  enum param {
    /// Verbosity level of the interface. 0 for none, 1 for errors, 2 for warnings, 3 for all.
    VERBOSITY,
    /// Duration of one discrete time step of the robot.
    CYCLE_TIME_MS,
    /// Default movement speed.
    DEFAULT_SPEED,
    /// Toggles holding unused joints in position.
    HOLD
  };

  /** Initializes the connection with the robot.
    * \return false if connection could not be opened */
  virtual bool init () throw () = 0;

  /** Sets an internal parameter.
    * \param param identifier of the parameter to be set
    * \param value desired value
    * \return false if identifier not known */
  inline bool set_param (Loid::param param, int value) throw () {
    switch (param) {
    case Loid::VERBOSITY:
      _verbosity = value;
      break;
    case Loid::CYCLE_TIME_MS:
      _delta_t = value;
      break;
    case Loid::DEFAULT_SPEED:
      _default_speed = value;
      break;
    case Loid::HOLD:
      _hold = value == 1;
      break;
    default:
      std::cerr << "Unknown parameter identifier." << std::endl;
      return false;
    }
    return true;
  };

  /** Determines the position and speed of one of the robot's joints.
    * \param id ID of the joint to be read
    * \param pos return value for the joint's position
    * \param speed return value for the joint's speed
    * \return false if position could not be determined */
  virtual bool get_pos (unsigned char id, int& pos, int& speed) throw () = 0;

  /** Determines the position and speed of several of the robot's joints.
    * \param motors number of motors to be read
    * \param id IDs of the joints to be read
    * \param pos return buffer for the joint positions
    * \param speed return buffer for the joint speeds
    * \return false if position could not be determined */
  virtual bool get_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw () = 0;

  /** Reads the data of the IMU.
    * \param data pointer to an array of length 6 which will hold the determined roll, pitch, yaw, and three velocities, in that order
    * \return false if data could not be read */
  virtual bool get_imu (int* data) throw () = 0;

  /** Reads the data of the IMU.
    * \param roll return variable for the IMU's roll
    * \param pitch return variable for the IMU's pitch
    * \param yaw return variable for the IMU's yaw
    * \param vel1 return variable for the IMU's first velocity
    * \param vel2 return variable for the IMU's second velocity
    * \param vel3 return variable for the IMU's third velocity
    * \return false if data could not be read */
  virtual bool get_imu (int& roll, int& pitch, int& yaw, int& vel1, int& vel2, int& vel3) throw () = 0;

  /** Advances the robot by one time step. Obligatory for Simloid derivates.
    * \return false if no step could be performed */
  virtual bool step () throw () = 0;

  /** Causes the robot to wait until all dynamics are stable.
    * \return false if robot could not be reset */
  virtual bool reset () throw () = 0;

  /** Sets the target position of a single robot servo.
    * \param id ID of the joint to be moved
    * \param pos target position
    * \return false if target could not be set */
  virtual bool set_pos (unsigned char id, int pos) throw () = 0;

  /** Sets the target position and active speed of a single robot servo.
    * \param id ID of the joint to be moved
    * \param pos target position
    * \param speed target active speed
    * \return false if target could not be set */
  virtual bool set_pos (unsigned char id, int pos, int speed) throw () = 0;

  /** Sets the target position of multiple robot servos at once.
    * \param motors number of motors to be moved
    * \param id list of IDs of the joints to be moved
    * \param pos list of target positions
    * \return false if target could not be set */
  virtual bool set_pos (unsigned char motors, unsigned char* id, int* pos) throw () = 0;

  /** Sets the target position and active speed of multiple robot servos at once.
    * \param motors number of motors to be moved
    * \param id list of IDs of the joints to be moved
    * \param pos list of target positions
    * \param speed list of target active speeds
    * \return false if target could not be set */
  virtual bool set_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw () = 0;

protected:
  int	_verbosity, _delta_t, _default_speed;
  bool _hold;
};

#endif

