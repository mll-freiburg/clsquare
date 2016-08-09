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

#ifndef _MORPHEUSPARAMS_H_
#define _MORPHEUSPARAMS_H_

/** Defines parameters of AX-12 servos with the Morpheus custom firmware that can be set through
  * MorpheusDevice::set_param(Morpheus::param,int).
  * Parameter values are identical to their address in the servo memory. */
namespace Morpheus
{
 enum param {

  /** The unique ID number assigned to each Dynamixel actuators for identifying them [0:253].
    * Different IDs are required for each Dynamixel actuators that are on the same network. */
  SERVO_ID = 0x01,

  /** The margin in encoder steps, i.e. the distance from the edge to consider as limit to
    * avoid entering the dead zone of the servo encoder. */
  CTL_UP_LIMIT = 0x02,
  /// \copydoc Morpheus::CTL_UP_LIMIT 
  CTL_LW_LIMIT = 0x03,

  /** Constants to control the PID for the static positioning control loop.
    * Result = 1000 - ((KP \f$\times e_t\f$) + (KI x \f$\times \sum_{i=0}^t e_i\f$) / 100
    *  + (KD \f$\times \delta e\f$ */
  KPP = 0x04,
  /// \copydoc Morpheus::KPP
  KIP = 0x05,
  /// \copydoc Morpheus::KPP
  KDP = 0x06,

  /** Constants to control the PID for the dynamic positioning control loop.
    * Result = 1000 - ((KP \f$\times e_t\f$) + (KI x \f$\times \sum_{i=0}^t e_i\f$) / 100
    *  + (KD \f$\times \delta e\f$ */
  KPV = 0x07,
  /// \copydoc Morpheus::KPV
  KIV = 0x08,
  /// \copydoc Morpheus::KPV
  KDV = 0x09,

  /** Determines the communication speed [0:254].
    * The computation is done by the following formula: Speed (BPS) = 2000000 / (Morpheus::BAUDRATE + 1) */
  BAUDRATE = 0x0a,

  /** Servo position in encoder steps [0+Morpheus::LW_LIMIT : 1023-Morpheus::UP_LIMIT].
    * When reading, returns the current position; when writing, sets the target position. */
  POSITION = 0x0b + 0x50,

  /** Movement speed in encoder steps per 10ms [0:10].
    * When reading, returns the current position; when writing, sets the target position. */
  SPEED = 0x0d,

  /** The internal temperature of the Dynamixel actuator in degrees Celsius (read-only). */
  TEMPERATURE = 0x0e + 0x50,

  /** The current PWM to be used by the motor. */
  PWM = 0x10 + 0x50
 };
}

#endif

