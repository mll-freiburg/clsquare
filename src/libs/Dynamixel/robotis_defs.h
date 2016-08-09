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

#ifndef _ROBOTISPARAMS_H_
#define _ROBOTISPARAMS_H_

/** Defines parameters of Dynamixel servos running the official firmware that can be set
  * through RobotisBase::set_param(Robotis::param,int).
  * Parameter values < 0x50 are identical to their address in the servo memory.
  * Values >= 0x50 are used for parameters that occupy two bytes in the memory,
  * and their position is that of the value - 0x50. */
namespace Robotis
{
 enum param {

  /** Model number (read-only).
    * Should be 0x0c (12) for AX-12, 0x40 (64) for RX-64. */
  MODEL =  0x50 + 0x00,

  /** Version of the motor's firmware (read-only). */
  FIRMWARE = 0x02,

  /** Current angular position of the Dynamixel actuator output (read-only). */
  PRESENT_POSITION = 0x50 + 0x24,

  /** Current angular velocity of the Dynamixel actuator output (read-only). */
  PRESENT_SPEED = 0x50 + 0x26,

  /** The magnitude of the load on the operating Dynamixel actuator (read-only). */
  PRESENT_LOAD = 0x50 + 0x28,

  /** The voltage currently applied to the Dynamixel actuator (read-only).
    * The value is 10 times the actual voltage. For example, 10V is represented as 100 (0x64). */
  PRESENT_VOLTAGE = 0x2a,

  /** The internal temperature of the Dynamixel actuator in degrees Celsius (read-only). */
  PRESENT_TEMPERATURE = 0x2b,

  /** Set to 1 when the Dynamixel actuator is moving by its own power (read-only). */
  MOVING = 0x2e,

  /** The unique ID number assigned to each Dynamixel actuators for identifying them [0:150].
    * Different IDs are required for each Dynamixel actuators that are on the same network. */
  SERVO_ID = 0x03,

  /** Determines the communication speed [0:254].
    * The computation is done by the following formula: Speed (BPS) = 2000000 / (Robotis::BAUDRATE + 1) */
  BAUDRATE = 0x04,

  /** The time it takes for the status packet to return after the instruction packet is sent [0:254].
    * The delay time is given by 2uSec x Robotis::LATENCY value. */
  LATENCY = 0x05,

  /** Sets the Dynamixel actuator's lower operating angle range [0:1023].
    * The goal position needs to be within the range of: Robotis::CW_ANGLE_LIMIT <= Goal Position <=
    * Robotis::CCW_ANGLE_LIMIT. An Angle Limit Error will occur if the goal position is set outside
    * this range set by the operating angle limits. */
  CW_ANGLE_LIMIT = 0x50 + 0x06,

  /** Sets the Dynamixel actuator's upper operating angle range [0:1023].
    * \copydetails Robotis::CW_ANGLE_LIMIT */
  CCW_ANGLE_LIMIT = 0x50 + 0x08,

  /** The upper limit of the Dynamixel actuator’s operating temperature.
    * If the internal temperature of the Dynamixel actuator gets higher than this value, the
    * Overheating Error Bit (bit 2 of the status packet) will return the value 1, and an alarm
    * will be set by Robotis::ALARM_LED and Robotis::ALARM_SHUTDOWN. The values are in degrees Celsius.
    * Accepted values vary between different types of Dynamixel servos. For the AX-12, the range is
    * [0:150], for the RX-64 [10:99]. */
  MAX_TEMPERATURE = 0x0b,

  /** The lower limit of the Dynamixel actuator’s operating voltage [50:250].
    * If Robotis::PRESENT_VOLTAGE is out of the specified range, a Voltage Range Error Bit
    * (bit 0 of the status packet) will return the value 1, and an alarm will be set by
    * Robotis::ALARM_LED and Robotis::ALARM_SHUTDOWN. The values are 10 times the actual voltage value. For
    * example, if the Robotis::MIN_VOLTAGE value is 80, then the lower voltage limit is set to 8V. */
  MIN_VOLTAGE = 0x0c,

  /** The upper limit of the Dynamixel actuator’s operating voltage [50:250].
    * \copydetails Robotis::MIN_VOLTAGE */
  MAX_VOLTAGE = 0x0d,

  /** The maximum torque output for the Dynamixel actuator (EEPROM) [0:1023].
    * When this value is set to 0, the Dynamixel actuator enters the free run mode. There are
    * two locations where this maximum torque limit is defined; in the EEPROM (Robotis::MAX_TORQUE)
    * and in the RAM (Robotis::TORQUE_LIMIT). When the power is turned on, the maximum torque
    * limit value defined in the EEPROM is copied to the location in the RAM. The torque of
    * the Dynamixel actuator is limited by the values located in the RAM. */
  MAX_TORQUE = 0x50 + 0x0e,

  /** Determines whether the Dynamixel actuator will return a status packet after receiving an
    * instruction packet [0:2].
    * \li 0: do not respond to any instructions
    * \li 1: respond only to READ_DATA instructions
    * \li 2: respond to all instructions
    * 
    * In the case of an instruction which uses the Broadcast ID (0XFE) the status packet will
    * not be returned regardless of the Robotis::STATUS_RETURN_LEVEL 0x10 value. */
  STATUS_RETURN_LEVEL = 0x10,

  /** If the corresponding Bit is set to 1, the LED blinks when an Error occurs [0:127].
    * \li Bit 6: if set to 1, the LED blinks when an Instruction Error occurs
    * \li Bit 5: if set to 1, the LED blinks when an Overload Error occurs
    * \li Bit 4: if set to 1, the LED blinks when a Checksum Error occurs
    * \li Bit 3: if set to 1, the LED blinks when a Range Error occurs
    * \li Bit 2: if set to 1, the LED blinks when an Overheating Error occurs
    * \li Bit 1: if set to 1, the LED blinks when an Angle Limit Error occurs
    * \li Bit 0: if set to 1, the LED blinks when an Input Voltage Error occurs
    * 
    * This function operates following the OR logical operation of all bits. For example, if the
    * value is set to 0X05, the LED will blink when an Input Voltage Error occurs or when an
    * Overheating Error occurs. Upon returning to a normal condition from an error state, the
    * LED stops blinking after 2 seconds. */
  ALARM_LED = 0x11,

  /** If the corresponding Bit is set to a 1, the Dynamixel actuator's torque will be turned off
    * when an error occurs [0:127].
    * \li Bit 6: if set to 1, torque off when an Instruction Error occurs
    * \li Bit 5: if set to 1, torque off when an Overload Error occurs
    * \li Bit 4: if set to 1, torque off when a Checksum Error occurs
    * \li Bit 3: if set to 1, torque off when a Range Error occurs
    * \li Bit 2: if set to 1, torque off when an Overheating Error occurs
    * \li Bit 1: if set to 1, torque off when an Angle Limit Error occurs
    * \li Bit 0: if set to 1, torque off when an Input Voltage Error occurs
    * 
    * This function operates following the OR logical operation of all bits. However, unlike
    * the Alarm LED, after returning to a normal condition, it maintains the torque off status.
    * To recover, Robotis::TORQUE_ENABLE needs to be reset to 1. */
  ALARM_SHUTDOWN = 0x12,

  /** When the power is first turned on, the Dynamixel actuator enters the Torque Free Run
    * condition (zero torque); setting this value to 1 enables the torque [0:1]. */
  TORQUE_ENABLE = 0x18,

  /** The LED turns on when set to 1 and turns off if set to 0 [0:1]. */
  LED = 0x19,

  /** The compliance of the Dynamixel actuator is defined by setting the compliance Margin
    * and Slope [0:254]. This feature can be utilized for absorbing shocks at the output shaft. */
  CW_COMPLIANCE_MARGIN = 0x1a,
  /// \copydoc Robotis::CW_COMPLIANCE_MARGIN 
  CCW_COMPLIANCE_MARGIN = 0x1b,
  /// \copydoc Robotis::CW_COMPLIANCE_MARGIN 
  CW_COMPLIANCE_SLOPE = 0x1c,
  /// \copydoc Robotis::CW_COMPLIANCE_MARGIN 
  CCW_COMPLIANCE_SLOPE = 0x1d,

  /** Requested angular position for the Dynamixel actuator output to move to [0:1023].
    * Setting this value to 0x3ff moves the output shaft to the position at 300 degrees. */
  GOAL_POSITION = 0x50 + 0x1e,

  /** Sets the angular velocity of the output moving to the Goal Position [0:1023].
    * Setting this value to its maximum value of 0x3ff moves the output with an angular
    * velocity of 114 RPM, provided that there is enough power supplied. The lowest velocity
    * is when this value is set to 1. When set to 0, the velocity is the largest possible
    * for the supplied voltage, e.g. no velocity control is applied. */
  MOVING_SPEED = 0x50 + 0x20,

  /** The maximum torque output for the Dynamixel actuator (RAM) [0:1023].
    * \copydetails Robotis::MAX_TORQUE */
  TORQUE_LIMIT = 0x50 + 0x22,

  /** Set to 1 when an instruction is assigned by the REG_WRITE command; set to 0
    * after it completes the assigned instruction by the Action command [0:1]. */
  REGISTERED_INSTRUCTION = 0x2c,

  /** If set to 1, only Robotis::TORQUE_ENABLE, Robotis::LED, Robotis::CW_COMPLIANCE_MARGIN,
    * Robotis::CCW_COMPLIANCE_MARGIN, Robotis::GOAL_POSITION, Robotis::MOVING_SPEED and
    * Robotis::TORQUE_LIMIT can be written to and other areas cannot [0:1].
    * Once locked, it can only be unlocked by turning the power off. */
  LOCK = 0x2f,

  /** The minimum current supplied to the motor during operation [0:1023].
    * The initial value is set to 0x20 and its maximum value is 0x3ff. */
  PUNCH = 0x50 + 0x30
 };
}

#endif

