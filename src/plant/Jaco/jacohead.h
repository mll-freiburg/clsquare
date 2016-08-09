/*
clsquare - closed loop simulation system
Copyright (c) 2010-2012 Machine Learning Lab, 
Prof. Dr. Martin Riedmiller, University of Freiburg

Author: Thomas Lampe

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

#ifndef _JACO_FACE_PLANT_H_
#define _JACO_FACE_PLANT_H_

#include "jacovision.h"
#include "morpheus.h"
#include <sys/time.h>

/** Provides a general-purpose base plant for use with a Kinova Jaco robotic arm with
  * a set of Dynamixel servos and a set of cameras.
  * It is based on the JacoVisionPlant class, from which it inherits state and action
  * description, configuration settings and functionality.
  *
  * The state is extended with the encoder positions of the servos used. Servos to
  * use need to be declared through the parameter \b servo_ids, which lists the
  * servos' unique IDs.
  *
  * The action is extended with as many dimensions, the interpretation of which
  * depends on the parameter \b servo_mode:
  *  - \e position [0:1023]: interprets the action as a target encoder position
  *  - \e direction [-1:+1]: interprets the action as a direction of rotation
  *  - \e incremental [-1023:1023]: interprets the action as an increment or
  *       decrement to be added to the current position
  *  - \e pwm [0:2000]: PWM signal to be sent to the servo, with 1000 being neutral
  *
  * Additional parameters are:
  *  - \b servo_min, \b servo_max (\f$n \times\f$int): maximum and minimum
  *    encoder positions each servo is allowed to take
  *    \warning when using PWM mode, the servos may violate the safety constraints
  *             by as much as they can move during a single cycle
  *  - \b servo_speed (int, default 10): movement speed to use for non-PWM modes
  *  - \b servo_tolerance (int, default 10): maximum allowed deviation of
  *     actual and intended position in check_initial_state(), in encoder steps.
  *  - \b servo_delta (int, default 10): intended total cycle time in ms; if
  *     servo communication plus JacoVisionPlant::get_next_plant_state() take less
  *     than this, the plant will wait for the remaining time
  *  - \b servo_baudrate (int, default 1000000): baud rate at which the
  *     servos operate
  *  - \b servo_ftd2xx (bool, default \e true): communication library to use;
  *     \e false for libftdi, \e true for FTD2XX
  * 
  * @ingroup PLANT
  * @ingroup HARDWARE
  * @author Thomas Lampe
  **/
class JacoFacePlant : public JacoVisionPlant {
public:
  JacoFacePlant() {};
  ~JacoFacePlant() {};

  // specification as per plant.h
  bool get_next_plant_state (const double *current_state, const double *current_action, double *next_state);
  bool get_measurement (const double *state, double *measurement);
  bool init (int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 
  bool check_initial_state (double* initial_state);
  void deinit ();

protected:
  void parse_action (const double action, const double state, int& pos, int& speed);

  MorpheusDevice *_base;
  unsigned char *_id, _servos;
  int *_hpos, *_hspeed, *_smin, *_smax, _delta_s, _hsdef, _htolerance;
  long int _remaining;
  double _tmp;
  struct {
    int sdim, mdim, adim;
  } _visionplant;
  enum {PWM, Increment, Position, Direction} _hmode;
  timeval _starttime, _stoptime;
};

#endif

