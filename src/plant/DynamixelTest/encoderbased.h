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

#ifndef _DYNAMIXELENCTEST_PLANT_H_
#define _DYNAMIXELENCTEST_PLANT_H_

#include "plant.h"
#include <sys/time.h>
#include "dynamixel.h"

#define MAX_NUM_AX12 20

/**
  * Tests the functionality of Dynamixel servos by trying to move them to a specified goal position.
  * Primarily to be used with CenterControl, but should work with any other controller.
  *
  * The task can be configured by the following parameters:
  * \li \b servos (int, default 1): number of servos to be used
  * \li \b ids (\e servos \f$\times\f$ int, default 0 ... \e servos-1):
  *     list of ids of the servos to be used
  * \li \b torque (\e servos \f$\times\f$ int, default 100): torque
  *     of all servos
  * \li \b start (\e servos \f$\times\f$ int, default 512): initial
  *     encoder positions for all servos (plant ignores
  *     Input module)
  * \li \b scaling (\e servos \f$\times\f$ int, default 100): value
  *     that the action is multiplied with
  * \li \b delta_t (int, default 100): expected cycle duration;
  *     if communication takes longer, a warning will be given
  *
  * Depending on how the servos have been set up, the following
  * parameters may have to be adjusted:
  * \li \b baudrate (int, default 1000000): baudrate that the
  *     servos have been set to
  * \li \b use_ftd2xx (bool, default \e false): if set to \e true,
  *     will communicate through the FTD2xx library rather than
  *     the default libftdi.
  * \li \b custom_firmware (bool, default \e false): if set to
  *     \e true, will assume the servos run the Morpheus
  *     firmware rather than the default Robotis one.
  *
  * @ingroup PLANT
  * @ingroup HARDWARE
  * @author Thomas Lampe
  **/
class DynamixelEncoderTest : public Plant {
  public:
  	DynamixelEncoderTest();
	  ~DynamixelEncoderTest();

  	// specification as per plant.h
	  bool get_next_plant_state (const double *current_state, const double *current_action, double *next_state);
  	bool get_measurement (const double *state, double *observation);
	  bool init (int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 
  	void deinit ();
	  void notify_episode_starts ();
  	bool check_initial_state (double* initial_state);

  protected:
    
	  DynamixelDevice *_base;
  	int _i, _delta_t, _goal[MAX_NUM_AX12], _tmpe, _tmpv, _null[MAX_NUM_AX12], _torque[MAX_NUM_AX12], _scale[MAX_NUM_AX12], _tmpa, _speed[MAX_NUM_AX12], _pos[MAX_NUM_AX12];
	  timeval _start, _stop;
    bool _morph;
  	long int _remaining;
    unsigned char _servos, _id[MAX_NUM_AX12];
};

#endif

