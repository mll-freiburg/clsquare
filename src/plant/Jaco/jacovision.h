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

#ifndef _JACOVISION_PLANT_H_
#define _JACOVISION_PLANT_H_

#include "jacobase.h"
#include "tapir.h"
#include <vector>

class TapirDummyCam : public TapirDetector
{
public:
  void update () throw () {};
};

/** Provides a general-purpose base plant for use with a Kinova Jaco robotic arm with a camera.
  * Vision data is provided by the Tapir 3 tool and can in principle be of any type of camera.
  * The plant is identical to the JacoBasePlant with a few additions:
  * \li The state and measurement are extended with the data of a list of objects detected by Tapir.
  *     This results in 4 additional features per camera, representing the last known horizontal and
  *     vertical position, the size of the object (-1 if object not visible), as well as the rotation
  *     of each object.
  * \li Initial states will be rejected if there is no object visible in a camera. This behavior
  *     may be suppressed for any camera by setting the parameter \b required, which consists of
  *     a list of boolean values.
  * \li The name of the Tapir configuration files to be used (required mostly for shared memory
  *     setup) may be set through the parameters \b cam_i, with \e i being a continuous index starting
  *     at 1. If any such camera is set to \e false, the matching camera will always be treated as if
  *     no object were visible, but space will still be reserved for it in the state vector.
  *     \note \e cam_1 is set to \e tapir1.cfg by default and \e cam_2 to \e false, or to \e tapir2.cfg
  *     if the deprecated parameter \b binocular is set to \e true; consequently, there are always at
  *     least 8 additional state dimensions over the JacoBasePlant.
  * 
  * @ingroup PLANT
  * @ingroup HARDWARE
  * @author Thomas Lampe
  **/
class JacoVisionPlant : public JacoBasePlant {
public:
  JacoVisionPlant() {};
  ~JacoVisionPlant() {};

  // specification as per plant.h
  bool get_next_plant_state (const double *current_state, const double *current_action, double *next_state);
  bool get_measurement (const double *state, double *measurement);
  bool init (int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 
  bool check_initial_state (double* initial_state);

protected:

  bool get_camera_data (TapirDetector *cam, double *dest, const double *last);

  std::vector<TapirDetector*> _cam;
  int _cams;
  bool *_required;
};

#endif

