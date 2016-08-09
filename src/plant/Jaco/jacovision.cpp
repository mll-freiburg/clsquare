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

#include "jacovision.h"
#include "valueparser.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

bool JacoVisionPlant::get_camera_data (TapirDetector *cam, double *dest, const double *last)
{
  if (cam != NULL) {
    cam->update();
    if (cam->get_object().known) {
      dest[0] = cam->get_object().x;
      dest[1] = cam->get_object().y;
      dest[2] = cam->get_object().size;
      dest[3] = cam->get_object().angle;
      if (dest[2] < 1) dest[2] = 1.;
      return true;
    }
  }
  dest[0] = last[0];
  dest[1] = last[1];
  dest[2] = -1000.;
  dest[3] = last[3];
  return false;
}

bool JacoVisionPlant::get_next_plant_state (const double *state, const double *action, double *next)
{
  if (!JacoBasePlant::get_next_plant_state(state, action, next)) return false;
  for (_i=0; _i<_cams; _i++)
    get_camera_data(_cam[_i], &next[_sdim+_i*4], &state[_sdim+_i*4]);
  //get_camera_data(_cam1, &next[_sdim],   &state[_sdim]);
  //get_camera_data(_cam2, &next[_sdim+4], &state[_sdim+4]);
  return true;
}

bool JacoVisionPlant::get_measurement (const double *state, double *measurement)
{
  if (!JacoBasePlant::get_measurement(state, measurement)) return false;
  for (_i=0; _i<4*_cams; _i++)
    measurement[_sdim+_i] = state[_sdim+_i];
  return true;
}

bool JacoVisionPlant::check_initial_state (double *state)
{
  if (!JacoBasePlant::check_initial_state(state)) return false;

  for (_i=0; _i<_cams; _i++)
    _cam[_i]->update();
  usleep(100000);

  double last[4] = {0., 0., 0., -1000.};
  for (_i=0; _i<_cams; _i++) {
    get_camera_data(_cam[_i], &state[_sdim+_i*4], last);
    if (_required[_i] && !_cam[_i]->get_object().known) {
      EOUT("No object visible at starting pose.");
      return false;
    }
  }
  return true;
}

#define SSSET(xxx) { ss.str(""); ss << xxx; };
bool JacoVisionPlant::init (int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  if (!JacoBasePlant::init(state_dim, measurement_dim, action_dim, delta_t, fname, chapter)) return false;

  ValueParser vp(fname, chapter==0?"Plant":chapter);
  std::stringstream ss;

  bool bino;
  vp.get("binocular", bino, false);

  for (_i=1; ; _i++) {
    char *config = new char[255];
    SSSET("cam_" << _i);
    if (vp.get(ss.str().c_str(), config, 255) < 1) {
      if (_i==2 && !bino)
        sprintf(config, "false");
      else if (_i < 3)
        sprintf(config, "tapir%d.cfg", _i);
      else break;
    }
    if (strcmp(config, "false") == 0) break;

    IOUT("Loading configuration for camera " << _i << " from file " << config);
    _cam.push_back(new TapirSharedMemoryDetector(config));
  }
  _cams = _cam.size();

  _required = new bool[_cams];
  for (_i=0; _i<_cams; _i++)
    _required[_i] = true;
  vp.get("required", _required, _cams);

  state_dim += 4 * _cams;
  measurement_dim += 4 * _cams;

  return true;
}

REGISTER_PLANT(JacoVisionPlant, "Visual servoing plant for a Kinova Jaco robotic arm.");

