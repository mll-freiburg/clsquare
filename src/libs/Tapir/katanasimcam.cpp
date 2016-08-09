/*
libTapir - Tapir vision toolkit interface
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

#include "katanasimcam.h"
#include <cstring>
#include <cmath>

KatanaSimCam::KatanaSimCam (FLIBase* robot, double* obj, bool draw)
{
  _draw = draw;
  _robot = robot;
  for (_i=0; _i<3; _i++)
    _object_xyz[_i] = obj[_i];
  currentObject.reset();
  _current.push_back(TapirObject());
}

void KatanaSimCam::set_object_position (double* obj)
{
  for (_i=0; _i<3; _i++)
    _object_xyz[_i] = obj[_i];
}

void KatanaSimCam::update () throw ()
{
  // data
  ///< \todo consider offsets, focal point not at gripper center and not straight
  _pose = _robot->get_pose();

  // generate axis rotation matrices
#define CELSET(mat,xxx,yyy,val) \
  mat[(xxx)%3][(yyy)%3] = val;
#define ROTMAT(mat,ang,axs) \
  for (_i=0; _i<9; _i++) CELSET(mat,_i,_i/3,0.); \
  CELSET(mat,axs,axs,1.); \
  CELSET(mat,axs+1,axs+1,cos(ang)); \
  CELSET(mat,axs+2,axs+2,cos(ang)); \
  CELSET(mat,axs+1,axs+2,sin(ang)); \
  CELSET(mat,axs+2,axs+1,-sin(ang));
  ROTMAT(_rot1, _pose[5], 2);
  ROTMAT(_rot2, _pose[4], 0);
  ROTMAT(_rot3, _pose[3], 2);

  // compute ZYZ rotation matrix
#define MATMUL(xxx,yyy,zzz) \
  for (_i=0; _i<3; _i++) \
    for (_j=0; _j<3; _j++) { \
      zzz[_i][_j] = 0.; \
      for (_k=0; _k<3; _k++) \
        zzz[_i][_j] += xxx[_i][_k] * yyy[_k][_j]; \
  }
  MATMUL(_rot1, _rot2,_rot12);
  MATMUL(_rot12,_rot3,_roto);

  // apply camera translation
  for (_i=0; _i<3; _i++) {
    _d[_i] = 0.;
    for (_j=0; _j<3; _j++)
      _d[_i] += _roto[_i][_j] * (_object_xyz[_j] - _pose[_j]);
  }

  // check if object is in front of camera
  currentObject.known = _current[0].known = false;
  if (_d[2] > 0.) {

    // project object onto sensor
    _d[0] *= 320. / _d[2];
    _d[1] *= 320. / _d[2];

    // check if projected object within camera resolution
    if (fabs(_d[0]) <= 320. && fabs(_d[1]) <= 240) {
      currentObject.pos.x = _current[0].x = _d[0];
      currentObject.pos.y = _current[0].y = _d[1];
      _current[0].x += 320.;
      _current[0].y += 240.;
      currentObject.known = _current[0].known = true;
    }
  }

  if (_draw) {
    // visualize projected object
    int bx=4, by=10;
    for (int a=0; a<641; a+=bx) std::cout << '-';
    std::cout << "--" << std::endl;
    for (int b=0; b<481; b+=by) {
      std::cout << '|';
      for (int a=0; a<641; a+=bx)
        std::cout << (_current[0].known && fabs(_current[0].x-a)<bx && fabs(_current[0].y-b)<by ? 'X' : a==320&&b==240 ? '+' : ' ');
      std::cout << '|' << std::endl;
    }
    for (int a=0; a<641; a+=bx) std::cout << '-';
    std::cout << "--" << std::endl;
  }
}

