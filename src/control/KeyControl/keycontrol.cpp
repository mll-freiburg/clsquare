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

#include "keycontrol.h"
#include "valueparser.h"
#include <cmath>
#include <stdio.h>
#include <termios.h>
#include "unistd.h"

bool KeyControl::get_action (const double *state, double *action)
{
  for (_i=0; _i<_adim; _i++)
    _mem[_i] = _mem[_i+10];

  _cmd = 0;
  if (read(_fd, &_cmd, 1) == 0)
    EOUT("Could not read key.");

  switch (_cmd) {
    case 'x':
    case 'q':
      IOUT("Exit signal sent by user.");
      return false;
    case '0':
      _cmd += 10;
    default:
      break;
  }

  _cmd -= '0';
  if (_cmd > 0 && _cmd < 11)
    _mem[_cmd-1] = _negative ? _mem[_cmd+9] * -1. : abs(_mem[_cmd+9]-1.);

  for (_i=0; _i<_adim; _i++)
    _mem[_i+10] = action[_i] = _mem[_i];

  return true;
}

bool KeyControl::init (const int observed_state_dim, const int action_dim, double delta_t, const char *fname, const char* chapter)
{
  _adim = action_dim;
  if (action_dim > 10) {
    EOUT("KeyControl supports only up to 10 action dimensions.");
    return false;
  }

  _fd = fileno(stdin);
  termios flags;

  if (tcgetattr(_fd, &flags) < 0) {
    EOUT("Error: could not set up key capture.");
    return false;
  }
  tcgetattr(_fd, &_dflags);

  flags.c_lflag &= ~ICANON;
  flags.c_lflag &= ~ECHO;
  flags.c_cc[VMIN] = 0;
  flags.c_cc[VTIME] = 0;

  if (tcsetattr(_fd, TCSANOW, &flags) < 0) {
    EOUT("Error: could not set up key capture.");
    return false;
  }

  ValueParser vp(fname, chapter==NULL ? "Controller" : chapter);
  vp.get("negative", _negative, false);

  for (_i=0; _i<_adim; _i++)
    _mem[_i+10] = _negative ? -1. : 0.;

  return true;
}

void KeyControl::deinit ()
{
  tcsetattr(_fd, TCSANOW, &_dflags);
}

REGISTER_CONTROLLER(KeyControl, "Controller which allows controlling a plant via keyboard.")
