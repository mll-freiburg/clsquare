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

#ifndef _KEYCONTROL_H_
#define _KEYCONTROL_H_

#include "controller.h"
#include <termios.h>


/** Allows controlling a plant via keyboard.
  * The action may be 10-dimensional at most.
  * Pressing a number on the keyboard toggles the matching action between
  * 0 and 1, or between -1 and 1 if the config entry \b negative has been set
  * to \e true.
  * Actions start at the lower value by default.
  *
  * @ingroup CONTROLLER
  * @ingroup STATIC
  * @author Thomas Lampe
  **/
class KeyControl : public Controller
{
 public:
  bool get_action (const double* state, double* action);
  bool init (const int observed_state_dim, const int action_dim, double deltat, const char* fname=0, const char* chapter=0);
  void deinit();

  KeyControl() {};
  ~KeyControl() {};

 protected:

  char _cmd;
  int _adim, _fd, _i;
  double _mem[20];
  bool _negative;
  termios _dflags;
};

#endif
