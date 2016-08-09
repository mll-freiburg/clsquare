/*
clsquare - closed loop simulation system
Copyright (c) 2012, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef _PARRALELPOLE_DYNAMICS_H_
#define _PARRALELPOLE_DYNAMICS_H_

class ParallelPoleDynamics
{
 public:
  virtual void next_state (const double* state, const double* action, double* next_state) = 0;
  virtual void notify_episode_starts (const double* state) {};

  void set_mass (const double mc, const double mp1, const double mp2);
  void set_length (const double lt, const double lp1, const double lp2);
  void set_friction (const double fc, const double fp1, const double fp2);
  void set_double (bool dbl);
  void set_delta (double delta);

 protected:
  double _MASS_POLE1, _MASS_POLE2, _MASS_CART;
  double _LENGTH_POLE1, _LENGTH_POLE2, _LENGTH_TRACK;
  double _FRIC_POLE1, _FRIC_POLE2, _FRIC_CART;
  double _DELTA;
  bool   _DOUBLE;
};

#endif

