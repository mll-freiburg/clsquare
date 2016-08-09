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

#ifndef _FTDEVICE_H_
#define _FTDEVICE_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#define FTDERROR(x) {if (_verbosity > 0) std::cerr << "Error: " << x << std::endl;}
#define FTDWARNING(x) {if (_verbosity > 1) std::cerr << "Warning: " << x << std::endl;}
#define FTDLOG(x) {if (_verbosity > 2) std::cout << x << std::endl;}

/** Implements a connection to a Dynamixel device through an FTDI interface,
  * using libftdi. Communication is much slower than with the FTD2xx library,
  * but libftdi is open source and available for more platforms. */
class FTDevice
{
public:

  FTDevice () throw () {
    _verbosity = 2;
  };
  ~FTDevice () throw () {
    ;
  }
  enum type {FTDI, FTD2XX} _type;

  virtual bool connect (int baudrate) throw () = 0;
  virtual bool disconnect () throw () = 0;
  virtual int communicate (unsigned char* packet, unsigned char* buffer, int send, int receive, bool twice=false) throw () = 0;
  inline void set_verbosity (int verbosity) {
    _verbosity = verbosity;
  }
  inline void set_timeout (int timeout) {
    if (timeout > -1) _timeout = timeout;
  }
  inline void set_latency (int latency) {
    if (latency > -1) _latency = latency;
  }

protected:

  int _verbosity, _timeout, _latency;
};

#endif

