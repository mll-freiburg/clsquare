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

#ifndef _FTDXXBASE_H_
#define _FTDXXBASE_H_

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include "ftdevice.h"
#ifdef FOUND_ftd2xx
#include "ftd2xx.h"
#endif

/** Implements a connection to a Dynamixel device through an FTDI interface,
  * using the FTD2xx library. FTD2xx is proprietary and not available for
  * as many platforms as the open libftdi, but considerably faster. */
class FTDXXDevice : public FTDevice
{
public:

  FTDXXDevice () throw () {
    _timeout = 20;
    _latency = 300;
    _initialized = false;
  };
  ~FTDXXDevice () throw () {
    ;
  }

  bool connect (int baudrate) throw ();
  bool disconnect () throw ();
  int communicate (unsigned char* packet, unsigned char* buffer, int send, int receive, bool twice=false) throw ();

protected:

#ifdef FOUND_ftd2xx
  FT_HANDLE	_handle;
  FT_STATUS	_status;
  DWORD	_temp_rx, _temp_dword, _bytes, _totalbytes;
#else
  int _temp_rx, _temp_dword, _bytes, _totalbytes;
#endif

  int _t;
  bool _initialized;

};

#endif

