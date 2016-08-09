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

#include "dynamixel.h"
#include "ftdidevice.h"
#include "ftdxxdevice.h"

/************** Setup **************/

DynamixelDevice::DynamixelDevice () throw ()
{
  _verbosity = 1;
  _baudrate  = 1000000;
  _latency   = -1;
  _timeout   = -1;
  _validate_response = true;
  _com = new FTDIDevice();
  _com->set_verbosity(_verbosity);
}

bool DynamixelDevice::test_response (int id) throw ()
{
  if (id < 0) {
    DYNALOG("Negative ID specified, skipping servo.");
    return true;
  }
  if (!get_pos(id,_tmpp,_tmps)) {
    DYNAWARNING("servo " << id << " not responsive!");
    return false;
  }
  return true;
}

bool DynamixelDevice::set_param (DynamixelDevice::param param, int value) throw ()
{
  switch (param) {
  case DynamixelDevice::BAUDRATE:
    _baudrate = value;
    break;
  case DynamixelDevice::LATENCY:
    _latency = value;
    _com->set_latency(value);
    break;
  case DynamixelDevice::VERBOSITY:
    _verbosity = value;
    _com->set_verbosity(_verbosity);
    break;
  case DynamixelDevice::TIMEOUT:
    _timeout = value;
    _com->set_timeout(value);
  case DynamixelDevice::VALIDATE:
    if (value > 0) _validate_response = true;
    break;
  case DynamixelDevice::COMTYPE:
    switch (value) {
      case FTDevice::FTD2XX:
        _com = new FTDXXDevice();
        break;
      case FTDevice::FTDI:
        _com = new FTDIDevice();
        break;
      default:
        DYNAERROR("unknown FTD device type.");
        return false;
    }
    _com->set_verbosity(_verbosity);
    _com->set_timeout(_timeout);
    _com->set_latency(_latency);
  default:
    DYNAWARNING("unknown parameter");
    return false;
  }
  return true;
}

