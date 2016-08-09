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

#include "ftdxxdevice.h"

#define FXXCHECK(x) {if (_status != FT_OK) {FTDERROR("Error (code " << _status << "): " << x); return false;}}

bool FTDXXDevice::connect (int baudrate) throw ()
{
#ifdef FOUND_ftd2xx

  disconnect();

  _status = FT_SetVIDPID(0x0403, 0x6001);
  FXXCHECK("Could not set IDs.");

  _status = FT_Open(0, &_handle);
  FXXCHECK("Could not open handle.");

  _status = FT_SetBaudRate(_handle, baudrate);
  FXXCHECK("Could not set baudrate.");

  _status = FT_SetLatencyTimer(_handle, 0);
  FXXCHECK("Unable to set latency timer.");

  _status = FT_Purge(_handle, FT_PURGE_RX | FT_PURGE_TX);
  FXXCHECK("Unable to purge buffers.");

  _initialized = true;
  FTDLOG("Connection successful.");
  return true;

#else

  _initialized = false;
  FTDERROR("cannot connect FTD2xx device: compiled without required libraries");
  return false;

#endif
}

bool FTDXXDevice::disconnect () throw ()
{
  if (!_initialized) return false;
  _initialized = false;
  FTDLOG("Closing FTDI bus.");
#ifdef FOUND_ftd2xx
  FT_Close(_handle);
  FXXCHECK("Unable to close handle.");
#endif
  return true;
}

int FTDXXDevice::communicate (unsigned char* packet, unsigned char* buffer, int send, int receive, bool twice) throw ()
{
  if (!_initialized) return false;
  _bytes = _totalbytes = 0;

#ifdef FOUND_ftd2xx
  _t = _timeout;
  _status = FT_Write(_handle, &packet[0], send, &_bytes);
  FXXCHECK("Failed to write data.");
  while (_totalbytes < (unsigned int)receive && _t > 0) {
    _bytes = 0;
    usleep(_latency);
    FT_GetStatus(_handle, &_temp_rx, &_temp_dword, &_temp_dword);
    if (_temp_rx != 0) {
      _status = FT_Read(_handle, &buffer[_totalbytes], _temp_rx, &_bytes);
      FXXCHECK("Failed to read data.");
    }
    _totalbytes += _bytes;
    _t--;
  }
#endif
  return _totalbytes;
}

