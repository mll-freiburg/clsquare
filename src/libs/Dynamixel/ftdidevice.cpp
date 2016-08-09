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

#include "ftdidevice.h"

bool FTDIDevice::connect (int baudrate) throw ()
{
#ifdef FOUND_ftdi

  int ret, i;
  disconnect();

  ftdi_init (&_ftdic);
  while ((ret = ftdi_usb_open(&_ftdic, 0x0403, 0x6001)) < 0)
    FTDERROR("unable to open ftdi device: " << ret << " (" << ftdi_get_error_string(&_ftdic) << ")");

  for (i=0, ret=-1; i<10 && ret!=0; i++)
    ret = ftdi_usb_reset(&_ftdic);
  if (ret != 0) {
    FTDERROR("unable to reset ftdi device.");
    return false;
  }

  for (i=0, ret=-1; i<10 && ret!=0; i++)
    ret = ftdi_set_baudrate(&_ftdic, baudrate);
  if (ret != 0) {
    FTDERROR("unable to set baud rate.");
    return false;
  }

  if (_latency > 0) for (i=0, ret=-1; i<10 && ret!=0; i++)
      ret = ftdi_set_latency_timer(&_ftdic, _latency);
  if (ret != 0) {
    FTDERROR("unable to set latency timer.");
    return false;
  }

  for (i=0, ret=-1; i<10 && ret!=0; i++)
    ret = ftdi_usb_purge_buffers(&_ftdic);
  if (ret != 0) {
    FTDERROR("unable to purge buffers.");
    return false;
  }

  _initialized = true;
  FTDLOG("Connection successful.");
  return true;

#else

  _initialized = false;
  FTDERROR("cannot connect FTDI device: compiled without required libraries");
  return false;

#endif
}

bool FTDIDevice::disconnect () throw ()
{
  if (!_initialized) return false;
  _initialized = false;
  FTDLOG("Closing FTDI bus.");
#ifdef FOUND_ftdi
  ftdi_usb_close(&_ftdic);
#endif
  return true;
}

/************** Communication **************/

int FTDIDevice::communicate (unsigned char* packet, unsigned char* buffer, int send, int receive, bool twice) throw ()
{
  if (!_initialized) return false;

  _t = _timeout;

  // if expected_bytes==6 we are waiting for ping; why does FTDI never send one back? FTD2xx does!
  if (receive == 6) receive = 0;

#ifdef FOUND_ftdi
  while (receive > 0 && _bytes == 0 && _t > 0) {
    ftdi_write_data(&_ftdic, &packet[0], send);
    if (twice)
      ftdi_write_data(&_ftdic, &packet[0], send);
    usleep(_latency);
    _bytes = ftdi_read_data(&_ftdic, &buffer[0], 100);
    _t--;
  }
#endif
  if (receive > 0 && (_bytes == 0 || _bytes % receive != 0)) {
    FTDERROR("communication failure: " << _bytes << " bytes received, should be a multiple of " << receive << ".");
    return 0;
  }
  return _bytes;
}

