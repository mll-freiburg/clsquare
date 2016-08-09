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

#include "morpheus.h"

///< \todo cover case speed=0, can lead to oscillation; firmware modification could handle it, but need solution for default

/************** Setup **************/

MorpheusDevice::MorpheusDevice () throw ()
{
  _packet[0] = 0xaa;
  _packet[3] = 0xfe;
  _motors = 50;
  _started = false;
  _leftover = 0;
  _missing = 0;
  _timeout = 15;
  _latency = 300;
}

MorpheusDevice::MorpheusDevice (DynamixelDevice* dev) throw ()
{
  _packet[0] = 0xaa;
  _packet[3] = 0xfe;
  _motors = 50;
  _started = false;
  _leftover = 0;
  _missing = 0;
  _timeout = 15;
  _latency = 300;
  _com = dev->get_base();
}

/************** READ_DATA **************/

bool MorpheusDevice::get_pos (unsigned char id, int& pos, int& speed) throw ()
{
  _packet[5] = 0x0b; // address
  _packet[6] = 0x03; // number of bytes to be read
  pos = -1;
  speed = 0;
  communicate(id, 2, 0x02, 3);
  for (_i=0; _i<_packetnum; _i++) {
    _l = _packetlist[_i];
    if (_buffer[_l+3] != id) continue;
    if (_buffer[_l+1] == 5) {
      pos   = _buffer[_l+4] + _buffer[_l+5] * 256;
      speed = _buffer[_l+6];
      return true;
    } else if (_buffer[_l+1] == 3 && _buffer[_l+4] == 0x08) {
      DYNAERROR("Servo " << (int)id << " reports NOK.");
    }
  }
  return false;
}

bool MorpheusDevice::get_param (unsigned char id, Morpheus::param param, int& value) throw ()
{
  _packet[5] = (unsigned char)param;

  // long param
  if (_packet[5] > 0x4f) {
    _packet[5] -= 0x50;
    _packet[6] = 2;
    communicate(id, 2, 0x02, 2);
    for (_i=0; _i<_packetnum; _i++) {
      _l = _packetlist[_i];
      if (_buffer[_l+3] != id) continue;
      if (_buffer[_l+1] == 4) {
        value = _buffer[_l+4] + _buffer[_l+5] * 256;
        return true;
      } else if (_buffer[_l+1] == 3 && _buffer[_l+4] == 0x08) {
        DYNAERROR("Servo " << (int)id << " reports NOK.");
      }
    }

  // short param
  } else {
    _packet[6] = 1;
    communicate(id, 2, 0x02, 1);
    for (_i=0; _i<_packetnum; _i++) {
      _l = _packetlist[_i];
      if (_buffer[_l+3] != id) continue;
      if (_buffer[_l+1] == 3) {
        value = _buffer[_l+4];
        return true;
      }
    }
  }
  return false;
}

/************** WRITE_DATA **************/

bool MorpheusDevice::set_pos (unsigned char id, int pos, int speed) throw ()
{
  _packet[5] = 0x0b; // address
  _packet[6] = LOWER(pos);
  _packet[7] = UPPER(pos);
  _packet[8] = speed < 0 ? 0 : speed > 10 ? 10 :speed;
  communicate(id, 4, 0x03, 1, true);
  for (_i=0; _i<_packetnum; _i++) {
    _l = _packetlist[_i];
    if (_buffer[_l+3] != id) continue;
    if (_buffer[_l+1] == 3)
      return _buffer[_l+4] == 0x04;
  }
  return false;
}

bool MorpheusDevice::set_pos (unsigned char id, int pos) throw ()
{
  _packet[5] = 0x0b; // address
  _packet[6] = LOWER(pos);
  _packet[7] = UPPER(pos);
  communicate(id, 3, 0x03, 1, true);
  for (_i=0; _i<_packetnum; _i++) {
    _l = _packetlist[_i];
    if (_buffer[_l+3] != id) continue;
    if (_buffer[_l+1] == 3)
      return _buffer[_l+4] == 0x04;
  }
  return false;
}

bool MorpheusDevice::set_id (unsigned char old_id, unsigned char new_id) throw ()
{
  if (!test_response(old_id)) {
    DYNAWARNING("Old ID does not exist.");
    return false;
  }
  if (test_response(new_id)) {
    DYNAWARNING("New ID already taken.");
    return false;
  }
  set_param(old_id,Morpheus::SERVO_ID,new_id);
  if (!test_response(new_id)) {
    DYNAWARNING("New ID not responsive.");
    return false;
  }
  return true;
}

bool MorpheusDevice::set_param (unsigned char id, Morpheus::param param, int value) throw ()
{
  switch (param) {
  case Morpheus::TEMPERATURE:
    DYNAERROR("Cannot write read-only data.");
    return false;
  default:
    _packet[5] = (unsigned char)param;
    if (_packet[5] > 0x4f) {
      _packet[5] -= 0x50;
      _packet[6] = LOWER(value);
      _packet[7] = UPPER(value);
      communicate(id, 3, 0x03, 1, true);
    } else {
      _packet[6] = value;
      communicate(id, 2, 0x03, 1, true);
    }
  }

  for (_i=0; _i<_packetnum; _i++) {
    _l = _packetlist[_i];
    if (_buffer[_l+3] != id) continue;
    if (_buffer[_l+1] == 3)
      return _buffer[_l+4] == 0x04;
  }
  return false;
}

/************** READ_DATA_SYNC **************/

bool MorpheusDevice::start (unsigned char& motors, unsigned char* ids) throw ()
{
  if (_started)
    return true;

  _com->set_timeout(1000);
  communicate(0xff, 0, 0x06, MORPHEUS_BUFFER_SIZE-5, false);
  _com->set_timeout(_timeout);

  if ((motors > 0 && motors != _packetnum) || _packetnum < 1) {
    DYNAERROR("Bus init failed.");
    return false;
  }

  for (_i=0; _i<_packetnum; _i++) {
    _l = _packetlist[_i];
    _ids[_i] = _buffer[_l+3];
  }
  _motors = motors = _packetnum;
  _started = true;
  DYNALOG("Bus init successful, found " << (int)_motors << " servos.");
  return true;
}

bool MorpheusDevice::get_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw ()
{
  if (!_started || motors < 1)
    return false;

  for (_i=0; _i<motors; _i++) {
    pos[_i] = -1;
    vel[_i] = 0;
  }

  communicate(0xff, 0, 0x07, 7*(_motors-1)+2);
  for (_i=0; _i<_packetnum; _i++) {
    _l = _packetlist[_i];
    if (_buffer[_l+1] == 4) {
      for (_k=0; _k<motors; _k++)
        if (id[_k] == _buffer[_l+3])
           pos[_k] = _buffer[_l+5] + _buffer[_l+4] * 256;
           ///< \todo speed missing
    } else if (_buffer[_l+1] == 3 && _buffer[_l+4] == 0x08) {
      DYNAERROR("Servo " << (int)_buffer[_l+3] << " reports NOK.");
    }
  }
  for (_k=0; _k<motors; _k++)
    if (pos[_i] == -1)
      return false;
  return true;
}

/************** WRITE_DATA_SYNC **************/

bool MorpheusDevice::set_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ()
{
  if (motors < 1) return false;
  _packet[5] = 0x0b; // address
  _packet[6] = 3; // bytes written per ID
  for (_i=0; _i<motors; _i++) {
    _packet[4*_i+7] = id[_i];
    _packet[4*_i+8] = LOWER(pos[_i]);
    _packet[4*_i+9] = UPPER(pos[_i]);
    _packet[4*_i+10] = speed[_i] < 0 ? 0 : speed[_i] > 10 ? 10 : speed[_i];
  }
  communicate(0xff, motors*4+2, 0x83, 0, true);
  return true;
}

bool MorpheusDevice::set_pos (unsigned char motors, unsigned char* id, int* pos) throw ()
{
  if (motors < 1) return false;
  _packet[5] = 0x0b; // address
  _packet[6] = 2; // bytes written per ID
  for (_i=0; _i<motors; _i++) {
    _packet[3*_i+7] = id[_i];
    _packet[3*_i+8] = LOWER(pos[_i]);
    _packet[3*_i+9] = UPPER(pos[_i]);
  }
  communicate(0xff, motors*3+2, 0x83, 0, true);
  return true;
}

bool MorpheusDevice::set_param (unsigned char motors, unsigned char* id, Morpheus::param param, int* value) throw ()
{
  if (motors < 1) return false;
  if (param == Morpheus::TEMPERATURE) {
    DYNAERROR("Cannot write read-only data.");
    return false;
  }
  
  _packet[5] = (unsigned char)param;
  if (_packet[5] > 0x4f) {
    _packet[5] -= 0x50;
    _packet[6] = 2; // bytes written per ID
    for (_i=0; _i<motors; _i++) {
      _packet[3*_i+7] = id[_i];
      _packet[3*_i+8] = LOWER(value[_i]);
      _packet[3*_i+9] = UPPER(value[_i]);
    }
    communicate(0xff, motors*3+2, 0x83, 0, true);
  } else {
    _packet[6] = 1; // bytes written per ID
    for (_i=0; _i<motors; _i++) {
      _packet[2*_i+7] = id[_i];
      _packet[2*_i+8] = value[_i];
    }
    communicate(0xff, motors*2+2, 0x83, 0, true);
  }
  return true;
}

/************** WRITE_AND_READ_SYNC **************/

bool MorpheusDevice::update_pos (unsigned char motors, unsigned char* id, int* pos, int* vel) throw ()
{
  if (!_started || motors < 1)
    return false;

  ///< \todo one-shot communication not stable
/*
  if (_motors > motors)
    DYNAWARNING("Requested data from " << motors << " servos, but there are " << _motors << " initialized on the bus. Returning only the first " << motors << " ones that responded.");
  if (_motors < motors)
    DYNAWARNING("Requested data from " << motors << " servos, but there are only " << _motors << " initialized on the bus. Position buffer will be filled only partially.");

  _packet[5] = 0x0b; // address
  _packet[6] = 3; // bytes written per ID
  for (_i=0; _i<motors; _i++) {
    _packet[4*_i+7] = id[_i];
    _packet[4*_i+8] = LOWER(pos[_i]);
    _packet[4*_i+9] = UPPER(pos[_i]);
    _packet[4*_i+10] = vel[_i] < 0 ? 0 : vel[_i] > 10 ? 10 : vel[_i];
  }
  _packet[4*motors+8] = 0xaa;
  _packet[4*motors+9] = 0x03;
  _packet[4*motors+10] = 0xff;
  _packet[4*motors+11] = 0xfe;
  _packet[4*motors+12] = 0x07;
  checksum(&_packet[4*motors+8]);
  if (!communicate(0xff, motors*4+2, 0x83, false, 2, motors, 1, 6))
    return false;
  for (_i=0; _i<_motors; _i++) {
    pos[_i] = -1;
    vel[_i] = -1;
    if (_i<_motors) {
      for (_k=0; _k<motors; _k++)
        if (id[_k] == _buffer[_i*7+3]) {
          pos[_i] = _buffer[_i*7+5] + _buffer[_i*7+4] * 256;
          vel[_i] = -1;
        }
    }
  }
*/

  set_pos(motors, id, pos, vel);
  usleep(_latency);
  get_pos(motors, id, pos, vel);
  return true;
}


/************** RESET **************/

bool MorpheusDevice::reset (unsigned char id) throw ()
{
  if (!test_response(id)) {
    DYNAWARNING("ID does not exist");
    return false;
  }
  if (test_response(1)) DYNAWARNING("ID 1 already taken; you might have to unplug the other servos to find the servo after it has been reset.");
  communicate(id, 0, 0x09, 0, true);
  return true;
}

/************** Helpers **************/

void MorpheusDevice::checksum (unsigned char* packet) throw ()
{
  packet[packet[1]+2] = 0;
  for (_i=1; _i<packet[1]+2; _i++)
    packet[packet[1]+2] += ~packet[_i];
  packet[packet[1]+2] &= 0xFF;
}

bool MorpheusDevice::communicate (unsigned char id, unsigned char length, unsigned char command, int expected, bool twice, int extra) throw ()
{
  // packets are always at least 6 bytes long
  _exp = expected;
  if (expected > 0) _exp += 5;
  

  // add what was left last time
  for (_k=0; _k<_leftover; _k++)
    _buffer[_k] = _leftovers[_k];

  // check buffer limit
  if (_exp + _leftover > MORPHEUS_BUFFER_SIZE) {
    DYNAERROR("Communication not possible: expected response size of " << _exp+_missing << " exceeds buffer capacity of " << MORPHEUS_BUFFER_SIZE << ".");
    return 0;
  }

  // preparation
  _packet[1] = length + 3;
  _packet[2] = id;
  _packet[4] = command;
  for (_i=_leftover; _i<MORPHEUS_BUFFER_SIZE; _i++)
    _buffer[_i] = 0;
  checksum(&_packet[0]);

#if 0
  std::cout << "Out:";
  for (_k=0; _k<length+6+extra; _k++)
  std::cout << " " << (int)_packet[_k];
  std::cout << std::endl;
#endif

  // communication
  _ret = _com->communicate(&_packet[0], &_buffer[_leftover], length+6+extra, _exp+_missing, twice);

#if 0
  std::cout << "In: ";
  for (_k=0, _l=_ret+_leftover; _k<_l; _k++)
  std::cout << " " << (int)_buffer[_k];
  std::cout << std::endl;
#endif

  // check input for valid packets
  _packetnum = 0;
  for (_k=0, _l=_ret+_leftover; _k<_l; _k++) {

    // this should never happen unless we have a leftover that was not completed in the beginning
    if (_buffer[_k] != 0xaa) {
      DYNAWARNING("Communication failure: non-start byte found at buffer index " << _k << " contrary to expectation.");
      continue;
    }

    // we have too little left, and we would have skipped data in a real one, so treat it as leftover
    _left = _l - _k;
    if (_left < 6 || _left < _buffer[_k+1]+3) {
      for (_i=0; _i<_left; _i++)
        _leftovers[_i] = _buffer[_k+_i];
      _leftover = _left;
      if (_verbosity > 1) {
        std::cout << "Communication failure: incomplete packet: [";
        for (_i=0; _i<_left; _i++)
          std::cout << " " << (int)_buffer[_k+_i];
        std::cout << "]" << std::endl;
      }
      break;
    }

    // update counters
    _plength = _buffer[_k+1] + 3;
    _leftover = 0;

    // checksum comparison
    _tmp_csum = _buffer[_k+_plength];
    checksum(&_buffer[_k]);
    if (_validate_response && _tmp_csum != _buffer[_k+_plength]) {
      if (_verbosity > 1) {
        std::cout << "Comunication failure: packet with invalid checksum: [";
        for (_i=0; _i<_plength; _i++)
          std::cout << " " << (int)_buffer[_k+_i];
        std::cout << "] should end with " << (int)_tmp_csum << std::endl;
      }
      continue;
    }

    _packetlist[_packetnum] = _k;
    _packetnum++;
    _k += _plength - 1;
  }

  // make a guess what's coming next
  if (_ret == _exp || _ret == _exp+_missing)
    _missing = 0;
  else if (_ret > _exp)
    _missing -= _ret - _exp;
  else
    _missing = _exp - _ret;
  if (_missing < 0) _missing = 0;

  return _missing == 0;
}

