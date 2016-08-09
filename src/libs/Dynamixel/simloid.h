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

#ifndef _SIMLOID_H_
#define _SIMLOID_H_

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef FOUND_boost
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#endif

#include "loid.h"

#ifdef FOUND_boost
using boost::asio::ip::tcp;
#endif

/** Implementation of a simulated Loid-type humanoid robot, which attaches to a
  * modified version of the Simloid simulation made by the AT Humboldth of the
  * Humboldt-University Berlin. */
class Simloid : public Loid
{
public:
  Simloid () throw ();
  ~Simloid () throw ();

  bool init () throw ();
  bool set_param (Loid::param param, int value) throw () {
    return Loid::set_param(param,value);
  };
  bool reset () throw ();
  bool step () throw ();

  // specific output as per interface
  bool get_pos (unsigned char id, int& pos, int& speed) throw ();
  bool get_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ();
  bool get_imu (int& roll, int& pitch, int& yaw, int& vel1, int& vel2, int& vel3) throw ();
  bool get_imu (int* data) throw ();

  // specific input as per interface
  bool set_pos (unsigned char id, int pos) throw ();
  bool set_pos (unsigned char id, int pos, int speed) throw ();
  bool set_pos (unsigned char motors, unsigned char* id, int* pos) throw ();
  bool set_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ();

#ifdef FOUND_boost
  // callback handler
  void on_timer (const boost::system::error_code &err);
#endif

  /** Writes a snapshot of the simulation to file.
    * \return false if the snapshot could not be saved */
  bool save_snapshot () throw ();

  /** Writes a snapshot of the simulation to disc.
    * \return false if the snapshot could not be loaded */
  bool load_snapshot () throw ();

protected:
#ifdef FOUND_boost
  boost::asio::io_service	*_io;
  boost::asio::streambuf	_request;
  tcp::socket *_socket;
#endif
  double _imu[6];
  unsigned int _position[20];
  int _velocity[20];
  int _target[20];
  bool _got_imu, _got_pos, _initialized;
};

#endif

