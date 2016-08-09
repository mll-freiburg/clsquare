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

#ifdef FOUND_boost
#include <boost/algorithm/string.hpp>
#include <boost/lambda/lambda.hpp>
#endif
#include <vector>
#include <list>

#include "simloid.h"

///< \todo allow setting step length; need to modify simloid.conf

/************ Setup ************/

Simloid::Simloid() throw ()
{
  if (system("xterm -e \"./Simloid\" &") < 0)
    std::cerr << "Warning: could not launch xterm." << std::endl;
  sleep(2);

  _hold = true;
  for (int i=0; i<19; i++) _target[i] = -1;
  _initialized = false;
}

bool Simloid::init () throw ()
{
#ifdef FOUND_boost

  _initialized = true;
  _io = new boost::asio::io_service();
  _socket = new tcp::socket(*_io);
  tcp::resolver resolver(*_io);
  tcp::resolver::query query("localhost", "7777");
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
  tcp::resolver::iterator end;

  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && endpoint_iterator != end) {
    _socket->close();
    _socket->connect(*endpoint_iterator++, error);
  }
  if (error) return false;

  for (int i=0; i<19; i++) _target[i] = _hold ? 512 : -1;

  std::ostream request_stream(&_request);
  for (int i=1; i<20; i++) request_stream << "(pid " << i << " 512)\n";
  request_stream << "(done)\n";
  boost::asio::write(*_socket, _request);
  step();
  return true;

#else

  _initialized = false;
  std::cout << "Error: cannot initialize Simloid interface without boost libraries." << std::endl;
  return false;

#endif
}

Simloid::~Simloid() throw ()
{
  ;
}

/************ Output ************/

bool Simloid::get_pos (unsigned char id, int& pos, int& speed) throw ()
{
  pos = 1024 - _position[id+1];
  speed = _velocity[id+1];
  return _got_pos;
}

bool Simloid::get_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ()
{
  for (int i=0; i<motors; i++)
    get_pos(id[i], pos[i], speed[i]);
  return _got_pos;
}

bool Simloid::get_imu (int* data) throw ()
{
  return get_imu(data[0],data[1],data[2],data[3],data[4],data[5]);
}

bool Simloid::get_imu (int& roll, int& pitch, int& yaw, int& vel1, int& vel2, int& vel3) throw ()
{
  ///< \todo better match with real output
  roll =  1000 * _imu[3];
  pitch = 1000 * _imu[4];
  yaw =   1000 * _imu[5];
  vel1 =  1000 * _imu[0];
  vel2 =  1000 * _imu[1];
  vel3 =  1000 * _imu[2];
  return _got_imu;
}

/************ Input ************/

bool Simloid::reset () throw ()
{
  if (!_initialized) return false;
  _initialized = false;
#ifdef FOUND_boost
  std::ostream request_stream(&_request);
  request_stream << "(reset)\n";
  for (int i=1; i<20; i++) request_stream << "(pid " << i << " 512)\n";
  request_stream << "(done)\n";
  boost::asio::write(*_socket, _request);
#endif

  for (int i=0; i<19; i++) _target[i] = _hold ? 512 : -1;
  step();
  step();
  _initialized = true;
  return true;
}

bool Simloid::step () throw ()
{
  if (!_initialized) return false;
  for (unsigned int i=0; i<20; i++)
    _velocity[i] = _position[i] = -1;

#ifdef FOUND_boost
  std::string line;
  std::vector< std::string > vec;
  boost::regex angle_expr(".*\\(angle ([\\d\\s]*)\\).*");
  boost::regex orient_expr(".*\\(imu ([\\d\\s\\.\\-]*)\\).*");
  boost::regex rate_expr(".*\\(rate ([\\d\\s\\.\\-]*)\\).*");
  boost::smatch match;

  std::ostream request_stream(&_request);
  for (int i=0; i<19; i++)
    if (_target[i] > -1)
      request_stream << "(pid " << (unsigned int)i+1 << " " << 1024-_target[i] << ")\n";
  boost::asio::write(*_socket, _request);

  for (int i=0; i<10; i++) {
    std::ostream request_stream(&_request);
    request_stream << "(done)\n";
    boost::asio::write(*_socket, _request);

    boost::asio::streambuf b;
    boost::asio::read_until(*_socket, b, angle_expr);
    std::istream is(&b);
    std::getline(is, line);
  }

  // get positions and calculate speed in case getting it fails
  if ((_got_pos = boost::regex_search(line, match, angle_expr))) {
    std::string test(match[1].str());
    boost::split(vec, test, boost::is_space(), boost::token_compress_on);
    for (unsigned int i=0; i<vec.size(); ++i) {
      _velocity[i+1] = -1 * _position[i+1];
      _position[i+1] = boost::lexical_cast<int>(vec[i]);
      _velocity[i+1] += _position[i+1];
    }
  }

  // get speed
  if (boost::regex_search(line, match, rate_expr)) {
    std::string test(match[1].str());
    boost::split(vec, test, boost::is_space(), boost::token_compress_on);
    for (unsigned int i=0; i<vec.size(); ++i)
      _velocity[i+1] = boost::lexical_cast<int>(vec[i]);
  }

  // get imu data
  if ((_got_imu = boost::regex_search(line, match, orient_expr))) {
    std::string test(match[1].str());
    boost::split(vec, test, boost::is_space(), boost::token_compress_on);
    for (unsigned int i=0; i<vec.size(); ++i)
      _imu[i] = boost::lexical_cast<float>(vec[i]);
  }
#endif

  return true;
}

bool Simloid::set_pos (unsigned char id, int pos) throw ()
{
  _target[id] = pos;
  return true;
}

bool Simloid::set_pos (unsigned char id, int pos, int speed) throw ()
{
  ///< \todo allow setting velocity
  LWARNING("target velocities not supported by simulation");
  return set_pos(id, pos);
}

bool Simloid::set_pos (unsigned char motors, unsigned char* id, int* pos) throw ()
{
  for (int i=0; i<motors; i++)
    set_pos(id[i], pos[i]);
  return true;
}

bool Simloid::set_pos (unsigned char motors, unsigned char* id, int* pos, int* speed) throw ()
{
  ///< \todo allow setting velocity
  LWARNING("target velocities not supported by simulation");
  for (int i=0; i<motors; i++)
    set_pos(id[i], pos[i]);
  return true;
}

bool Simloid::save_snapshot () throw ()
{
  if (!_initialized) return false;
#ifdef FOUND_boost
  std::ostream request_stream(&_request);
  request_stream << "(save)\n(done)\n";
  boost::asio::write(*_socket, _request);
#endif
  return true;
}

bool Simloid::load_snapshot () throw ()
{
  if (!_initialized) return false;
#ifdef FOUND_boost
  std::ostream request_stream(&_request);
  request_stream << "(load)\n(done)\n";
  boost::asio::write(*_socket, _request);
#endif
  return true;
}

