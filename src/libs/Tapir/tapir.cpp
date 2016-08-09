/*
libTapir - Tapir vision toolkit interface
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

#include "tapir.h"
#include "global.h"
#include "valueparser.h"
#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h> 
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <sys/param.h>
#include <sstream>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "unistd.h"

#define SETSTRING(target,source) { target = new char[strlen(source)+1]; strncpy(target, source, strlen(source)); target[strlen(source)]='\0'; }

#ifdef __BYTE_ORDER
  # if __BYTE_ORDER != __LITTLE_ENDIAN
    Error: no support for big-endian systems yet!
  #endif 
#endif

TapirObject::TapirObject ()
{
  this->x = this->y = this->size = this->angle = this->time = -1;
  this->known = false;
}

TapirDetector::TapirDetector (const char* fname)
{
  _nopipe = _nowarn = true;
}

void TapirDetector::open_pipe (const char* fname) throw ()
{
  ValueParser vp(fname==0 ? "tapir.cfg" : fname, "Main");
  if (vp.get("pipe", _pipename, 255) > 0) {

    int pipeCreation = mkfifo(_pipename, 0666);
    if (pipeCreation < 0) {
      if (errno == EEXIST) {
        IOUT("Pipe " << _pipename << " already exists.");
      } else {
        EOUT("Could not create pipe " << _pipename << ": " << strerror(errno));
      }
    } else {
      IOUT("Pipe " << _pipename << " created.");
    }

    _pipe = open(_pipename, O_NONBLOCK);
    if (_pipe < 0) {
      EOUT("Could not open pipe " << _pipename << ": " << strerror(errno));
    } else {
      IOUT("Pipe " << _pipename << " opened.");
      _nopipe = false;
    }
  }
}

void TapirDetector::wait_until_running (const char* config)
{
  ///< \todo works only if tapir in same folder and written with lowercase T
  // test only works for single instance
  if (server_running())
    WOUT(10, "An instance of Tapir is already running. Will not be able to tell whether this one started correctly.")

  int count = 0;
  do {
    if (count != 0) {
      EOUT("Error: Tapir could not be started. Please launch the program manually, then press enter.");
      std::cin.ignore();
    }
    count++;
    IOUT("Starting Tapir...");
    stringstream ss;
    ss << "xterm -e \"./tapir " << config << "\" &";
    _i = system(ss.str().c_str());
    sleep(2);
  } while (!server_running());
}

bool TapirDetector::server_running ()
{
  int rc = system("ps -C tapir > /dev/null");
  return WEXITSTATUS(rc) == 0;
}

void TapirDetector::wait_until_data ()
{
  if (_current.size() < 1) return;
  while (_current[0].time == -1.)
    update();
}

const TapirObject TapirDetector::get_object () throw ()
{
  return get_objects()[0];
}

const std::vector<TapirObject> TapirDetector::get_objects () throw ()
{
  if (_current.size() < 1)
    _current.push_back(TapirObject());
  return _current;
}

void TapirDetector::notify (std::string message) throw ()
{
  if (_nopipe && _nowarn) {
    WOUT(10, "No input pipe specified by Tapir; will not be able to send messages to it.");
    _nowarn = false;
    return;
  }

  ///< \todo Inefficient and stupid. Get pipe to work properly!
  char tmp[255];
  sprintf(tmp, "echo \"%s\" > %s", message.c_str(), _pipename);
  _i = system(tmp);
IOUT(tmp);
  //_i = write(_pipe, message.c_str(), message.size());
}

///////////////////
// Shared Memory //
///////////////////

///< @todo should move constructor content into init() to be able to pass return value down (alternative: exception)
TapirSharedMemoryDetector::TapirSharedMemoryDetector (const char* fname)
{
  // must run locally
  TapirDetector::wait_until_running(fname);

  ValueParser vp(fname==0 ? "tapir.cfg" : fname, "Output");
  _success = false;

  key_t key;
  int num = 4;
  int size = 2 * (num * sizeof(double) + sizeof(int)) + sizeof(bool);

  char keypath[255] = "/tmp\0";
  int keyid;
  vp.get("key_path", keypath, 255);
  vp.get("key_id", keyid, 98);
  if ((key=ftok(keypath, keyid)) == (key_t)-1) {
    EOUT("Failed to get IPC key (path " << keypath << ", ID " << keyid << ")");
    return;
  }

  int shmflg = IPC_CREAT | SHM_R | SHM_W;
  int shmid;
  if ((shmid = shmget(key, size, shmflg)) < 0) {
    EOUT("Failed to get IPC ID: " << shmid);
    return;
  }

  void *shmdata;
  if ((shmdata = shmat(shmid, NULL, 0)) == (char*) -1) {
    EOUT("shmat");
    return;
  }

  _access  = (bool*)shmdata;
  _time1   = (double*) ((char*)shmdata + sizeof(bool));
  _buffer1 = (double*) ((char*)shmdata + sizeof(bool) + sizeof(double));
  _time2   = (double*) ((char*)shmdata + sizeof(bool) + sizeof(double) * (num+1));
  _buffer2 = (double*) ((char*)shmdata + sizeof(bool) + sizeof(double) * (num+2));
  _success = true;
  TapirDetector::wait_until_data();

  _current.push_back(TapirObject());

  open_pipe(fname);
}

void TapirSharedMemoryDetector::update () throw ()
{
  if (!_success) {
    EOUT("Detector was not properly initialized.");
    return;
  }
  TapirObject new_object;

  if (*_access == 0) {
    if (*_time2 != _current[0].time) {
      commit(&_buffer2[0], new_object, *_time2);
      _updated = true;
    }
  } else {
    if (*_time1 != _current[0].time) {
      commit(&_buffer1[0], new_object, *_time1);
      _updated = true;
    }
  }

  if (_updated) {
    _current.clear();
    _current.push_back(new_object);
    _updated = false;
  } else {
    _current[0].known = false;
  }
}

void TapirSharedMemoryDetector::commit (double* buf, TapirObject& obj, double time)
{
  obj.x     = buf[0];
  obj.y     = buf[1];
  obj.size  = buf[2];
  obj.angle = buf[3];
  obj.time  = time;
  obj.known = true;
}

////////////////////////////////
// Multi-Object Shared Memory //
////////////////////////////////

TapirMOSharedMemoryDetector::TapirMOSharedMemoryDetector (const char* fname)
{
  ValueParser vp(fname==0 ? "tapir.cfg" : fname, "Output");
  _success = false;

  vp.get("nb_objects",_nb_objects,1);

  key_t key;
  int num = 4;
  int size = _nb_objects * (num * sizeof(double) + sizeof(pthread_mutex_t));

  char keypath[255] = "/tmp\0";
  int keyid;
  vp.get("key_path", keypath, 255);
  vp.get("key_id", keyid, 98);
  if ((key=ftok(keypath, keyid)) == (key_t)-1) {
    EOUT("Failed to get IPC key (path " << keypath << ", ID " << keyid << ")");
    return;
  }

  int shmflg = IPC_CREAT | SHM_R | SHM_W;
  int shmid;
  if ((shmid = shmget(key, size, shmflg)) < 0) {
    EOUT("Failed to get IPC ID: " << shmid);
    return;
  }

  void *shmdata;
  if ((shmdata = shmat(shmid, NULL, 0)) == (char*) -1) {
    EOUT("shmat");
    return;
  }

  _mutexes = (pthread_mutex_t*)shmdata;
  _buffer = (double*) ((char*)shmdata + _nb_objects * sizeof(pthread_mutex_t));
  _success = true;

  TapirDetector::wait_until_data();
  _current.push_back(TapirObject());

  open_pipe(fname);
}

void TapirMOSharedMemoryDetector::update () throw ()
{
  if (!_success) {
    EOUT("Detector was not properly initialized.");
    return;
  }
  std::vector<TapirObject> newObjects(_nb_objects);

  for(int i=0; i<_nb_objects; i++){
    pthread_mutex_lock(&_mutexes[i]);
    commit(&_buffer[i*5], newObjects[i], _buffer[i+5]);
    pthread_mutex_unlock(&_mutexes[i]);
  }

  _current = newObjects;
}

void TapirMOSharedMemoryDetector::commit (double* buf, TapirObject& obj, double time)
{
  obj.x     = buf[0];
  obj.y     = buf[1];
  obj.size  = buf[2];
  obj.angle = buf[3];
  obj.time  = time;
  obj.known = true;
}

/////////
// TCP //
/////////

TapirTCPDetector::TapirTCPDetector (const char* fname)
{
  ValueParser vp(fname==0 ? "tapir.cfg" : fname, "Output");
  char *address = new char[25];
  if (vp.get("address", address, 25) < 1)
    SETSTRING(address, "127.0.0.1");
  int port;
  vp.get("port", port, 7102);

  try {
    _socket = new TCPSocket(address, port);
    _connected = true;
  } catch (SocketException e) {
    EOUT("Could not create socket: " << e.what());
    return;
  }

  _current.push_back(TapirObject());
  _ping[0] = _ping[1] = '#';
}

void TapirTCPDetector::update () throw ()
{
  if (!_connected) return;

  // send data
  try {
    _socket->send((void*)&_ping[0], 2);
  } catch (SocketException e) {
    EOUT("Failed to send data: " << e.what());
    _current[0].known = false;
    return;
  }

  // get data
  try {
    _ret = _socket->recv((void*)&_buffer[0], 255);
  } catch (SocketException e) {
    EOUT("Failed to receive data: " << e.what());
    _current[0].known = false;
    return;
  }

  // not enough bytes means probably no object visible
  if (_ret < 42) {
    _current[0].known = false;
    return;
  }

  // no start bytes means something's wrong
  if (_buffer[0] != '*' || _buffer[1] != '*') {
    EOUT("Malformed packet.");
    _current[0].known = false;
    return;
  }

  memcpy((void*)&_current[0].time,  (void*)&_buffer[ 2], 8);
  memcpy((void*)&_current[0].x,     (void*)&_buffer[10], 8);
  memcpy((void*)&_current[0].y,     (void*)&_buffer[18], 8);
  memcpy((void*)&_current[0].size,  (void*)&_buffer[26], 8);
  memcpy((void*)&_current[0].angle, (void*)&_buffer[34], 8);
  _current[0].known = true;
}

