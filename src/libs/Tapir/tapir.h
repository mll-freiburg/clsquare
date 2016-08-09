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

#ifndef _TAPIR_H_
#define _TAPIR_H_

#include "Socket.h"
#include <vector>

/** Base specification of objects detected by Tapir. */
class TapirObject
{
 public:
  TapirObject ();
  ~TapirObject () {};
  double x, y, size, angle, time;
  bool known;
};

/** Base class for implementations that allow retrieving information about
  * objects detected by Tapir. */
class TapirDetector
{
 public:

  /** Default constructor.
    * \param path to the configuration file */
  TapirDetector (const char* fname="tapir.cfg");
  virtual ~TapirDetector () {};

  /** Accesses the current object information.
    * \returns first object containing information gathered by the last call to update() */
  const TapirObject get_object () throw ();

  /** Accesses the current object information.
    * \returns list of objects containing information gathered by the last call to update() */
  const std::vector<TapirObject> get_objects () throw ();

  /** Fetches new information about the current object from the TapirTool.
    * If no new information can be gotten, the current object will be marked as unknown,
    * while all other values will remain those of the last known instance. */
  virtual void update () throw () = 0;

  /** Sends a message to Tapir.
    * \param message string to send */
  virtual void notify (std::string message) throw ();

  /** Prepares a pipe for communication. */
  void open_pipe (const char* fname) throw ();

 protected:

  /** Checks if Tapir is running on the local machine.
    * NOTE: this does not ensure that server and client use the same shared memory
    * block.
    * \return true if any process with a name including "Tapir" is running */
  bool server_running ();

  /** Waits until an instance of Tapir is found to be running. */
  void wait_until_running (const char* fname="tapir.cfg");

  /** Waits until Tapir sends valid image data. */
  void wait_until_data ();

  std::vector<TapirObject> _current;
  int _i, _pipe;
  char _pipename[255];
  bool _nopipe, _nowarn;
};

/** Allows to retrieve information about a single object detected by Tapir
  * via shared memory. */
class TapirSharedMemoryDetector : public TapirDetector
{
 public:

  TapirSharedMemoryDetector (const char* fname="tapir.cfg");
  ~TapirSharedMemoryDetector () {};
  void update () throw ();

 protected:

  /** Writes the contents of the shared memory into an object.
    * \param shared memory address where data begins (index of horizontal position)
    * \param object container to write data into */
  void commit (double* buffer, TapirObject& object, double time);

  // shared memory stuff
  void *_data;
  bool *_access;
  double *_buffer1;
  double *_buffer2;
  double *_time1;
  double *_time2;
  bool _updated;

  bool _success;
};

/** Allows to retrieve information about multiple objects detected by Tapir
  * via shared memory. */
class TapirMOSharedMemoryDetector : public TapirDetector
{
 public:

  TapirMOSharedMemoryDetector (const char* fname="tapir.cfg");
  ~TapirMOSharedMemoryDetector () {};
  void update () throw ();

 protected:

  /** Writes the contents of the shared memory into an object.
    * \param shared memory address where data begins (index of horizontal position)
    * \param object container to write data into */
  void commit (double* buffer, TapirObject& object, double time);

  int _nb_objects;

  // shared memory stuff
  void *_data;
  pthread_mutex_t *_mutexes;
  bool *_access;
  double *_buffer;
  double *_buffer2;
  double *_time1;
  double *_time2;
  bool _updated;

  bool _success;
};

/** Allows to retrieve information about objects detected by Tapir
  * via TCP/IP. */
class TapirTCPDetector : public TapirDetector
{
 public:

  TapirTCPDetector (const char* fname="tapir.cfg");
  ~TapirTCPDetector () {};
  void update () throw ();

 protected:

  /** Writes the contents of the shared memory into an object.
    * \param shared memory address where data begins (index of horizontal position)
    * \param object container to write data into */
  void commit (double* buffer, TapirObject& object, double time);

  TCPSocket *_socket;
  int _i, _ret;
  bool _connected;
  char _buffer[255], _ping[2];
};

#endif
