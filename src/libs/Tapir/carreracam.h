/*
libTapir - Tapir vision toolkit interfac
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

#ifndef _KATCTR_CarreraDetector_H_
#define _KATCTR_CarreraDetector_H_

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#define CARRERAVISION_DATA 23

/** A simple 2D vector. */
struct CarreraCoord {
  CarreraCoord () {reset();};
  void set (double a, double b) {x = a; y = b;};
  void set (double *data) {x = data[0]; y = data[1];};
  void get (double *data) {data[0] = x; data[1] = y;};
  void reset () {x = y = 0.0;};
  bool operator== (CarreraCoord &c) {return x==c.x && y==c.y;};
  double x, y;
};

/** A simple 2D line or rectangle. */
struct CarreraPair {
  CarreraPair () {reset();};
  void set (double a, double b, double c, double d) {start.set(a,b); end.set(c,d);};
  void set (double *data) {start.set(&data[0]); end.set(&data[2]);};
  void get (double *data) {start.get(&data[0]); end.get(&data[2]);};
  void reset () {start.reset(); end.reset();};
  bool operator== (CarreraPair &p) {return start==p.start && end==p.end;};
  CarreraCoord start, end;
};

/** Defines an object as detected by the CarreraTool. */
class CarreraObject
{
 public:
  CarreraObject () {reset();};
  void reset () {
    pos.reset(); vel.reset();
    main.reset(); secondary.reset(); bbox.reset(); brect.reset();
    size = area = angle = compactness = ratio = 0.;
    known = false;
  };
  bool operator== (CarreraObject &o) {
    return known==o.known && pos==o.pos && vel==o.vel
        && size==o.size && angle==o.angle && compactness==o.compactness
        && ratio==o.ratio && area==o.area
        && main==o.main && secondary==o.secondary
        && bbox==o.bbox && brect==o.brect;
  };
  CarreraCoord pos, vel;
  CarreraPair main, secondary, bbox, brect;
  double size, area, angle, compactness, ratio;
  bool known;
};

/** Base class for implementations that allow retrieving information about
  * objects detected by the CarreraTool. */
class CarreraDetector
{
 public:

  /** Default constructor.
    \param extended set to true for use with Tapir2
    \param index number of the camera as set in tapir.cfg */
  CarreraDetector (bool extended=true, int index=0);
  ~CarreraDetector () {};

  /** Accesses the current object information.
    * \param extended determines which scheme to use, forwarded to set_buffers(bool)
    * \returns object containing information gathered by the last call to update() */
  CarreraObject get_object () throw ();

  /** Fetches new information about the current object from the CarreraTool.
    * If no new information can be gotten, currentObject will be marked as unknown,
    * while all other values will remain those of the last known instance. */
  virtual void update () throw () = 0;

 protected:

  /** Checks if the CarreraTool is running on the local machine.
    * \return true if any process with a name including "Carrera" is running */
  bool server_running ();

  /** Waits until an instance of the CarreraTool is found to be running. */
  void wait_until_running ();

  CarreraObject currentObject;
  bool _extended,_verbose;
  int _i;
};

/** Allows to retrieve information about objects detected by the CarreraTool
  * via shared memory. */
class CarreraSharedMemoryDetector : public CarreraDetector
{
 public:

  CarreraSharedMemoryDetector (bool extended=true, int index=0);
  ~CarreraSharedMemoryDetector () {};
  void update () throw ();

 protected:

  /** Re-initializes the data offsets within the shared memory.
    * \param extended determines which scheme to use; false for Tapir versions before
    * August 2011, true otherwise
    * \param offset offset within shared memory at which to begoin storing this camera's data */
  void set_buffers (bool extended, int offset);

  /** Writes the contents of the shared memory into an object.
    * \param shared memory address where data begins (index of horizontal position)
    * \param object container to write data into */
  void commit (int* buffer, CarreraObject& object);

  // shared memory stuff
  int visionTime;
  double _tmpbuf[CARRERAVISION_DATA];
  void *visionData;
  bool *visionClientAccess;
  int *visionClientBuffer1;
  int *visionClientBuffer2;
  int *visionClientTime1;
  int *visionClientTime2;
  bool visionUpdated;
  int visionLastUsed;
};

/*  OBSOLETE: superceded by TapirTCPDetector
  * Allows to retrieve information about objects detected by the CarreraTool
  * via TCP/IP.
class CarreraTCPDetector : public CarreraDetector
{
 public:

  CarreraTCPDetector (bool extended=true, int index=0) {};
  ~CarreraTCPDetector () {};
  void update () throw () {};
}; */

#endif
