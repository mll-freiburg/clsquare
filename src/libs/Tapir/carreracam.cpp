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

#include "carreracam.h"
#include <iostream>
#include <cmath>
#include <cstdlib>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

CarreraDetector::CarreraDetector (bool extended, int camindex)
{
  _verbose = false;
  currentObject.reset();
}

void CarreraDetector::wait_until_running ()
{
  if (!server_running()) {
    printf("Starting CarreraTool...\n");
    _i = system("xterm -e \"./Tapir\" &");
    sleep(5);
    while (!server_running()) {
      printf("Error: CarreraTool could not be started. Please launch the program manually, then press enter.\n");
      std::cin.ignore();
    }
  }
}

bool CarreraDetector::server_running ()
{
  int rc = system("ps -C carrera");
  return WEXITSTATUS(rc) == 0;
}

CarreraObject CarreraDetector::get_object () throw ()
{
  return currentObject;
}

CarreraSharedMemoryDetector::CarreraSharedMemoryDetector (bool extended, int camindex)
{
  CarreraDetector::wait_until_running();

  key_t visionkey;
  int shmflg = IPC_CREAT | SHM_R | SHM_W;
  int visionid;
  int size = 1024;

  if (!extended && camindex != 0) {
    camindex = 0;
    printf("Warning: Tapir v1 does not support multiple cameras. Ignoring index.\n");
  }

  int needed = (camindex+1) * (sizeof(char) + 100*sizeof(int));
  if (needed > size) {
    printf("Reserved shared memory (%i) cannot hold %i devices (%i). Using index 0.\n", size, camindex, needed);
    camindex = 0;
  }

  if ((visionkey = ftok ("/tmp", 'b')) == (key_t) - 1)
    printf("IPC error: could not generate a key for the vision module.\n");
  else if (_verbose)
    printf("Vision key: %i",visionkey);

  if ((visionid = shmget (visionkey, size, shmflg)) < 0)
    printf("Could not get handle to shared memory for the vision module.\n");
  else if (_verbose)
    printf("Vision ipc-id: %i",visionid);

  if ((visionData = shmat (visionid, NULL, 0)) == (char *) -1)
    printf("Could not attach to shared mem for the vision module\n");

  set_buffers(extended, camindex * (sizeof(char) + 100*sizeof(int)));
  visionTime     = -1;
  visionLastUsed = 0;

  printf("Waiting for vision client to connect...\n");
  usleep(300000);
  visionTime = (*visionClientAccess == 0) ? *visionClientTime2 : *visionClientTime1;
  usleep(100000);
}

void CarreraSharedMemoryDetector::update () throw ()
{
  CarreraObject newObject;

  if (*visionClientAccess == 0) {
    if (*visionClientTime2 != visionTime) {
      commit(&visionClientBuffer2[0], newObject);
      visionTime = *visionClientTime2;
      visionUpdated = true;
    }
  } else {
    if (*visionClientTime1 != visionTime) {
      commit(&visionClientBuffer1[0], newObject);
      visionTime = *visionClientTime1;
      visionUpdated = true;
    }
  }

  if (visionUpdated) {
    newObject.vel.x = newObject.pos.x - currentObject.pos.x;
    newObject.vel.y = newObject.pos.y - currentObject.pos.y;
    currentObject = newObject;
    visionUpdated = false;
    visionLastUsed = visionTime;
  } else {
    currentObject.known = false;
  }
}

void CarreraSharedMemoryDetector::set_buffers (bool extended, int offset)
{
  if (!extended) {
    visionClientAccess  = (bool*) visionData;
    visionClientBuffer1 = (int*) ((char*) visionData + sizeof (char));
    visionClientTime1   = (int*) ((char*) visionData + sizeof (char) + 6 * sizeof (int));
    visionClientBuffer2 = (int*) ((char*) visionData + sizeof (char) + 7 * sizeof (int));
    visionClientTime2   = (int*) ((char*) visionData + sizeof (char) + 13 * sizeof (int));
  } else {
    visionClientAccess  = offset + (bool*) visionData;
    visionClientBuffer1 = offset + (int*) ((char*) visionData + sizeof (char) +  1 * sizeof (int));
    visionClientTime1   = offset + (int*) ((char*) visionData + sizeof (char) +  0 * sizeof (int));
    visionClientBuffer2 = offset + (int*) ((char*) visionData + sizeof (char) + 51 * sizeof (int));
    visionClientTime2   = offset + (int*) ((char*) visionData + sizeof (char) + 50 * sizeof (int));
  }   
  _extended = extended;
}

void CarreraSharedMemoryDetector::commit (int* buf, CarreraObject& obj)
{
  for (_i=0; _i<CARRERAVISION_DATA; _i++)
    _tmpbuf[_i] = buf[_i];

  obj.pos.set(&_tmpbuf[0]);
  obj.size = _tmpbuf[2];
  obj.known = true;

  if (abs(obj.pos.x) > 320 || abs(obj.pos.y) > 240)
    printf("Sanity check of object position failed; shared memory may be corrupted.\n");

  // stop here if using an old version of the CarreraTool
  if (!_extended) return;

  obj.area = _tmpbuf[4];
  obj.compactness = _tmpbuf[5] / 1000.;
  obj.ratio = _tmpbuf[6] / 1000.;
  obj.angle = double(_tmpbuf[3]) / 10.;

  for (_i=7; _i<15; _i++)
    _tmpbuf[_i] /= 10.;
  obj.main.set(&_tmpbuf[7]);
  obj.secondary.set(&_tmpbuf[11]);

  obj.bbox.set(&_tmpbuf[15]);
  obj.brect.set(&_tmpbuf[19]);
}

