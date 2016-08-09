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

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <ctime>
#include <signal.h>
#include <sys/time.h>

#define CMD(x) strcmp(_command,x)==0
#define ROBOTIS ((RobotisDevice*)robot)
#define MORPHEUS ((MorpheusDevice*)robot)

#include "robotis.h"
#include "morpheus.h"
DynamixelDevice *robot;

using namespace std;
bool loop = true;

void sighandler (int sig)
{
  loop = false;
}

int main (int args, char **argv)
{
  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGINT, &sighandler);

  vector<int> read;
  vector<int> par[2];
  int imu = -1;

  bool command = false;
  bool morph = false;
  int verb = 100;
  enum {Default, Scan, PIDTest} mode = Default;
  int baudrate = 1000000;
  int timeout = 20;
  int latency = 500;
  bool sync_read = false;

  if (args < 2) {
    cout << "Arguments: " << endl;
    cout << " Mode: " << endl;
    cout << "  <default>      print specified output" << endl;
    cout << "  -s             scan for operational servos" << endl;
    cout << "  -c             command mode" << endl;
    cout << "  -o $0          test PID settings of servo $0" << endl;
    cout << " Output: " << endl;
    cout << "  -r $0 [$1 ...] read position and speed of servos $0, $1, and so on" << endl;
    cout << "  -p $0 $1       read parameter $1 on servo $0" << endl;
    cout << "  -i $0          read IMU at servo ID $0" << endl;
    cout << " Settngs: " << endl;
    cout << "  -v $0          set verbosity to $0" << endl;
    cout << "  -b $0          set baud rate to $0 bps" << endl;
    cout << "  -l $0          set library type to $0 (FTDI or FTD2xx)" << endl;
    cout << "  -f $0          set firmware type to $0 (Robotis or Morpheus)" << endl;
    cout << "  -t $0 $1       set communication timeout to $0 and latency to $1" << endl;
    cout << "  -y             use synchronous read" << endl;
    return -1;
  }

  robot = new RobotisDevice();
  for (int a=0; a<args; a++) if (strcmp(argv[a],"-f") == 0) {
    if (strcmp(argv[a+1],"Robotis") == 0) {
      morph = false;
    } else if (strcmp(argv[a+1],"Morpheus") == 0) {
      morph = true;
    } else {
      cout << "Invalid firmware type: " << argv[a+1] << endl;
      return -1;
    }
  }

  if (!morph) {
    cout << "Firmware type: Robotis (original)" << endl;
    robot = new RobotisDevice();
  } else {
    cout << "Firmware type: Morpheus" << endl;
    robot = new MorpheusDevice();
  }
  
  bool renewed = false;
  for (int a=0; a<args; a++) if (argv[a][0]=='-') switch (argv[a][1]) {
    case 's':
      mode = Scan;
      break;
    case 'o':
      mode = PIDTest;
      imu = atoi(argv[a+1]);
      a++;
      break;
    case 'y':
      sync_read = true;
      break;
    case 'i':
      imu = atoi(argv[a+1]);
      if (morph) {
        imu = -1;
        cout << "Error: Morpheus firmware does not support IMU. Ignoring option." << endl;
      }
      break;
    case 'r':
      for (a++; a<args && argv[a][0]!='-'; a++)
        read.push_back(atoi(argv[a]));
      a--;
      break;
    case 'p':
      par[0].push_back(atoi(argv[a+1]));
      par[1].push_back(atoi(argv[a+2]));
      break;
    case 'v':
      verb = atoi(argv[a+1]);
      a++;
      break;
    case 'c':
      command = true;
      break;
    case 'b':
      baudrate = atoi(argv[a+1]);
      break;
    case 't':
      timeout = atoi(argv[a+1]);
      latency = atoi(argv[a+2]);
      break;
    case 'l':
      if (strcmp(argv[a+1],"FTDI") == 0) {
        cout << "Device type: FTDI" << endl;
        robot->set_param(DynamixelDevice::COMTYPE, FTDevice::FTDI);
        renewed = true;
      } else if (strcmp(argv[a+1],"FTD2xx") == 0) {
        cout << "Device type: FTD2xx" << endl;
        robot->set_param(DynamixelDevice::COMTYPE, FTDevice::FTD2XX);
        renewed = true;
      } else {
        cout << "Invalid interface type: " << argv[a+1] << endl;
        return -1;
      }
      break;
    case 'f':
      break;
    default:
      printf("Invalid command -%c\n",argv[a][1]);
      return -1;
  }
  if (!renewed) cout << "Device type: FTDI" << endl;

  robot->set_param(DynamixelDevice::BAUDRATE, baudrate);
  cout << "Baudrate set to " << baudrate << endl;
  robot->set_param(DynamixelDevice::VERBOSITY, verb);
  robot->set_param(DynamixelDevice::TIMEOUT, timeout);
  robot->set_param(DynamixelDevice::LATENCY, latency);
  robot->connect();

  int _pos, _speed;
  char _command[10];
  int _params[10];
  int _imudata[6];
  unsigned int _i;
  int bsize = read.size() > 0 ? read.size() : 1;
  int pbuf[bsize], vbuf[bsize];
  unsigned char ibuf[bsize];
  for (_i=0; _i<read.size(); _i++)
    ibuf[_i] = read[_i];
  timeval start, end;
  int p;
  double total = 0.; 

  if (mode == Scan) {
    int found = 0;
    for (int i=0; i<256; i++) {
      if (robot->get_pos(i,_pos,_speed)) {
        printf("id %i: pos %i, speed %i\n",i,_pos,_speed);
        found++;
      }
    }
    std::cout << found << " servos found on bus at current baudrate." << std::endl;
    return 0;
  }

  if (mode == PIDTest) {
    for (_i=1; _i<=10; _i++) {
      total = 0;
      std::cout << "Speed " << (morph?_i:_i*100) << std::endl;
      robot->set_pos(imu, 500);
      do {
        robot->get_pos(imu, _pos, _speed);
        sleep(1);
      } while (abs(_pos-500)>5);
      robot->set_pos(imu, 600, 4/*morph?_i:_i*100*/);
      int last = 400;
      double stable = -1;
      do {
        gettimeofday(&start,NULL);
        robot->get_pos(imu, _pos, _speed);
        gettimeofday(&end,NULL);
        total += 1000.*(end.tv_sec-start.tv_sec)+(end.tv_usec-start.tv_usec)/1000.;
        cout << "[" << total << "] " << 1000.*(end.tv_sec-start.tv_sec)+(end.tv_usec-start.tv_usec)/1000. << " -> " << _pos << endl;
        if (last == _pos && stable == -1 && last == 600)
          stable = total;
        last = _pos;
      } while (total < 2000/*8000/_i*/);
      std::cout << "Stable at: " << stable << std::endl << std::endl;
    }
    return 0;
  }
 
  int cycle = 0;
  do {
    cycle++;
    gettimeofday(&start,NULL);
    p = 0;

    // read imu
    if (imu > -1) {
      for (_i=0; _i<6; _i++) _imudata[_i] = -1;
      ROBOTIS->get_imu(imu,&_imudata[0]);
      for (_i=0; _i<6; _i++) printf("i%i:%i ",_i,_imudata[_i]);
      p++;
    }

    // read servos
    if (read.size() > 0) {
      if (!sync_read) {
        for (_i=0; _i<read.size(); _i++) {
          _pos = _speed = -1;
          robot->get_pos(read[_i],_pos,_speed);
          printf("s%i:%i/%i ",read[_i],_pos,_speed);
        }
      } else {
        robot->get_pos(bsize, ibuf, pbuf, vbuf);
        for (_i=0; _i<read.size(); _i++)
          printf("s%i:%i/%i ",read[_i],pbuf[_i],vbuf[_i]);
      }
      p += _i;
    }

    // read parameters
    for (_i=0; _i<par[0].size(); _i++) {
      _pos = -1;
      if (morph)
        MORPHEUS->get_param(par[0][_i],(Morpheus::param)par[1][_i],_pos);
      else
        ROBOTIS->get_param(par[0][_i],(Robotis::param)par[1][_i],_pos);
      printf("p%x@s%i:%i ",par[1][_i],par[0][_i],_pos);
    }
    p += _i;

    if (p > 0) cout << endl;

    // get command
    if (command) {
      _command[0] = -1;
      for (_i=0; _i<10; _i++) _params[_i] = -1;
      cin >> _command;
      cin.ignore(100,'\n');

      if (CMD("help")) {
        cout << "Possible commands:" << endl;
        cout << "exit/quit: terminate program" << endl;
        cout << "reset:     reset a servo to factory values" << endl;
        cout << "change:    change the ID of a servo" << endl;
        cout << "move:      move a servo to a position with a given speed" << endl;
        cout << "quickmove: move a servo to a position with the default speed" << endl;
        cout << "set:       set a parameter on a servo (type 'params' for listing)" << endl;
      } else if (CMD("") || CMD(" ")) {
        ;
      } else if (CMD("exit") || CMD("quit")) {
        loop = false;
      } else if (CMD("reset")) {
        cout << "id: ";
        cin >> _params[0];
        robot->reset(_params[0]);
      } else if (CMD("change")) {
        cout << "old_id new_id: ";
        cin >> _params[0] >> _params[1];
        if (morph)
          MORPHEUS->set_param(_params[0],Morpheus::SERVO_ID,_params[1]);
        else
          ROBOTIS->set_param(_params[0],Robotis::SERVO_ID,_params[1]);
      } else if (CMD("quickmove")) {
        cout << "id position: ";
        cin >> _params[0] >> _params[1];
        robot->set_pos(_params[0],_params[1]);
      } else if (CMD("move")) {
        cout << "id goal_pos goal_speed: ";
        cin >> _params[0] >> _params[1] >> _params[2];
        robot->set_pos(_params[0], _params[1], _params[2]);
      } else if (CMD("set")) {
        cout << "id param value: ";
        cin >> _params[0] >> _params[2] >> _params[1];
        if (morph)
          MORPHEUS->set_param(_params[0],(Morpheus::param)_params[2],_params[1]);
        else
          ROBOTIS->set_param(_params[0],(Robotis::param)_params[2],_params[1]);
      } else if (CMD("params")) {
        if (morph) {
          cout << "SERVO_ID = 1" << endl;
          cout << "CTL_UP_LIMIT = 2" << endl;
          cout << "CTL_LW_LIMIT = 3" << endl;
          cout << "KPP = 4" << endl;
          cout << "KIP = 5" << endl;
          cout << "KDP = 6" << endl;
          cout << "KPV = 7" << endl;
          cout << "KIV = 8" << endl;
          cout << "KDV = 9" << endl;
          cout << "BAUDRATE = 10" << endl;
          cout << "POSITION = 91" << endl;
          cout << "SPEED = 13" << endl;
          cout << "TEMPERATURE = 94" << endl;
          cout << "PWM = 96" << endl;
        } else {
          cout << "SERVO_ID = 3" << endl;
          cout << "BAUDRATE = 4" << endl;
          cout << "LATENCY = 5" << endl;
          cout << "CW_ANGLE_LIMIT = 86" << endl;
          cout << "CCW_ANGLE_LIMIT = 88" << endl;
          cout << "MAX_TEMPERATURE = 11" << endl;
          cout << "MIN_VOLTAGE = 12" << endl;
          cout << "MAX_VOLTAGE = 13" << endl;
          cout << "MAX_TORQUE = 94" << endl;
          cout << "STATUS_RETURN_LEVEL = 16" << endl;
          cout << "ALARM_LED = 17" << endl;
          cout << "ALARM_SHUTDOWN = 18" << endl;
          cout << "TORQUE_ENABLE = 24" << endl;
          cout << "LED = 25" << endl;
          cout << "CW_COMPLIANCE_MARGIN = 26" << endl;
          cout << "CCW_COMPLIANCE_MARGIN = 27" << endl;
          cout << "CW_COMPLIANCE_SLOPE = 28" << endl;
          cout << "CCW_COMPLIANCE_SLOPE = 29" << endl;
          cout << "GOAL_POSITION = 110" << endl;
          cout << "MOVING_SPEED = 112" << endl;
          cout << "TORQUE_LIMIT = 114" << endl;
          cout << "REGISTERED_INSTRUCTION = 44" << endl;
          cout << "LOCK = 47" << endl;
          cout << "PUNCH = 128" << endl;
        }
      }
    }

    gettimeofday(&end,NULL);
    cout << "Elapsed time: " << 1000.*(end.tv_sec-start.tv_sec)+(end.tv_usec-start.tv_usec)/1000. << "ms" << endl << endl;
    total += 1000.*(end.tv_sec-start.tv_sec)+(end.tv_usec-start.tv_usec)/1000.;
  } while (loop);
  cout << "Average communication time: " << total/cycle << "ms, averaged over " << cycle << " cycles." << endl;
  robot->disconnect();
  delete robot;
  return 0;
}

