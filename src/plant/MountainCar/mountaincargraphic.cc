/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Adapted from a previous version of moutaincar visualisation by Ralf Schoknecht and Artur Merke

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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USEOF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/


#include <stdio.h>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include "valueparser.h"
#include "mountaincargraphic.h"


bool MountainCarGraphic::notify(const double *plant_state, const double *observation, const double *reference_input, 
                    const double *action, const long cycle, const long episode, const float total_time, 
                    const float episode_time, const long total_num_of_cycles)
{  
  static char buffer[200];
  float pos_x = plant_state[0];
  float pos_y;

  if (!graphic_active)
    return true;

  if (pos_x < 0) 
    pos_y= pos_x*pos_x+pos_x;
  else
    pos_y= pos_x / sqrt( 1.0 + 5.0 * pos_x * pos_x );

  float ang;
  {
    float tangent;
    if (pos_x < 0) 
      tangent= 2.0*pos_x+1.0;
    else 
      tangent= 1/(sqrt(pow(1.0 + 5.0*pos_x*pos_x,3.0)));
    ang= atan(tangent);
  }

 char dum[7]="";
 sprintf(buffer,"MOV 1 (%f,%f,%f); SL \"(pos= %.2f, vel= %.2f)  act= %.2f, run= %ld, time= %f %s\";", 
	 pos_x, pos_y, ang, 
         plant_state[0], plant_state[1], action[0], episode ,total_time, dum);
 
 sock.send_msg(buffer, strlen(buffer) );
 return true;
}

bool MountainCarGraphic::deinit(){
  sock.close_socket_fd();
  return true;
}

bool MountainCarGraphic::init(int state_dim, int observation_dim, int action_dim, int reference_input_dim, double _delta_t, const char *fname)
{
  graphic_active= true;
  port= 6010;
  sprintf(hostname,"localhost");

  read_options(fname);
  if (!graphic_active)
    return true;
  if (state_dim != 2) 
    return false;
  sock.init_socket_fd();
  sock.init_serv_addr(hostname,port);

  std::stringstream out;
  double scale= 0.5;
  double car_length= scale * 0.2;
  double car_height= scale * 0.1;
  const int num_points_for_plot= 400;

  // out << "VA (0,0.5,4,1.5) EMP 0
  out << "VA (0,0.5,2.9,1.5); EMP 0;"
      << "\nINS FRAME id=1 lay=1;"
      << "\nINS 1 POLYGON fil=1 col=cc3333 "
      << "(" << -car_length*0.5 << "," << 0.25 * car_height << ")"
      << "(" <<  car_length*0.5 << "," << 0.25 * car_height << ")"
      << "(" <<  car_length*0.5 << "," << 1.25 * car_height << ")"
      << "(" << -car_length*0.5 << "," << 1.25 * car_height << ");" 
      << ""
      << "\nINS 1 CIRCLE lay=1 "
      << "(" << -car_length * 0.375 << "," <<  0.25 * car_height << "," << 0.25 * car_height << ")"
      << "(" <<  car_length * 0.375 << "," <<  0.25 * car_height << "," << 0.25 * car_height << ");";
  out << "INS POINT col=cc3333";
  for (int i=0; i<num_points_for_plot; i++) {
    float x= -1.1 + float(i)/float(num_points_for_plot) * 2.0;
    float y;
    if (x<0.0)
      y= x*x+x;
    else
      y= x / sqrt( 1.0 + 5.0 * x*x );
    out << "(" << setprecision(6) <<  x << "," << setprecision(6) << y << ")";
  }
  out << ";";
  
  sock.send_msg(out.str().c_str(), out.str().length());
   
  return true;
}

bool MountainCarGraphic::read_options(const char * fname) 
{
  if(fname == 0)
    return true;
  
  ValueParser vp(fname,"Graphic");

  vp.get("active",graphic_active);
  vp.get("hostname",hostname,500);
  vp.get("port",port);
  return true;
}


REGISTER_GRAPHIC(MountainCarGraphic, "Visualizes the state of the mountain car");

