/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Adapted from a previous version of maze visualisation by Ralf Schoknecht and Artur Merke

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
#include <string.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "valueparser.h"
#include "unipole_graphic.h"

/*****************************************************************
 * Cartpole Graphic
 *****************************************************************/

bool UniversalCartPoleGraphic::notify(const double *state, const double *observed_state,const double *reference_input, 
			     const double *action, const long cycle, const long episode, const float total_time, 
			     const float episode_time, const long total_num_of_cycles)
{
if (!graphic_active)
    return true;
  if(x_dim <2) // canot display anything
    return true;

  static char buffer[2000];
  float pole_angle= -state[0]; 
  float pole_angle2= -state[2]; 

  if(x_dim <4){// display either cart or pole
    if(cart_only==true){ // display only cart
      sprintf(buffer,"MOV 1 (%.2f,0); SL \"(pos=%5.2f, vel=%5.2f) act=%4.2f cyc=%ld seq=%ld lap=%5.2f total=%5.2f \";", 
	      state[0],
	      state[0], state[1],
	      action[0], cycle, episode, episode_time, total_time);
    }
    else{ // default: move pole only
      sprintf(buffer,"MOV 2 (%.2f); SL \"(ang=%5.2f, ang_vel=%5.2f) act=%4.2f cyc=%ld seq=%ld lap=%5.2f total=%5.2f \";", 
	      pole_angle, 
	      state[0], state[1],
	      action[0], cycle, episode, episode_time, total_time);
    }
  }
  else if(x_dim == 6){ // cart double pole
    float pole2_x = scaled_pole_len * sin(state[0]) + state[4];
    float pole2_y = scaled_pole_len * cos(state[0]) + pole1_origin_y;

    if (parallel)
      sprintf(buffer,"MOV 1 (%.2f,0); MOV 2 (%.2f); MOV 3 (%.2f); SL \"(ang=%5.2f, ang_vel=%5.2f,ang2=%5.2f, ang2_vel=%5.2f, pos=%5.2f, vel=%5.2f) act=%4.2f cyc=%ld seq=%ld lap=%5.2f total=%5.2f \";",
        state[4], pole_angle, pole_angle2,
        state[0], state[1], state[2], state[3], state[4], state[5],
        action[0], cycle, episode, episode_time, total_time);
    else
      sprintf(buffer,"MOV 1 (%.2f,0); MOV 2 (%.2f); MOV 3 (%.2f,%.2f,%.2f); SL \"(ang=%5.2f, ang_vel=%5.2f,ang2=%5.2f, ang2_vel=%5.2f, pos=%5.2f, vel=%5.2f) act=%4.2f cyc=%ld seq=%ld lap=%5.2f total=%5.2f \";", 
  	    state[4], pole_angle,  pole2_x, pole2_y,  pole_angle2,
	      state[0], state[1], state[2], state[3], state[4], state[5],
	      action[0], cycle, episode, episode_time, total_time);
  }
  else{// move complete cart pole system
    sprintf(buffer,"MOV 1 (%.2f,0); MOV 2 (%.2f); SL \"(ang=%5.2f, ang_vel=%5.2f, pos=%5.2f, vel=%5.2f) act=%4.2f cyc=%ld seq=%ld lap=%5.2f total=%5.2f \";", 
	    state[2]*scale, pole_angle, 
	    state[0], state[1], state[2], state[3],
	    action[0], cycle, episode, episode_time, total_time);
  }
  
  sock.send_msg(buffer, strlen(buffer) );

  return true;
}

bool UniversalCartPoleGraphic::init(int _state_dim, int _observation_dim, int _action_dim, int _reference_input_dim, double _delta_t, const char *fname)
{
  std::stringstream out;
  x_dim = _state_dim;
  u_dim = _action_dim;
  graphic_active= true; 
  parallel = false;
  port = 6010;
  sprintf(hostname,"localhost");
  cart_only = false;
  scale  = 3.0;
  pole_length = 0.2;

  if (!read_options(fname))
    return false;

  if (x_dim == 6)  // cart double pole
    scaled_pole_len= scale * pole_length;
  else
    scaled_pole_len = 2 * (scale * pole_length);
 
  double car_length= scale * 0.1;
  double car_height= scale * 0.05;
  pole1_origin_y = car_height * 1.25;

  if (!graphic_active)
    return true;

  sock.init_socket_fd();
  //  sock.init_serv_addr("localhost",port);
  //  sock.init_serv_addr("129.217.56.247",port);
  sock.init_serv_addr(hostname,port);

  out << "VA (0,0,12,8); EMP 0;INS LINE lay= -1 (-5,0,5,0);INS FRAME id=1;" 
      <<"INS LINE lay= -1 (0,0,0,-.4);" 
      <<"INS LINE lay= -1 (" << 0.05 * scale << ",0," << 0.05 * scale << ",-.2);" 
      <<"INS LINE lay= -1 (" << -0.05 * scale << ",0,"<< -0.05 * scale << ",-.2);" 
      <<"INS LINE col=ff0000 lay= -1 (" << 0.35 * scale << ",0," << 0.35 * scale << ",-.2);" 
      <<"INS LINE col=ff0000 lay= -1 (" << -0.35 * scale << ",0," << -0.35 * scale << ",-.2);" 
      <<"INS LINE lay= -1 (" << -1 * scale << ",0," << -1 * scale << ",-.2);" 
      <<"INS LINE lay= -1 (" << 1 * scale << ",0," << 1 * scale << ",-.2);" 
      <<"INS LINE lay= -1 (" << 2 * scale << ",0," << 2 * scale << ",-.2);" 
      <<"INS LINE lay= -1 (" << -2 * scale << ",0," << -2 * scale << ",-.2);" 
      << "INS 1 POLYGON fil=1 col=cc3333 "
      << "(" << -car_length*0.5 << "," << 0.25 * car_height << ")"
      << "(" <<  car_length*0.5 << "," << 0.25 * car_height << ")"
      << "(" <<  car_length*0.5 << "," << 1.25 * car_height << ")"
      << "(" << -car_length*0.5 << "," << 1.25 * car_height << ");" 
      << "INS 1 LINE col=0000FF (0,"<< 0.25 * car_height << ",0,-.5);"

      << "INS 1 FRAME id=2 lay=1 (" << (parallel ? car_length * -0.25 : 0) << "," <<  pole1_origin_y << ");"
    //      << "INS 2 LINE col=0000FF (0,0,0,-.5);"
    << "INS 2 CIRCLE col=000000 (0,"<< scaled_pole_len<< ",.05);"
      << "INS 2 LINE col=000000 (0,0,0," << scaled_pole_len << ");";

  if (x_dim == 6){
    if (!parallel) {
      out << "INS FRAME id=3 lay=2 (0," <<  car_height * 1.25 + 0.1 * scaled_pole_len + scaled_pole_len << ");"
          << "INS 3 LINE col=000000 (0,0,0," << scaled_pole_len << ");";
    } else {
      out << "INS 1 FRAME id=3 lay=2 (" << car_length * 0.25 << "," << pole1_origin_y << ");"
          << "INS 3 CIRCLE col=000000 (0,"<< scaled_pole_len/2<< ",.05);"
          << "INS 3 LINE col=000000 (0,0,0," << scaled_pole_len/2 << ");";
    }
  }
// To show the tires switch to 1
#if 0

  out << " INS 1 CIRCLE lay= 1 "
      << "(" << -car_length * 0.375 << "," << 0.25 * car_height << "," << 0.25 * car_height << ")"
      << "(" <<  car_length * 0.375 << "," << 0.25 * car_height << "," << 0.25 * car_height << ");" 

#endif

    sock.send_msg(out.str().c_str(), out.str().length());
  return true;
}

bool UniversalCartPoleGraphic::deinit()
{
 sock.close_socket_fd();
  return true;
}

bool UniversalCartPoleGraphic::read_options(const char * fname) 
{
  if(fname == 0)
    return true;
  
  ValueParser vp(fname,"Graphic");

  vp.get("active",graphic_active);
  vp.get("cart_only",cart_only);
  vp.get("parallel",parallel);
  vp.get("port",port);
  vp.get("hostname",hostname,500);
   
  vp.get("scale",scale);
  
  if (!vp.get("pole_length",pole_length)) {
    ValueParser vp1(fname,"Plant");
    vp1.get("length_pole1",pole_length);
  }
    
  return true;
}

REGISTER_GRAPHIC(UniversalCartPoleGraphic, "visualizes the state of the cart pole")
