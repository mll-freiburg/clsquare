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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  OF
 THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "mazegraphic.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <unistd.h>       /* usleep */
#include "valueparser.h"
#include <cassert>

/*****************************************************************
 * Graphic implementation
 *****************************************************************/

using namespace std;

void MazeGraphic::sendMaze()
{ 
  static char zeile[512];
  int x_l,y_l;
  bool drawn = false;
  stringstream out;
  int status;

  x_l = maze->get_extend_X()-1;
  y_l = maze->get_extend_Y()-1;
  
  out << "HIDE 0;";
  sprintf(zeile,"EMP 0; VA (%i,%i,%i,%i); ",(x_l+1)/2,(y_l+1)/2,x_l+3,y_l+3);
  out << zeile;

  for(int y= 0; y <= y_l; y++) {
    sprintf(zeile,"INS 0 LINE lay=2 col=0000ff (0,%i,%i,%i); ",y,x_l+1,y);
    out << zeile;    
    for(int x= 0; x <= x_l; x++) {     
      status = maze->get_cell(x,y)->get_status();   
      if (status == OBSTACLE) {
        sprintf(zeile,"INS 0 POLYGON lay=2 fil=1 col=0000ff (%i,%i)(%i,%i)(%i,%i)(%i,%i); ",
          x,y,x+1,y,x+1,y+1,x,y+1);
        out << zeile;	
	
      }
      if (status == GOAL) {
        sprintf(zeile,"INS 0 POLYGON lay=3 fil=0 col=ff0000 (%i,%i)(%i,%i)(%i,%i)(%i,%i); ",
          x,y,x+1,y,x+1,y+1,x,y+1);
        out << zeile;      
      }
      
      if (!drawn) {
        sprintf(zeile,"INS 0 LINE lay=2 col=0000ff (%i,0,%i,%i); ",x,x,y_l+1);
        out << zeile;       
      }
      
      if (out.str().size()>=7000) {
        sock.send_msg(out.str().c_str(),out.str().size());  
        out.str("");
      }
    } 
    drawn = true;
  }

  sprintf(zeile,"INS 0 LINE lay=2 col=0000ff (0,%i,%i,%i); ",y_l+1,x_l+1,y_l+1);
  out << zeile;
  sprintf(zeile,"INS 0 LINE lay=2 col=0000ff (%i,0,%i,%i); ",x_l+1,x_l+1,y_l+1);
  out << zeile;
  out << "SHOW 0;";

  sock.send_msg(out.str().c_str(),out.str().size());  

}

bool MazeGraphic::notify(const double *plant_state, const double *observation, const double *reference_input, 
                         const double *action, const long cycle, const long sequence, const float total_time, 
                         const float sequence_time, const long total_num_of_cycles)
{
  stringstream out;
  char zeile[512];
  float w;

  if(graphic_active == false)
    return true;

  if (!mazeSent)
    sendMaze();
  mazeSent = true;  
   
  // Compute angle 
  if (action[0]==1) w = 1.5*PI;
  else
    if (action[0]==3) w = PI;
    else
      if (action[0]==2) w = 0;
      else
        w = 0.5*PI;

  // Build player 
  if (!plBuilt) {
    sprintf(zeile,"INS 0 FRAME id=5 lay=10 (%f,%f); INS 5 CIRCLE lay=10 col=ff0000 (0,0,0.4); INS 5 LINE lay=10 col=ff0000 (0,-0.4,0,0.4); ",0.5,0.5);
    out << zeile;
    sprintf(zeile,"INS 5 LINE lay=10 col=ff0000 (0,0.4,0.2,0); INS 5 LINE lay=10 col=ff0000 (0,0.4,-0.2,0); ");
    out << zeile;
    plBuilt = true;
  }

  // Change position and angle 
  
  double x=plant_state[0], y=plant_state[1];
  if (useObservation) {
    x = observation[0];
    y = observation[1];
  }
  sprintf(zeile,"MOV %i (%f,%f,%f); ",5,x+0.5,y+0.5,w);
  out << zeile;
  
  sock.send_msg(out.str().c_str(),out.str().size());
  return true;
}


bool MazeGraphic::deinit() {
  sock.close();
  return true;
}

void MazeGraphic::read_options(const char * fname) 
{
  if(fname == 0)
    return;

  ValueParser vp(fname,"Graphic");
  useObservation = false;
  vp.get("observation",useObservation);
  vp.get("active",graphic_active);
  vp.get("port",port);
 
}

bool MazeGraphic::init(int state_dim, int observation_dim, int action_dim, int reference_input_dim, double _delta_t, const char *fname)
{
  graphic_active= true;
  mazeSent = false;  
  plBuilt = false;  
  maze = new MazeData("maze.def");  
  // TCP-port Frameview;
  port= 20000;   
  sprintf(hostname,"localhost");
  read_options(fname);

  if(graphic_active){
    // Build TCP-connection 

    if ( !TCPutils::init_client(hostname,port,sock) ) 
       std::cerr << "ERROR: " << __PRETTY_FUNCTION__ << ": cannot establish TCP connection" << endl;         
  }

  assert (!useObservation || observation_dim >= 2);
  
  return true;
}

MazeGraphic::~MazeGraphic()
{
  delete maze;
}


REGISTER_GRAPHIC(MazeGraphic, "Visualizes the maze environment")

