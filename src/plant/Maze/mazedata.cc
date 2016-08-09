/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck

Author: Stephan Timmer

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
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <stdio.h>
#include <iostream>
#include <ctype.h>
#include <cstdlib>
#include <cstring>
#include "mazedata.h"

#define OK 0
#define ERROR 1

using namespace std;

MazeData::MazeData(const char* file)
{ 
  maze_file = file;
  grid = 0;   
  if (!read_maze())
    exit(1);  
  compute_next_cells();
}

bool MazeData::determine_maze_boundaries()
{
  FILE* maze; 
  char s[MAX_LINE_LEN];
  bool first_maze_line = true;

  // Skip borders of maze (marked with X in maze-definition)
  x_extend = -3;
  y_extend = -2;

  // Open file
  if ((maze = fopen(maze_file,"r")) == NULL){
    std::cerr << "ERROR: " << __PRETTY_FUNCTION__ << ": cannot read definition of maze" << endl;    
    return(false);
  }

  // For each line
  while (fgets (s,MAX_LINE_LEN,maze) != NULL) {     
    // Jump over comments and empty lines
    if(*s=='\n'||*s =='%'||*s =='#'||*s==' '){
    }      
    // Calculate boundaries 
    else{    
      if(first_maze_line){
	first_maze_line = false;	 
	int i = 0;
	while(s[i] != '\0') {
	  x_extend++;
	  i++;
	}               
      }
      y_extend++;      
    }
  }

  // Close file
  fclose(maze);
  
  return(true);
}

void MazeData::print_maze() {  
  printf("\nno.columns %i\n",x_extend);
  printf("no.rows %i\n",y_extend);
  for (int j = y_extend-1; j >=0;j--) {
    for (int i = 0;i < x_extend;i++) {     
      int status = grid[j*x_extend + i]->get_status();
      if (status == GOAL) {     	            
	printf("G");
      }
      else if (status == FREE) {
	printf(".");
      }
      else if (status == OBSTACLE) {
	printf("X");
      }
      else {
	std::cerr << "ERROR: " << __PRETTY_FUNCTION__ << ":  Inner representation of the maze is corrupt! " << endl;
	exit(1);
      }      
    }
    printf("\n");
  }
  printf("\n");
}


bool MazeData::parse_config_line(char *s,int* row)
{   
  // Jump over comments and empty lines
  if(*s=='\n'||*s =='%'||*s =='#'||*s==' '){
  } 
  else{   
    if (maze_first_line || *row < 0)     
      maze_first_line = false;     
    else {      
      for(int col=0;col < x_extend;col++){		
	int status;
	switch(s[col+1]) {
	case 'X':
	  status = OBSTACLE;
	    break;
	case '.':
	  status=FREE;
	  break;
	case 'G':
	  status=GOAL;
	  break;     
	default:
	  std::cerr << "ERROR: " << __PRETTY_FUNCTION__ << ":  Illegal cell status " << endl;
	  return(false);
	} 
	// Construct new cell 
	Cell* cell = new Cell(col,*row,status);
	grid[(*row)*x_extend + col] = cell;
      }      
      (*row)--;
    }          
  }
  return(true);  
}

bool MazeData::read_maze() 
{ 
  FILE* maze;
  char s[MAX_LINE_LEN];

  // Determine the boundaries of the maze
  if(!determine_maze_boundaries())
    return(false);

  grid = new Cell*[x_extend*y_extend];
  // Open file
  if ((maze = fopen (maze_file,"r")) == NULL){
    std::cerr << "ERROR: " << __PRETTY_FUNCTION__ << ": cannot read definition of maze" << endl;      
    return(false);
  }
  
  maze_first_line = true;
  int* row = new int(y_extend-1); 
  // Read every single line of file and parse it 
  while (fgets (s,MAX_LINE_LEN,maze) != NULL) {     
    if(!parse_config_line(s,row))    
      return(false);    
  }

  fclose(maze); 
  return(true);
}

void MazeData::compute_next_cells()
{
 int new_col = 0;
  int new_row = 0;
 
  for (int col = 0;col < x_extend;col++){
    for (int row = 0;row < y_extend;row++){ 
      for(int action=0; action < NUM_MAZE_ACTIONS;action++){
	switch(action){  	  
	  // Left
	case LEFT:
	  new_col=col-1;
	  new_row= row;
	  break;
	  // Right
	case RIGHT:
	  new_col=col+1;
	  new_row=row;
	  break;
	  // Down
	case DOWN:
	  new_col=col;
	  new_row=row+1;
	  break;
	  // Up
	case UP:
	  new_col=col;
	  new_row=row-1;
	  break;
	  // Illegal
	default:
	  std::cerr << "ERROR: " << __PRETTY_FUNCTION__ << ": illegal action" << endl;      	 
	}

	if (new_col < 0 || new_col >= x_extend)      
	  new_col = col;
	if (new_row < 0 || new_row >= y_extend)
	  new_row = row;

	Cell* cell = grid[row*x_extend + col];
	Cell* new_cell = grid[new_row*x_extend + new_col];	
	if(cell->get_status() == OBSTACLE || new_cell->get_status() == OBSTACLE || cell->get_status() == GOAL)
	  cell->set_next_cell(action,cell);		
	else
	  cell->set_next_cell(action,new_cell);	 
      }
    }
  }
}

int MazeData::get_extend_X()
{
  return x_extend;
}

int MazeData::get_extend_Y()
{
  return y_extend;
}

Cell* MazeData::get_cell(int col,int row)
{
  return grid[row*x_extend + col];
}

MazeData::~MazeData()
{
  for (int i = 0; i < y_extend*x_extend;i++)    
      delete grid[i];
}
