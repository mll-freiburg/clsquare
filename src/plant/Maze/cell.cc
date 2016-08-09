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

#include "cell.h"
  
  Cell::Cell() 
{
  next_cells = new Cell*[NUM_MAZE_ACTIONS];
  for (int i = 0; i < NUM_MAZE_ACTIONS;i++)
     next_cells[i] = 0;
  x = 0;
  y = 0;
  status = FREE;
}
  
Cell::Cell(int x,int y, int status)
{
  next_cells = new Cell*[NUM_MAZE_ACTIONS];
  for (int i = 0; i < NUM_MAZE_ACTIONS;i++)
     next_cells[i] = 0;
  this->x = x;
  this->y = y;
  this->status = status;
}

int Cell::get_X() 
{
  return x;
}

void Cell::set_X(int x)
{
  this->x = x;
}

int Cell::get_Y()
{
  return y;
}

void Cell::set_Y(int y)
{
  this->y = y;
}

int Cell::get_status()
{
  return status;
}

void Cell::set_status(int status)
{
  this->status = status;
}
  
Cell* Cell::get_next_cell(int action)
{
  return next_cells[action];
}
 
void Cell::set_next_cell(int action,Cell* cell)
{
  next_cells[action] = cell;
}

 
bool Cell::is_blocked(int action)
{
  if (next_cells[action] != 0) {
    int status = next_cells[action]->get_status();
    if (status == FREE || status == GOAL)
      return false;
  }
  return true;
}

Cell::~Cell()
{
  delete[] next_cells;
}
