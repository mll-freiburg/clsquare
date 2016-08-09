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


#ifndef _CELL_H_
#define _CELL_H_


/** Defines the constant for an obstacle. 
 * It is not possible to enter a cell, which is
 * an obstacle. */
#define OBSTACLE 0

/** Defines the constant for a free cell. */
#define FREE 1

/** Defines the constant for a cell.
 * If the sytem reaches a goal cell, the trial ends
 *  successfully. */
#define GOAL 2

/** Amount of actions.
 * -Actions
 * 0:Left
 * 1:Right
 * 2:Down
 * 3:Up */
#define NUM_MAZE_ACTIONS 4 
#define LEFT 0
#define RIGHT 1
#define DOWN 2
#define UP 3

/** Stores information about one cell of a maze-matrix.
 *  Includes also information about reachable neighbour cells
 *  with respect to executed actions.
 *  \see Maze MazeData
 *  \author Stephan Timmer */
class Cell{     
 private: 
  /** Position of a cell in the maze.
   * x: column of grid.
   * y: row of grid. */
  int x,y;
  
  /** The status of a cell.
   * Takes on of the three values:
   * \li FREE
   * \li OBSTACLE
   * \li GOAL
   */
  int status;

  /** Array of pointers to next cells.
   * The array stores the neighbour cells with respect to
   * the executed action. Index i corresponds to action i */
  Cell** next_cells;

 public:
  /** Constructs a cell. */
  Cell(); 

  /** Constructs a cell with position and status. 
   * \param x Column of cell.
   * \param y Row of cell.
   * \param status Status of cell.
   *
   *  Valid values of status are:
   * \li FREE
   * \li OBSTACLE
   * \li GOAL */
  Cell(int x,int y,int status);

  /** Free memory for cell. */
  ~Cell();

  /** Get column of cell. */
  int get_X();
  /** Set column of cell.
   * \param x column of cell. */
  void set_X(int x);
  /** Get row of cell. */ 
  int get_Y();
  /** Set row of cell.
   *  \param y row of cell. */
  void set_Y(int y);
  /** Get status of cell. */
  int get_status();
  /** Set status of a cell. */
  void set_status(int status); 
  
  /** Gives pointer to a neighbour cell.
   * \param action Action, which is executed to reach neighbour cell.
   * \return Pointer to neighbour cell. */
  Cell* get_next_cell(int action);

  /** Sets a neigbour cell.
   *   \param action Action, which is executed to reach the neighbour cell.
   *   \param cell Pointer to neigbour cell, which is reached by action. */
  void set_next_cell(int action,Cell* cell);
  
  /** Checks, if an action is not possible in current cell.
   * A neighbour cell may be blocked, because it contains an obstacle.
   * \param action Action, which leads to a neighbour cell.
   * \return true, if action is not possible. */
  bool is_blocked(int action);
  
};

#endif
