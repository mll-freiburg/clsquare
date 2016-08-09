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


#ifndef _MAZEDATA_H_
#define _MAZEDATA_H_

#define MAX_LINE_LEN 500

#include "cell.h"

/** Implements a maze, that consists of a matrix of cells.
 * Four actions are possible in a 2-dimensional maze:
 * \li left (0)
 * \li right (1)
 * \li down (2)
 * \li up (3)
 * 
 * An action is not possible in a cell, 
 * if it leads to an obstacle (neighbour) cell. 
 * This class defines methods for reading a 
 * maze-definition from a file and
 * inspecting cells at a certain positions in the matrix. 
 * \see Cell Maze
 * \author Stephan Timmer */
class MazeData {
 private: 
  /** Matrix of cells.
   * Array of pointers to cells. */
  Cell** grid;

  /** Name of maze-definition file. */
  const char* maze_file;

  /** Width of maze. */
  int x_extend; 
  /** Height of maze. */
  int y_extend; 
   
  /** Reads maze from file.
   *  Reads the maze-definition file and 
   *  initialize data structures. 
   *  \return true, for success. */
  bool read_maze();

  /** Determines size of maze.
   * Checks widht and height of maze. */
  bool determine_maze_boundaries();

  /** Parse one line from the maze-definition file.
   * \param line One line from the maze-definition file.
   * \param row Which row should be read.
   * \return true, for success. */     
  bool parse_config_line(char* line,int* row);

  /** Helps to parse line. */
  bool maze_first_line;

  /** Computes neighbour cells for all cells in the maze.
   * Checks the effect of all four actions in each cell.*/
  void compute_next_cells();
  
 public:
  
  /** Constructs maze from file.
   \param file Maze-definition file. */
  MazeData(const char* file);
 
  /** Prints the maze.
   * Print maze to standard out.*/
  void print_maze();

  /** Gives width of maze.
   \return Width of maze. */
  int get_extend_X();
  /** Gives height of maze.
      \return Height of maze. */
  int get_extend_Y();

  /** Gives cell at a certain position.
   \param col Column of cell.
   \param row Row of cell.
   \return Pointer to cell at (col,row). */
  Cell* get_cell(int col,int row);

  /** Free memory for maze data-structures. */
  ~MazeData();
};

#endif
