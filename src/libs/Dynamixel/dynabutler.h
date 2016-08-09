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

#ifndef _DYNABUTLER_H_
#define _DYNABUTLER_H_

#include "morpheus.h"
#include "robotis.h"

#define DBMAXJOINTS 20

/** Defines default movement ranges and IDs for the Dynabutler arm. */
namespace AX12Arm
{
  const int min[6] = {556,  92,  99,    0, 100,    0};
  const int max[6] = {982, 880, 927, 1023, 750, 1023};
  const unsigned char ids[8] = {1, 5, 4, 2, 3, 6, 7, 8};
}

/** Implements a wrapper for a 6-DOF arm made from AX-12 servos.
  * Communication is realized through the FTD2xx library.
  * Servo IDs are hidden by the interface, and joint numbers are
  * used for control instead, with joint IDs being numbered from
  * base to effector. */
class Dynabutler
{
 public:
  Dynabutler();
  ~Dynabutler();

  /** Puts the arm into free-run mode. */
  void free () throw ();

  /** Moves a single joint to a specified position.
    * \param joint number of the joint to be moved, 0 being the base
    * \param goal target encoder position
    * \param speed target movement speed
    * \return false if command could not be sent */
  bool set_pos (unsigned char joint, int goal, int speed=2) throw ();

  /** Moves all six joints to specified positions with a default speed.
    * \param goal array of length 6 containing the goal encoders for each joint
    * \return false if command could not be sent */
  bool set_pos (int* goal) throw ();

  /** Moves all six joints to specified positions with a specified speed.
    * \param goal array of length 6 containing the goal encoders for each joint
    * \param speed array of length 6 containing the movement speeds for each joint
    * \return false if command could not be sent */
  bool set_pos (int* goal, int* speed) throw ();

  /** Determines the current encoder position of a single joint.
    * \param joint number of the joint to be read, 0 being the base
    * \param position return variable for the determined position
    * \return false if data could not be read */
  bool get_pos (unsigned char joint, int& position) throw ();

  /** Determines the current encoder position of all six joints.
    * \param buffer array of length 6 that the positions will be stored in
    * \return false if data could not be read */
  bool get_pos (int* buffer) throw ();

  /** Returns a pointer to the underlying Morpheus AX-12 device for manual control of the arm.
    * \return handle to the base device */
  inline MorpheusDevice* get_morpheus () throw () {
    return _arm;
  };

  /** Returns a pointer to the underlyingd Robotis MX-28 device for manual control of the shoulder.
    * \return handle to the base device */
  inline RobotisDevice* get_robotis () throw () {
    return _shoulder;
  };

  /** Registers multiple additional joints.
    * \param num number of joints to be added
    * \param ids servo IDs for the added joints
    * \param min lower movement limits for the added joints
    * \param max upper movement limits for the added joints
    * \return false if the new number of joints would exceed the maximum */
  bool add_joints (int num, const unsigned char* ids, const int* min, const int* max) throw ();

  /** Registers one additional joint.
    * \param id servo ID for the added joint
    * \param min lower movement limit for the added joint
    * \param max upper movement limit for the added joint
    * \return false if the new number of joints would exceed the maximum */
  bool add_joint (const unsigned char ids, const int min, const int max) throw ();

  /** Waits until the arm has arrived at a specified goal position.
    * \param goal goal positions for all joints
    * \param tolerance encoder distance the arm has to be from the goal to be considered there
    * \param timeout number of 500ms-cycles until aborting
    * \return false if function timed out */
  bool wait (const int* goal, int tolerance=10, int timeout=10) throw ();

  /** Sets the behaviour for out-of-range target positions.
    * If set to cropping more, invalid targets will be reduced to fit the movement range;
    * otherwise, invalid commands will be ignored.
    * \param whether to use cropping or not */
  inline void set_crop (bool crop=true) throw () {_crop = crop;};

 protected:
  
  MorpheusDevice *_arm;
  RobotisDevice  *_shoulder;
  int _i, _target, _joints;
  int _pos[DBMAXJOINTS+2], _vel[DBMAXJOINTS+2], _spd[DBMAXJOINTS], _min[DBMAXJOINTS], _max[DBMAXJOINTS];
  unsigned char _tmp_ids[2], _ids[DBMAXJOINTS+2];
  bool _success, _crop;
  bool check_range (int& target, const int joint) throw ();
};

#endif

