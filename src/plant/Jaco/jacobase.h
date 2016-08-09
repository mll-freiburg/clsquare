/*
clsquare - closed loop simulation system
Copyright (c) 2010-2012 Machine Learning Lab, 
Prof. Dr. Martin Riedmiller, University of Freiburg

Author: Thomas Lampe

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

#ifndef _JACOBASE_PLANT_H_
#define _JACOBASE_PLANT_H_

#include "plant.h"
#include <mono/jit/jit.h>
#include <mono/metadata/mono-config.h>
#include <mono/metadata/assembly.h>
#include <mono/metadata/debug-helpers.h>
#include <vector>
#include "setdef.h"

/** Provides a general-purpose base plant for use with a Kinova Jaco robotic arm.
  *
  * The plant state consists of:
  * \li 6x cartesian position
  * \li 6x angular position
  * \li 3x finger position
  * \li API control: 1 if active, 0 otherwise
  * \li 6x cartesian force
  * \li 6x angular force
  * \li 9x currents (includes fingers)
  * \li 6x cartesian speed
  * \li 6x angular speed
  * \li 3x finger speed
  *
  * It accepts a 9-dimensional action, the semantics of which is decided
  * by the parameter \b action_mode. It can be set to:
  * \li \e joystick to interpret it as joystick events; each dimension is a value in
  *     the interval [-1;1] that resembles a signed percentage of the
  *     maximum velocity.
  *     \todo finger movement does not work properly in joystick mode, and is only
  *           possible if all other DOFs remain static, since the arm needs to be
  *           switched to position mode; would need special control mapping
  * \li \e position to interpret them as target positions in the chosen reference
  *     frame; movement limits can be specified by \b pos_min and \b pos_max, both of
  *     which are lists of length 9
  * \li \e direction emulates the joystick mode through position control and
  *     interprets actions as steps into one direction with a fixed step size (1 in
  *     cartesian mode or 90 in angular mode); the main difference compared to
  *     joystick mode lies in the ability to use \b pos_min and \b pos_max, but
  *     in exchange there is no support for unique velocities per DOF. Actions with an
  *     absolute value equal to or below \b dir_threshold (float, default 0.01) will
  *     be interpreted as 0.
  * \li \e position_with_speed works just as \e position, but expects a 12-dimensional
  *     action, with the last three dimensions being the linear speed of the arm
  *     (0-0.15 m/s), the joint speed (0-0.6 rad/s) and the finger speed (0-0.15 m/sec),
  *     respectively. Speeds that are out of range are ignored and the default speed
  *     is used.
  * \li \e direction_with_speed is analogous to \e position_with_speed for direction
  *     mode.
  *
  * Furthermore, the meaning of the action is influenced by the parameter \b cartesian,
  * which decides whether the first six dimensions specify directions/positions in
  * cartesian space or in joint space.
  * Note that angular control disables any and all security features, such as
  * safety zones and self-collision avoidance.
  *
  * The plant supports a number of additional parameters:
  * \li \b position_mode (bool, default \e false):
  *     <em> Used only as a fallback if \e action_mode has not been specified. </em>
  *     If true, actions will be interpreted as positions, otherwise as joystick
  *     commands.
  * \li \b reclaim (bool, default \e false):
  *     If set to \e true, the API will try to reclaim control over the robot at the
  *     beginning of each episode if it has been lost previously (for instance if the
  *     joystick was used).
  * \warning Use with caution! This option may prevent you from using the joystick
  *     to stop the arm from moving in dangerous situations. 
  * \li \b manual_mode (bool, default \e false):
  *     If set to \e true, episodes will not be aborted once API control is lost,
  *     allowing one to record user-generated trajectories.
  * \warning Use with caution! This option may prevent you from using the joystick
  *     to stop the arm from moving in dangerous situations. 
  * \li \b wait_threshold (6 \f$\times\f$ float, default 0.02 for \e cartesian=true,
  *     1.0 for \e false):
  *     Sets the position deviation threshold for all waiting movement, particularly
  *     that to the initial position at the beginning of an episode. Once the robot
  *     does not move any more, if any degree of freedom deviates more than the
  *     threshold, the pose will be rejected.
  * \li \b drop (float, default -1.0):
  *     Defines a position that the robot's fingers will take at the end of each
  *     episode to achieve object dropping or pickup. If set to -1, no finger
  *     movement will occur.
  * \li \b preinit_x (9 \f$\times\f$ float, no default):
  *     Defines a trajectory that is to be performed before checking any initial
  *     states. \e x is a continuous index starting at 0.
  * \li \b profile (string, no default):
  *     Names a Jaco profile (saved under HOME/Kinova/...) to be loaded during
  *     initialization. Primarily used to guarantee that a set of protection
  *     zones will be loaded. Note that changed zones will only take place
  *     when the robot is restarted (for which the plant will offer an
  *     opportunity).
  * \li \b delta_t (int, default 1000):
  *     Sets the number of microseconds that the system will set between setting
  *     a command and reading the sensor data.
  * 
  * @ingroup PLANT
  * @ingroup HARDWARE
  * @author Thomas Lampe
  **/
class JacoBasePlant : public Plant {
public:
  JacoBasePlant();
  ~JacoBasePlant();

  // specification as per plant.h
  bool get_next_plant_state (const double *current_state, const double *current_action, double *next_state);
  bool get_measurement (const double *state, double *measurement);
  bool init (int& state_dim, int& measurement_dim, int& action_dim, double& delta_t, const char *fname=0, const char *chapter=0); 
  void notify_episode_stops ();
  void notify_episode_starts ();
  void notify_command_string (const char* buf);
  void deinit ();
  bool check_initial_state (double* initial_state);

protected:
  bool wait ();
  MonoMethod *_stop, *_deinit, *_get, *_set, *_pos, *_grip, *_reclaim, *_finger, *_posspeed;
  MonoObject **exception, *result;
  MonoDomain *_domain;
  void *_cargs[13], *_pargs[13], *_gargs[1];
  float _cmd[12], _wait_thresh[9], _gpos, _hmin[9], _hmax[9], _ainit[9], _cinit[9], _init[9], _increment, _dir_thresh;
  int _i, _j, _sdim, _adim, _inithelp, _offset, _soff, _pause, _episode;
  long int _delta_t;
  bool _cartesian, _reclaim_control, _in_init_region, _drop, _mono_ready, _chktarget, _abort, _manual, _fingers, _continue_at_exception, _finger_warning;
  std::vector<SetDef> _preinit;
  enum {Position, Direction, Joystick} _amode;
};

#endif

