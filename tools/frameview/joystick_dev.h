/*
 * Copyright (c) 1999 - 2001, Artur Merke <amerke@ira.uka.de> 
 *
 * This file is part of FrameView2d.
 *
 * FrameView2d is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * FrameView2d is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FrameView2d; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef _JOYSTICK_DEV_H_
#define _JOYSTICK_DEV_H_

#include "input_dev.h"
#include "valueparser.h"
#include "global_defs.h"

class JoystickDevice: public InputDevice {
  int joystick_fd;
  int num_axes;
  int num_buttons;
  int version;
  char name[MAX_NAME_LEN];

  int * button;
  int * axis;
  bool * button_chg;
  bool * axis_chg;

  struct Options {
    Options() { strcpy(joydev,"/dev/input/js0"); }
    char joydev[MAX_NAME_LEN];
   };

  Options options;

  bool process_options(const ValueParser & vp);

  template <class T>
  void print_option_entry(ostream & o,int mode, const char * option, const T & val, const char * description) const;
  void generic_description_of_options(ostream & o, int mode) const;

  int b_frame(int b) { return b+60; }
  int a_frame(int a) { return 20 + a/2; }

  public:
  JoystickDevice();
  bool process_options(const char * fname);
  bool process_options(int argc, char const* const* argv);
  //bool process_char_command(BuilderBase * build, char c,double x, double y) { return false; }
  void help_options(ostream & o) const;
  void help_char_command(ostream & o) const { o << "\njoystick commands:\n\n<no commands>"; }
  void generate_file_options(ostream & o) const;

  bool init_frames(BuilderBase * build);
  bool init_connection();

  bool destruct();

  bool process_input(fd_set * , BuilderBase * build);

  //const char * status_line() { return state_string; }
  int set_fds(fd_set * set);// { add_fd_to_set(sock.socket_fd,set); return sock.socket_fd; }
  bool got_fds(fd_set * set);// { return is_fd_in_set(sock.socket_fd,set); }
};

template <class T>
void JoystickDevice::print_option_entry(ostream & o,int mode, const char * option, const T & val, const char * description) const {
  if (mode == 0) 
    o << "\n"
      << "\n# " << description << " (default " << val << ")"
      << "\n" << option << " = "; //<< val;
  else 
    o << "\n"
      << "\n" << description
      << "\n-a_" << option << " [" << val << "]";
}

#endif
