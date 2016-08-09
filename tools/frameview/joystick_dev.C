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
#include "joystick_dev.h"
#include "ascii_processor.h"

#include <sys/ioctl.h>
//#include <sys/time.h>
//#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
//#include <stdio.h>
//#include <errno.h>
//#include <string.h>

#ifndef __APPLE__
#include <linux/joystick.h>
#endif

JoystickDevice::JoystickDevice() {
  num_axes= 0;
  num_buttons= 0;
}

void JoystickDevice::generic_description_of_options(ostream & o, int mode) const {
  print_option_entry(o,mode,"dev",options.joydev,"defines the joystick device");
}

bool JoystickDevice::process_options(const ValueParser & vp) {
  vp.get("dev", options.joydev,MAX_NAME_LEN);

  if ( vp.num_of_not_accessed_entries() ) {
    ERROR_OUT << "\nJoystickDevice: not recognized options:";
    vp.show_not_accessed_entries(ERROR_STREAM); 
    return false;
  }

  return true;
}

bool JoystickDevice::process_options(const char * fname) {
  ValueParser vp(fname,"JoystickDevice");
  return process_options(vp);
}

bool JoystickDevice::process_options(int argc, char const* const* argv) {
  ValueParser vp(argc,argv,"j_");
  return process_options(vp);
}

void JoystickDevice::help_options(ostream & o) const {
  o << "\njoystick device options:"
    << "\n";
  generic_description_of_options(o,1);
}

void JoystickDevice::generate_file_options(ostream & o) const {
  o << "\n[JoystickDevice]"
    << "\n"
    << "\n### all options can be used on the command line if prefixed with '-j_'"
    << "\n### for example '-j_dev' will specify the joystick device"
    //    << "\n"
    //    << "\n#color options: all colors are define by their rrggbb values"
    << "\n";
  generic_description_of_options(o,0);
}

bool JoystickDevice::init_frames(BuilderBase * build) {
#ifdef __APPLE__
  ERROR_OUT << "at the moment no joystick available on apple" << endl;
  return false;
#else
  if ((joystick_fd = open(options.joydev, O_RDONLY)) < 0) {
    ERROR_OUT << "could't open joystic device " << options.joydev;// perror("joystic device");
    exit(1); //debug
    return false;
  }
  fcntl(joystick_fd, F_SETFL, O_NONBLOCK);

  unsigned char num;
  ioctl(joystick_fd, JSIOCGVERSION, &version);
  ioctl(joystick_fd, JSIOCGAXES, &num);
  num_axes= num;
  ioctl(joystick_fd, JSIOCGBUTTONS, &num);
  num_buttons= num;
  ioctl(joystick_fd, JSIOCGNAME(MAX_NAME_LEN), name);

  cout << "\nJoystick    = " << name
       << "\nversion     = " << version
       << "\nnum_axes    = " << num_axes
       << "\nnum_buttons = " << num_buttons
       << flush;

  button= new int[num_buttons];
  button_chg= new bool[num_buttons];
  axis= new int[num_axes];
  axis_chg= new bool[num_axes];

  return AsciiProcessor::scan_and_parse(build,"\
VA (0,3,12,8);\
EMP 0;\
\
INS FRAME id= 10 (-2,0);\
INS 10 CIRCLE (0,0,1);\
INS 10 POLYGON (-1,-1) ( 1,-1) (1,1) (-1,1);\
INS 10 LINE (-0.1,0,0.1,0) (0,-0.1,0,0.1) (-1.0,0,-0.9,0) (1.0,0,0.9,0) (0,1.0,0,0.9) (0,-1.0,0,-0.9);\
INS 10 FRAME id= 20 lay= 1;\
INS 20 LINE col=ff0000 (-0.2,0,0.2,0) (0,-0.2,0,0.2); \
\
INS FRAME id= 11 ( 2,0);\
INS 11 CIRCLE (0,0,1);\
INS 11 POLYGON (-1,-1) ( 1,-1) (1,1) (-1,1);\
INS 11 LINE (-0.1,0,0.1,0) (0,-0.1,0,0.1) (-1.0,0,-0.9,0) (1.0,0,0.9,0) (0,1.0,0,0.9) (0,-1.0,0,-0.9);\
INS 11 FRAME id= 21 lay=1;\
INS 21 LINE col=ff0000 (-0.2,0,0.2,0) (0,-0.2,0,0.2); \
\
INS FRAME id= 40 (4,2);\
INS 40 CIRCLE (0,0,0.25);\
INS 40 FRAME id= 60;\
INS 60 CIRCLE fil=1 col=0000ff (0,0,0.25);\
\
INS FRAME id= 41 (3,3);\
INS 41 CIRCLE (0,0,0.25);\
INS 41 FRAME id= 61;\
INS 61 CIRCLE fil=1 col=0000ff (0,0,0.25);\
\
INS FRAME id= 42 (5,3);\
INS 42 CIRCLE (0,0,0.25);\
INS 42 FRAME id= 62;\
INS 62 CIRCLE fil=1 col=0000ff (0,0,0.25);\
\
INS FRAME id= 43 (4,4);\
INS 43 CIRCLE (0,0,0.25);\
INS 43 FRAME id= 63;\
INS 63 CIRCLE fil=1 col=0000ff (0,0,0.25);\
\
INS FRAME id= 44 (-2,5);\
INS 44 POLYGON  (0,0) (-2,0) (-2,0.5) (0,0.5);\
INS 44 FRAME id= 64;\
INS 64 POLYGON  fil=1 col=0000ff (0,0) (-2,0) (-2,0.5) (0,0.5);\
\
INS FRAME id= 45 (-2,6);\
INS 45 POLYGON  (0,0) (-1.5,0) (-1.5,0.5) (0,0.5);\
INS 45 FRAME id= 65;\
INS 65 POLYGON  fil=1 col=0000ff (0,0) (-1.5,0) (-1.5,0.5) (0,0.5);\
\
INS FRAME id= 46 (2,5);\
INS 46 POLYGON  (0,0) (2,0) (2,0.5) (0,0.5);\
INS 46 FRAME id= 66;\
INS 66 POLYGON  fil=1 col=0000ff (0,0) (2,0) (2,0.5) (0,0.5);\
\
INS FRAME id= 47 (2,6);\
INS 47 POLYGON  (0,0) (1.5,0) (1.5,0.5) (0,0.5);\
INS 47 FRAME id= 67;\
INS 67 POLYGON  fil=1 col=0000ff (0,0) (1.5,0) (1.5,0.5) (0,0.5);\
\
INS FRAME id= 48 (-4,6);\
INS 48 POLYGON  (0,0) (-1,0) (-1,1) (0,1);\
INS 48 FRAME id= 68;\
INS 68 POLYGON  fil=1 col=0000ff (0,0) (-1,0) (-1,1) (0,1);\
\
INS FRAME id= 49 (4,6);\
INS 49 POLYGON  (0,0) (1,0) (1,1) (0,1);\
INS 49 FRAME id= 69;\
INS 69 POLYGON  fil=1 col=0000ff (0,0) (1,0) (1,1) (0,1);\
\
INS 20 FRAME id=70;\
INS 70 CIRCLE fil=1 col=0000ff (0,0,0.25);\
\
INS 21 FRAME id=71;\
INS 71 CIRCLE fil=1 col=0000ff (0,0,0.25);\
");
#endif
}

bool JoystickDevice::init_connection() {
  //connection to the device was already done in init_frames
  return true;
}

bool JoystickDevice::destruct() {
  close(joystick_fd);
  return true; 
}

bool JoystickDevice::process_input(fd_set *set, BuilderBase * build) {
#ifndef __APPLE__
  //EnDeCoderBin enc;
  if (is_fd_in_set(joystick_fd,set)) {
    //cout << "+" << flush;
    struct js_event js;
    bool redraw= false;

    for (int i=0; i<num_buttons; i++)
      button_chg[i]= false;
    for (int i=0; i<num_axes; i++)
      axis_chg[i]= false;

    while(1) { 
      int size= read(joystick_fd, &js, sizeof(struct js_event));
      if (size != sizeof(struct js_event)) 
	break;

      switch(js.type & ~JS_EVENT_INIT) {
      case JS_EVENT_BUTTON:
	button[js.number]= js.value;
	button_chg[js.number]= true;
	break;
      case JS_EVENT_AXIS:
	axis[js.number] = js.value;
	axis_chg[js.number]= true;
	break;
      }
    }

    for (int i=0; i<num_axes; i+=2) 
      if ( axis_chg[i] || axis_chg[i+1] ) {
	redraw= true;
	//cout << "changing axis " << i << " to " << axis[i] << " " << axis[i+1] << flush;
	if (i <= 2)
	  build->set_cmd_set_frame_pos( a_frame(i), Point2d( double(axis[i])/32767.0,-double(axis[i+1])/32767.0));
	else
	  cout << "\njust 4 axis are supported now" << flush;
      }

    for (int i = 0; i < num_buttons; i++) 
      if (button_chg[i]) {
	redraw= true;
	build->set_cmd_set_frame_visible(b_frame(i), button[i]);
	//cout << "changing button " << i << " to " << button[i] << flush;
      }
    
    if (!redraw)
      cout << "strange: nothing to redraw" << flush;
    //cout << "\nend of event processing" << flush;

    return redraw;
  }

  cout << "\nstrange: joystick_fd wasn't triggert" << flush;
#endif
  return false;
}

#if 1
int  JoystickDevice::set_fds(fd_set * set) { 
  add_fd_to_set(joystick_fd,set); 
  return joystick_fd;
}

bool JoystickDevice::got_fds(fd_set * set) { 
  return is_fd_in_set(joystick_fd,set);
}
#endif
