#include "Joystick.h"
#include <fcntl.h>
#include <linux/joystick.h>
#include <iostream>
#include <stdio.h>
#include "unistd.h"

using namespace std;

namespace {
  const double MAX_AXIS_VALUE = 32768;   // Groesste Auslenkung der Achsen
}

Joystick::Joystick (const char* devname) throw (std::invalid_argument, std::bad_alloc) : axis (6), button (12) {
  file_descriptor = open(devname, O_RDONLY);
  if (file_descriptor<0)
    throw std::invalid_argument ("invalid device name in Joystick::Joystick");
  
  fcntl(file_descriptor, F_SETFL, O_NONBLOCK); //don't block
  unsigned char n;
  ioctl(file_descriptor, JSIOCGAXES, &n);  // Anzahl Achsen abfragen
  axis.resize (n);
  ioctl(file_descriptor, JSIOCGBUTTONS, &n);  // Anzahl Knoepfe abfragen
  button.resize (n);
}

Joystick::~Joystick () throw () {
  close (file_descriptor);
}

void Joystick::update () throw () {
  js_event ev;
  int evsize = sizeof (struct js_event);
  while (true) {
    int size = read (file_descriptor, &ev, evsize);
    if (size!=evsize)
      break;
    
    if (ev.type==JS_EVENT_BUTTON)
      button[ev.number]=(ev.value==1);
    else if (ev.type==JS_EVENT_AXIS)
      axis[ev.number]=static_cast<double>(ev.value)/MAX_AXIS_VALUE;
  }
}

const std::vector<double>& Joystick::get_axis_state () throw () {
  update ();
  return axis;
}

const std::vector<bool>& Joystick::get_button_state () throw () {
  update ();
  return button;
}

std::string Joystick::get_type () throw (std::bad_alloc) {
  char buffer [500];
  ioctl(file_descriptor, JSIOCGNAME(500), buffer);
  return std::string (buffer);
}

int Joystick::get_version () throw () {
  int n;
  ioctl(file_descriptor, JSIOCGVERSION, &n);
  return n;
}

Rumble::Rumble(const char* dev_str)
{
  fd = -1;
  fd = open(dev_str, O_RDWR);
  if (fd == -1) {
   perror("Open device file");
   return;
  }
  
  /* a weak rumbling effect */
  effects[0].type = FF_RUMBLE;
  effects[0].id = -1;
  effects[0].u.rumble.strong_magnitude = 0x1000;
  effects[0].u.rumble.weak_magnitude = 0xc000;
  effects[0].replay.length = 300;
  effects[0].replay.delay = 0;

  if (ioctl(fd, EVIOCSFF, &effects[0]) == -1) {
	perror("Upload effects[0]");
	close(fd);
        fd = -1;
  }
}

Rumble::~Rumble()
{
  if (fd >= 0) close(fd);
}

void Rumble::play(int i)
{
  struct input_event p;

  if (fd < 0) return;
  if (i<0 || i>=N_EFFECTS) return;

  p.type = EV_FF;
  p.code = effects[i].id;
  p.value = 1;

  if (write(fd, (const void*) &p, sizeof(p)) == -1) {
   perror("Play effect");
  }
}

void Rumble::stop(int i)
{
  struct input_event s;

  if (fd < 0) return;
  if (i<0 || i>=N_EFFECTS) return;

  s.type = EV_FF;
  s.code = effects[i].id;
  s.value = 0;

  if (write(fd, (const void*) &s, sizeof(s)) == -1) {
   perror("Stop effect");
  }
}

