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
#include "ascii_dev.h"
#include "ascii_processor.h"
#include "global_defs.h"

AsciiDevice::AsciiDevice(int port) {
  use_tcp= true;
  tcp_serv_can_accept= true;
  tcp_sock_open= false;
  state_string[0]= '\0';
}

void AsciiDevice::generic_description_of_options(ostream & o, int mode) const {
  print_option_entry(o,mode,"udp_port",options.udp_port,"defines the udp socket port for communication");
  print_option_entry(o,mode,"tcp_port",options.tcp_port,"defines the tcp socket port for communication");
}

bool AsciiDevice::process_options(const ValueParser & vp) {
  vp.get("udp_port", options.udp_port);
  vp.get("tcp_port", options.tcp_port);

  if ( vp.num_of_not_accessed_entries() ) {
    ERROR_OUT << "\nAsciiDevice: not recognized options:";
    vp.show_not_accessed_entries(ERROR_STREAM); 
    return false;
  }

  return true;
}

bool AsciiDevice::process_options(const char * fname) {
  ValueParser vp(fname,"AsciiDevice");
  return process_options(vp);
}

bool AsciiDevice::process_options(int argc, char const* const* argv) {
  ValueParser vp(argc,argv,"a_");
  return process_options(vp);
}

void AsciiDevice::help_options(ostream & o) const {
  o << "\nascii device options:"
    << "\n";
  generic_description_of_options(o,1);
}

void AsciiDevice::generate_file_options(ostream & o) const {
  o << "\n[AsciiDevice]"
    << "\n"
    << "\n### all options can be used on the command line if prefixed with '-a_'"
    << "\n### for example '-a_udp_port' will specify the port for udp communication"
    //    << "\n"
    //    << "\n#color options: all colors are define by their rrggbb values"
    << "\n";
  generic_description_of_options(o,0);
}

bool AsciiDevice::init_frames(BuilderBase * build) {
  return AsciiProcessor::scan_and_parse(build,"\
VA (7.5,1,17,4);\
INS FRAME id=1; \
INS 1 LINE col=ff0000 (0,0,1,0) (0,0,0,2) (0,2,1,2);\
INS FRAME id= 2 (2,0,0);\
INS 2 LINE col=00ff00 (0,0,1,0) (0,0,0,2);\
INS FRAME id= 3 (4,0,0);\
INS 3 LINE col=0000ff (0,0,1,0) (1,0,1,1) (1,1,0,1) (0,1,0,2) (0,2,1,2);\
INS FRAME id= 4 (6,0,0);\
INS 4 LINE col=ffcc00 (0,0,1,0) (0,0,0,2) (0,2,1,2) (1,0,1,2) (0.75,0.25,1.25,-0.25);\
INS FRAME id= 5 (8,0,0);\
INS 5 LINE col=00ccff (0,0,1,0) (0,0,0,2) (1,0,1,2);\
INS FRAME id= 6 (10,0,0);\
INS 6 LINE col=ff00cc (0,0,0,2) (0,2,1,2) (1,2,1,0) (0,1,1,1);\
INS FRAME id= 7 (12,0,0);\
INS 7 LINE col=ccff00 (0,0,0,2) (0,2,1,2) (1,2,1,1) (0,1,1,1) (0.25,1,1,0);\
INS FRAME id= 8 (14,0,0);\
INS 8 LINE col=cc00ff (0,0,1,0) (0,0,0,2) (0,1,1,1) (0,2,1,2);\
");

  return AsciiProcessor::scan_and_parse(build,"\
VA (0,1.5,4,4);\
INS FRAME id=1; \
INS 1 LINE (0,0,0,1);\
INS 1 FRAME id= 2 (0,1,0.78);\
INS 2 LINE col=ff0000 (0,0,0,1);\
INS 1 FRAME id= 3 (0,1);\
INS 3 LINE col=00ff00 (0,0,0,1);\
INS 1 FRAME id= 4 (0,1,-0.78);\
INS 4 LINE col=0000ff (0,0,0,1);\
INS 2 FRAME id= 5 (0,1,0.52);\
INS 5 LINE col=ffcc00 (0,0,0,1);\
INS 2 FRAME id= 6 (0,1,-0.52);\
INS 6 LINE col=ff00cc (0,0,0,1);\
INS 3 FRAME id= 7 (0,1,0.52);\
INS 7 LINE col=ccff00 (0,0,0,1);\
INS 3 FRAME id= 8 (0,1,-0.52);\
INS 8 LINE col=00ffcc (0,0,0,1);\
INS 4 FRAME id= 9 (0,1,0.52);\
INS 9 LINE col=cc00ff (0,0,0,1);\
INS 4 FRAME id= 10 (0,1,-0.52);\
INS 10 LINE col=00ccff (0,0,0,1);");
#ifdef BLABLA
  //both versions produce the same code!
#if 1
  return AsciiProcessor::scan_and_parse(build,"\
INS FRAME id= 1; \
INS 1 POLYGON fil=1 col= cc3333 (-10,10) (10,10) (10,-10) (-10,-10);\
INS 1 POLYLINE col= 000000 lay= 1 (0,0) (20,0) (30,30);");
#else
  RGBcolor c_car(204,51,51);
  RGBcolor c_pole(0,0,0);

  const int num_points= 10;
  
  Multi<Point2d> points;
  points.set_max_size(num_points);

  int cur_frame= 1;
  build->set_cmd_insert_frame(0,cur_frame,Point2d(0.0,0.0),0.0,1);

  //field
  int k= 0;
  points.tab[k++]= Point2d(-10, 10);
  points.tab[k++]= Point2d( 10, 10);
  points.tab[k++]= Point2d( 10, -10);
  points.tab[k++]= Point2d(-10, -10);
  points.set_cur_size(k);
  build->set_cmd_insert_f_polygon(cur_frame,0,points,0, c_car);

  k= 0;
  points.tab[k++]= Point2d(0, 0);
  points.tab[k++]= Point2d(20,0);
  points.tab[k++]= Point2d(30,30);

  points.set_cur_size(k);
  build->set_cmd_insert_polyline(cur_frame,0,points,1, c_pole);

  return true;
#endif
#endif
}

bool AsciiDevice::init_connection() {
  if (use_udp) {
    udp_sock.init_socket_fd(options.udp_port);
    udp_sock.set_fd_nonblock();
    //udp_sock.init_serv_addr("host",port); // nur als Empfaenger von Nachrichten konzipiert
  }

  if (use_tcp) {
    if ( !tcp_serv.init(options.tcp_port) ) {
      use_tcp= false;
      ERROR_STREAM << "\ntcp stopped";
      return false;
    }
    if ( !tcp_serv.listen() ) {
      use_tcp= false;
      return false;
    }
    tcp_serv_can_accept= true;
    tcp_sock_open= false;
  }
  return true;
}

bool AsciiDevice::destruct() {
  if (use_tcp) {
    if (tcp_sock_open) {
      //cout << "\nclosing tcp_sock";
      tcp_sock.close();
    }
    //cout << "\nclosing tcp_serv" << flush;
    tcp_serv.close();
  }
  return true; 
}
bool AsciiDevice::process_input(fd_set *set, BuilderBase * build) {
  //EnDeCoderBin enc;
  static char  buffer[2][BUFFER_MAX_SIZE];
  int buffer_idx= 0;
  char *dum= buffer[buffer_idx];
  int num_msg= 0;
  int num_bytes= 0;
  bool got_input= false;

  if (use_udp && is_fd_in_set(udp_sock.socket_fd, set) ) {
    //cout << "\n----------------------------------------";
    // int num_msg_squared= 0;
    while ( udp_sock.recv_msg( dum, num_bytes, BUFFER_MAX_SIZE-1, true) ) {
      //bool processed= false;
      dum[num_bytes]= '\0';
      
      while ( *dum== ' ')
	dum++;
      if ( *dum=='[' ) {
	while ( *dum != '\0' && *dum != ']' )
	  dum++;

	if ( * dum == ']' ) {
	  dum++;
	  AsciiProcessor::scan_and_parse(build,dum);
	  //cout << "\n msg num= " << num_msg_squared++ << ", bytes=" << num_bytes << flush;
	  got_input= true;
	}
	else {
	  cerr << "ascii_dev: no valid protocol option, skipping message";
	}
	continue;
      }

      buffer_idx= (buffer_idx+1)%2;
      dum= buffer[buffer_idx];
      num_msg+= 1;
	//cout << "\nnum_bytes= " << num_bytes;
      
    }

    buffer_idx= (buffer_idx+1)%2;
    dum= buffer[buffer_idx];

    if ( num_msg ) {
      if (num_msg > 1){
	//cout << "\nSKIPPED " << num_msg-1 << " MESSAGE(S)" << flush;
      }
      //cout << "\nbuf: " << dum << flush;
      AsciiProcessor::scan_and_parse(build,dum);
      got_input= true;
    }
  }

  if (use_tcp) {
    //cout << "\nentering tcp input" << flush;
    bool redraw= process_tcp_input(set, build);
    //cout << "\ngot tcp input";
    got_input = got_input || redraw;
    //cout << "\nleaving tcp input" << flush;
  }

  if ( use_stdin && is_fd_in_set(0, set) ) {
    //cout << "\n[got input]";
    cin.getline(buffer[0],BUFFER_MAX_SIZE);
    AsciiProcessor::scan_and_parse(build,buffer[0]);
    got_input= true; 
  }

  return got_input;
}

bool AsciiDevice::process_tcp_input(fd_set *set, BuilderBase * build) {
  static char  buffer[2*BUFFER_MAX_SIZE];
  static int neg_offset= 0;
  bool redraw= false;

  if ( is_fd_in_set(tcp_serv.fd,set) ) {
    //cout << "\nserver message" << flush;
    if ( !tcp_serv_can_accept )
      WARNING_OUT << "\njust one tcp connection is allowed (todo)";
    else {
      if ( tcp_serv.accept( tcp_sock ) ) {
	//cout << "\nsocket accepted" << flush;
	TCPutils::set_fd_nonblock(tcp_sock.fd);
	tcp_sock_open= true;
	tcp_serv_can_accept= false;
	neg_offset= 0;
      }
      else {
	WARNING_OUT << "\nstrange server message" << flush;
      }
      //return false; //no redraw
    }
  }
  
  if ( tcp_sock_open && is_fd_in_set(tcp_sock.fd,set) ) {
    int msg_len;
    char * dum= buffer+BUFFER_MAX_SIZE;
    while ( (msg_len = tcp_sock.recv_msg( dum, BUFFER_MAX_SIZE-1)) > 0 ) {
      dum[msg_len]= '\0';
      char * end= dum + msg_len;
      char * beg= dum - neg_offset;

#if 0
      cout << "\n--------";
      cout << "\nTCP msg, bytes=" << msg_len 
	   << "\ndum= [" << dum << "]"
	   << "\nneg_offset= " << neg_offset
	   << "\nbeg= [" << beg << "]";
#endif
      
      char * end_command= end;
      while ( end_command > dum && *end_command != ';')
	end_command--;

      if (*end_command != ';') {
	neg_offset= 0;
	ERROR_OUT << "\ncommand longer then the available buffer";
	continue;
      }
      end_command++;
      char tmp_char= *end_command;
      *end_command= '\0';
#if 0
      cout << "\ntruncating" 
	   << "\nbeg= [" << beg << "]";
#endif
      bool res= AsciiProcessor::scan_and_parse(build,beg);
      if (res)
	redraw= true;

      *end_command= tmp_char;

      neg_offset= end-end_command;
      if (neg_offset>0)
	memcpy( dum- neg_offset, end_command, neg_offset);
    }

    //cout << "\nmsg_len= " << msg_len << flush;
    if ( msg_len == 0 ) {
      tcp_sock.close();
      tcp_sock_open= false;
      tcp_serv_can_accept= true;
      //cout << "\nmsg_len= " << msg_len << " -> closing connection" << flush;
    }
#if 0
    if ( msg_len < 0) {
      cout << "\nno more date available at the moment" << flush;
    }
#endif
  }
  return redraw;
}

#if 1
int  AsciiDevice::set_fds(fd_set * set) { 
  int max= 0;
  if ( use_udp ) {
    add_fd_to_set(udp_sock.socket_fd,set); 
    max= udp_sock.socket_fd;
  }
  if ( use_tcp ) {
    add_fd_to_set(tcp_serv.fd,set);
    if (tcp_serv.fd > max)
      max= tcp_serv.fd;

    if ( tcp_sock_open ) {
      add_fd_to_set(tcp_sock.fd, set);
      if (tcp_sock.fd > max)
	max= tcp_sock.fd;
    }
  }
  if ( use_stdin )
    add_fd_to_set(0,set); //standard input
  return max;
}

bool AsciiDevice::got_fds(fd_set * set) { 
  if (use_udp)
    if ( is_fd_in_set(udp_sock.socket_fd,set) ) return true;

  if (use_tcp) {
    if ( is_fd_in_set(tcp_serv.fd,set) )
      return true;

    if ( tcp_sock_open && is_fd_in_set(tcp_sock.fd,set) )
      return true;
  }

  if (use_stdin)
    return is_fd_in_set(0,set);
  return false;
}
#endif
