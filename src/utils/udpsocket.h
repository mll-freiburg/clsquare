/*
 * Copyright (c) 1999 - 2001, Artur Merke <artur.merke@udo.edu> 
 *

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

#ifndef _UDPSOCKET_H_
#define _UDPSOCKET_H_


#include <netinet/in.h>

class UDPsocket {
 public:
  static void set_fd_nonblock(int fd);
  static void set_fd_sigio(int fd);
 public:
  void set_fd_nonblock() { set_fd_nonblock(socket_fd); }
  void set_fd_sigio() { set_fd_sigio(socket_fd); }

  static const int MAXMESG= 4096;

  int			socket_fd;
  struct sockaddr_in	serv_addr;
  UDPsocket() { socket_fd= -1; }
  bool init_socket_fd(int port= 0);
  bool init_serv_addr(const char* host, int port);
  bool send_msg(const char* buf, int len);
  bool recv_msg(char *buf, int & len, bool redirect= false);
  bool recv_msg(char *buf, int & len, int max_len, bool redirect);
  void close_socket_fd();
};

#endif
