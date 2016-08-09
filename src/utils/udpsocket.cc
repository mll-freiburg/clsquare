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


/* author: Artur Merke */
#include "udpsocket.h"

#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>

#ifndef ERROR_STREAM
#define ERROR_STREAM std::cerr
#endif

#ifndef ERROR_OUT
#define ERROR_OUT ERROR_STREAM << "\n\n*** ERROR file=\"" << __FILE__ << "\" line=" << __LINE__
#endif

#ifndef WARNING_STREAM
#define WARNING_STREAM std::cerr
#endif

#ifndef WARNING_OUT
#define WARNING_OUT WARNING_STREAM << "\n\n*** WARNING file=\"" << __FILE__ << "\" line=" << __LINE__
#endif

#ifdef MEMSET
#define bzero(a, b) memset(a, 0, b)
#endif

#if 0
bool UDPsocket::init_socket_fd(int port) {
  struct sockaddr_in cli_addr;
  int fd;
  if( (fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    return false ;       /* Can't open socket. */
  }

  bzero((char *) &cli_addr, sizeof(cli_addr)) ;
  cli_addr.sin_family           = AF_INET ;
  cli_addr.sin_addr.s_addr      = htonl(INADDR_ANY) ;
  cli_addr.sin_port             = htons(port) ;
   
  if (bind(fd, (struct sockaddr *) &cli_addr, sizeof(cli_addr)) < 0){
    return false ;  /* Can't bind local address */
  }
  //set_fd_nonblock(fd);
  socket_fd= fd;
  return true;
}
#endif 

bool UDPsocket::init_socket_fd(int port) {
  struct sockaddr_in cli_addr;
  int fd;
  if( (fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    ERROR_OUT << "\ncan't open socket";
    return false ;       /* Can't open socket. */
  }

  socket_fd= fd;

  if (port <= 0)
    return true;

  bzero((char *) &cli_addr, sizeof(cli_addr)) ;
  cli_addr.sin_family           = AF_INET ;
  cli_addr.sin_addr.s_addr      = htonl(INADDR_ANY) ;
  cli_addr.sin_port             = htons(port) ;
   
  if (bind(fd, (struct sockaddr *) &cli_addr, sizeof(cli_addr)) < 0){
    ERROR_OUT << "can't bind local addres (port= " << port << ")";
    return false ;  /* Can't bind local address */
  }
  //set_fd_nonblock(fd);

  return true;
}

bool UDPsocket::init_serv_addr(const char* host, int port) {
  struct hostent        *host_ent ;
  struct in_addr      *addr_ptr ;
 
  if((host_ent = (struct hostent *)gethostbyname(host)) == NULL) {
    /* Check if a numeric address */
    if((int)inet_addr(host) == -1){
      ERROR_OUT << "\ncould'n initialize server address";
      return false;
    }
  }
  else {
    addr_ptr = (struct in_addr *) *host_ent->h_addr_list ;
    host = inet_ntoa(*addr_ptr) ;
  }

  bzero((char *) &serv_addr, sizeof(serv_addr)) ;
  serv_addr.sin_family		= AF_INET ;
  serv_addr.sin_addr.s_addr	= inet_addr(host) ;
  serv_addr.sin_port		= htons(port) ;
  return true;
}

bool UDPsocket::send_msg(const char *buf, int len) {
  if (!buf) return false;
  //n = strlen(buf) ;
    
  int res = sendto(socket_fd, buf, len, 0, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  if (res != len ) {
    ERROR_OUT << "\nproblems with sending, res= " << res << " len= " << len; 
    return false ;
  }

  return true;
}

bool UDPsocket::recv_msg(char *buf, int & len, bool redirect) {
  return recv_msg(buf,len,MAXMESG,redirect);
}

bool UDPsocket::recv_msg(char *buf, int & len, int max_len, bool redirect = false) {
  unsigned int addr_len ;
  struct sockaddr_in    recv_addr ;
      
  addr_len = (unsigned int) sizeof(recv_addr) ;
  len = recvfrom(socket_fd, buf, max_len, 0,(struct sockaddr *)&recv_addr, &addr_len);
  if(len < 0) {
    if( len == -1 && errno == EWOULDBLOCK){
      //std::cout << "-" << flush;
    }
    len = 0;
    buf[0]= '\0';
    return false ;
  }
  //std::cout << buf << flush;
  buf[len] = '\0' ;

  if ( redirect  
       && serv_addr.sin_addr.s_addr == recv_addr.sin_addr.s_addr //same host
       && serv_addr.sin_port != recv_addr.sin_port ) {           //but another port
    std::cout << "\nNow sender redirected from port " << ntohs(serv_addr.sin_port)
	 << " to port " << ntohs(recv_addr.sin_port) << " on same server";
    serv_addr.sin_port = recv_addr.sin_port ; // Aendert die Sendeadresse, muss noch geaendet werden
    }
  return true;
  
}

void UDPsocket::set_fd_nonblock(int fd) {
/* doc:
O_NONBLOCK 
      The bit that enables nonblocking mode for the file. If this bit is set, read requests on the file can return
      immediately with a failure status if there is no input immediately available, instead of blocking. Likewise, write
      requests can also return immediately with a failure status if the output can't be written immediately. 

O_NDELAY 
      This is a synonym for O_NONBLOCK, provided for compatibility with BSD. 

FASYNC
if you set the FASYNC status flag on a file descriptor (see section File Status Flags), a SIGIO signal is sent whenever input
or output becomes possible on that file descriptor. The process or process group to receive the signal can be selected
by using the F_SETOWN command to the fcntl function. If the file descriptor is a socket, this also selects the recipient of
SIGURG signals that are delivered when out-of-band data arrives on that socket; see section Out-of-Band Data. 
*/

  if ( fcntl( fd , F_SETOWN, getpid()) == -1) {
    ERROR_OUT << "\nfcntl returns -1";
    return;
  }

  //std::cout << "\nfcntl returns != -1";
  int val = fcntl(fd, F_GETFL, 0) ;

#if 1 
  val |= O_NDELAY ;
#else
  val |= O_NONBLOCK ;
#endif
  //val |= FASYNC;
  fcntl(fd, F_SETFL, val) ;
}

void UDPsocket::set_fd_sigio(int fd) {
/* doc:
O_NONBLOCK 
      The bit that enables nonblocking mode for the file. If this bit is set, read requests on the file can return
      immediately with a failure status if there is no input immediately available, instead of blocking. Likewise, write
      requests can also return immediately with a failure status if the output can't be written immediately. 

O_NDELAY 
      This is a synonym for O_NONBLOCK, provided for compatibility with BSD. 

FASYNC
if you set the FASYNC status flag on a file descriptor (see section File Status Flags), a SIGIO signal is sent whenever input
or output becomes possible on that file descriptor. The process or process group to receive the signal can be selected
by using the F_SETOWN command to the fcntl function. If the file descriptor is a socket, this also selects the recipient of
SIGURG signals that are delivered when out-of-band data arrives on that socket; see section Out-of-Band Data. 
*/

  if ( fcntl( fd , F_SETOWN, getpid()) == -1) {
    ERROR_OUT << "\nfcntl returns -1";
    return;
  }

  //std::cout << "\nfcntl returns != -1";
  int val = fcntl(fd, F_GETFL, 0) ;
  
  val |= FASYNC;
  fcntl(fd, F_SETFL, val) ;
}

void UDPsocket::close_socket_fd() {
  close(socket_fd);
}


#if 0
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/***************  T  E  S  T**************************************************/
main() {
  char buf[MAXMESG];
  int len;
  UDPsocket sock;
  std::cout << "\ninit_socket_fd = " << sock.init_socket_fd();
  std::cout << "\ninit_serv_addr = " << sock.init_serv_addr("robocup3",6000);
  std::cout << "\nsend_msg       = " << sock.send_msg("(init BS2k (version 5.25))");
  std::cout << "\nrecv_msg       = " << sock.recv_msg(buf,len);
  std::cout << "\nbuf= " << buf;
  sock.send_msg("(dash 100)");
  sock.close_socket_fd();
}
#endif 

