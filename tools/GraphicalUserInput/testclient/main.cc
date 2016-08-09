#include "Socket.h"
#include <string.h>
#include <stdio.h>
#include "unistd.h"

#include <iostream>

#define EOUT(__x__) std::cerr << "#ERROR[ " << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n";
#define IOUT(__x__) std::cerr << "#INFO [ " << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n";

int main() {
  TCPSocket client("127.0.0.1" , 7555);

  sleep(1);

  char buf[1024];

  client.recv( buf , 1024 );

  IOUT("Received: [" << buf << "]");

  sprintf(buf,"SLIDER Aktion -0.4 0.4 0 100\n");
  client.send(buf , strlen(buf));

  while(1) {
    int r;
    usleep(10000);
    sprintf(buf,"GET\n");
    client.send(buf , strlen(buf));
    r = client.recv( buf , 1024 );
    buf[r]='\0';
    IOUT("Received: [" << buf << "]");
  }
}
