#include "funcgen1d.h"
#include "global.h"
#include <assert.h>
#include <string.h>

InterpolLinFun1D::InterpolLinFun1D()
{
  xlist.clear();
  ylist.clear();
  lastid = 0;
}

InterpolLinFun1D::~InterpolLinFun1D()
{;}

bool InterpolLinFun1D::load(const std::string& fname)
{
  std::ifstream inp(fname.c_str()); 
  if (!inp) {
    EOUT("Can not load file: " << fname);
    return false;
  }
  xlist.clear();
  ylist.clear();
  double x, y;
  do {
    inp >> x >> y;
    if (inp) {
      xlist.push_back(x);
      ylist.push_back(y);
    }
  } while (inp);
  inp.close();
  return true;
}

double InterpolLinFun1D::get( double x )
{
  assert(xlist.size() > 0 && ylist.size()==xlist.size());
  int    id  = lastid;
  double res = 0;
  while (1) {
    if (xlist[id] > x) {
      if (id == 0) { res = ylist[id]; break;} // left of first entry. constant
      id=0; 
      continue;
    }
    if (xlist[id] < x) {
      if (id == (int)xlist.size()-1) { res = ylist[id]; break;} // right of las entry, constant
      if (xlist[id+1] > x) {
	res = ( ( x - xlist[id] ) / ( xlist[id+1] - xlist[id] ) ) * (ylist[id+1] - ylist[id]) + ylist[id]; // linear interpolation
	break;
      }
      // xlist[id+1] <= x
      id+=1; 
      continue;
    }
    if (xlist[id] == x) {
      if (id == (int)xlist.size()-1) { res = ylist[id]; break;} // last entry
      if (xlist[id+1] == x) {id+=1; continue;} // for multiple occurences of x entries
      res = ylist[id];
      break;
    } 
  }
  lastid = id;
  return res;
}


RefGUIClient::RefGUIClient() {
  client = new TCPSocket("127.0.0.1" , 7555);
}

bool RefGUIClient::request_gui(std::string cmd)
{
  char buf[1024];
  int r = client->recv( buf , 1024 );
  buf[r]='\0';
  IOUT("Received: [" << buf << "]");
  cmd+="\n";
  //sprintf(buf,"SLIDER Aktion -0.4 0.4 0 100\n");
  client->send(cmd.c_str() , strlen(cmd.c_str()));

  r = client->recv( buf , 1024 );
  buf[r]='\0';
  if (strcmp(buf , "OK\n") != 0) {
      EOUT("Can not create gui client: " << buf);
      return false;
  }

  return true;
}

RefGUIClient::~RefGUIClient()
{
  delete client;
}

double RefGUIClient::get(double)
{
  double res = 0.0;
   char buf[1024];
   sprintf(buf,"GET\n");
   client->send(buf , strlen(buf));
   int r = client->recv( buf , 1024 );
   buf[r]='\0';

   sscanf(buf,"%lf\n" , &res);

   //IOUT("Received: [" << buf << "]");
   return res;
}
