#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <iomanip>


using namespace std;

int main(int argc, char **argv){

  if(argc <4){
    cout<<"call by randomgen <dim> <number> <min> <max>"<<endl;
    exit(0);
  }
  srand48(16514);
  for(int i=0;i<atoi(argv[2]);i++){
    for(int j=0;j<atoi(argv[1]);j++){
      if(j==0)
	cout<<drand48()*(atof(argv[4])-atof(argv[3]))+atof(argv[3])<<" ";
      else
	cout<<"0.0";
    }
    cout<<endl;
  }
  cout<<endl;
}
