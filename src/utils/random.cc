#define MY_RAND_MAX 32767

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <stdio.h>
#include "random.h"


int Util::seed=1;

void Util::set_seed(const int seed){
    std::cout<<"NEW setting seed "<< seed <<std::endl;
    srand(seed);        
}


double my_ran0(int *idum)
{
 
  static double y, maxran, v[98];
  double dum;
  static int iff=0;      
  int j;                          

  if (*idum < 0 || iff == 0) {

    iff = 1;             
    maxran = MY_RAND_MAX+1.0;
    //prinf("%d \n", *idum);
    std::cout<<"setting seed "<< *idum <<std::endl;
    srand(*idum);        
    *idum = 1;           
    for (j=1; j<=97; j++)
      dum = rand()%MY_RAND_MAX;
    for (j=1; j<=97; j++)
      v[j] = rand()%MY_RAND_MAX; /* ensure sun-compatib. */
    y = rand()%MY_RAND_MAX;

  }

  j =(int) (1. + 97.0 * (double)y/(double)maxran);

  if (j > 97 || j < 1)   
    printf("my_ran0: This is nonsense\n");
  y = v[j];              
  v[j] = rand()%MY_RAND_MAX;

  return (y/maxran);     
}


double my_gasdev(int *idum)
{
 
  static int iset=0;
  static double gset;
  double fac, r, v1, v2;

  if (iset == 0) {

    do {
      v1 = 2.0 * my_ran0(idum) - 1.0;
      v2 = 2.0 * my_ran0(idum) - 1.0;
      r = v1*v1 + v2 *v2;
    }
    while (r >= 1.0);   

    fac = sqrt(-2.0 * log(r)/r);
    gset = v1 * fac;
    iset = 1;
    return v2*fac;

  }
  else {

    iset = 0;
    return gset;
  }
}


double Util::gaussian(double value, double std)
{

  static int firsttime = 1;
  double     temp;

  if (std == 0)
    return value;

  /* intitialize the random function with an arbitrary number */

  if (firsttime) {
    firsttime = 0;
    seed = 1; //-abs(((int) time(NULL)) % MY_RAND_MAX);
  }

  /* get me a value from my_gasdev,
     which is normal distributed about 0, var=1 */

  temp = my_gasdev(&seed)*std;

  return (value + temp);

}

