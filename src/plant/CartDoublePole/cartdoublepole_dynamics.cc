/*
clsquare - closed loop simulation system
Copyright (c) 2004, Neuroinformatics Group, Prof. Dr. Martin Riedmiller,
University of Osnabrueck


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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <math.h>
#include <stdio.h>

#define THETA1  0   /* angle */
#define THETA1P 1   /* angular velocity */
#define THETA2  2   /* angle */
#define THETA2P 3   /* angular velocity */
#define S       4   /* distance */
#define SP      5   /* cart velocity */
#define G 9.81

double determinante( double a1, double b1, double c1,
				    double a2, double b2, double c2,
				    double a3, double b3, double c3){
  return(a1*(b2*c3-b3*c2)-a2*(b1*c3-b3*c1)+a3*(b1*c2-b2*c1));
}


void compute_LGS(double *spp, double *theta2pp, double *theta3pp, 
				 double mc, double m1, double m2, double half_l1, double half_l2, double f, double *state)



/*  *spp, *theta2pp, *theta3pp;           output */
  /*   mc, m1, m2, half_l1, half_l2, f, *state;   input  */{
  double Det, Det1, Det2, Det3,
    a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3;

  /* Solve LGS: A * theta = D  (where A corresponds to 3x3 matrix 'D' in Bogdanov, eq.2, 
     and D =  Hu - C * thetap  + G in Bogdanov)

     where

     (a1 b1 c1)     (  spp   )       (d1)
     (a2 b2 c2)  *  (theta2pp)   =   (d2)
     (a3 b3 c3)     (theta3pp)       (d3)

     use Cramer's Rule to solve for theta = (spp, theta1pp, theta2pp);
  */

  /* Gleichung 27 :  a1*spp + b1*theta2pp + c1*theta3pp = d1     */
  a1 = mc+m1+m2;
  b1 = half_l1*(double)cos((double)state[THETA1])*(m1+2.0*m2);
  c1 = m2*half_l2*(double)cos((double)state[THETA2]);
  d1 = f +(state[THETA1P]*state[THETA1P]*half_l1*(double)sin((double)state[THETA1])*(m1+2.0*m2))
         +(m2*half_l2*state[THETA2P]*state[THETA2P]*(double)sin((double)state[THETA2]));
  
  /* Gleichung 32 :  a2*spp + b2*theta2pp + c2*theta3pp = d2     */
  a2 = (m1+2.0*m2)*half_l1*(double)cos((double)state[THETA1]);
  b2 = half_l1*half_l1*(4.0/3.0*m1+4.0*m2);
  c2 = m2*2.0*half_l1*half_l2*(double)cos((double)(state[THETA1]-state[THETA2]));
  d2 = -(state[THETA2P]*state[THETA2P]*m2*2.0*half_l1*half_l2*(double)sin((double)(state[THETA1]-state[THETA2])))
       +(G*half_l1*(double)sin((double)state[THETA1])*(m1+2.0*m2));
  
  /* Gleichung 37 :  a3*spp + b3*theta2pp + c3*theta3pp = d3     */
  a3 = m2*half_l2*(double)cos((double)state[THETA2]);
  b3 = 2.0*half_l1*half_l2*m2*(double)cos((double)(state[THETA1]-state[THETA2]));
  c3 = m2*half_l2*half_l2*4.0/3.0;
  d3 = +(state[THETA1P]*state[THETA1P]*2.0*half_l1*half_l2*m2*(double)sin((double)(state[THETA1]-state[THETA2])))
       +(m2*G*half_l2*(double)sin((double)state[THETA2]));

  Det = determinante(a1, b1, c1,
                     a2, b2, c2,
                     a3, b3, c3);

  Det1= determinante(d1, b1, c1,
                     d2, b2, c2,
                     d3, b3, c3);     /* ACHTUNG: man kann die Berechnung noch geschickter machen, wenn man identische Ausdruecke nur einmal berechnet */
  
  Det2= determinante(a1, d1, c1,
                     a2, d2, c2,
                     a3, d3, c3);
  
  Det3= determinante(a1, b1, d1,
                     a2, b2, d2,
                     a3, b3, d3);
  
  *spp      = Det1 / Det;
  *theta2pp = Det2 / Det;
  *theta3pp = Det3 / Det;
}


