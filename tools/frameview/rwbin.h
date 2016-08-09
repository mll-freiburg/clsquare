/*
 * Copyright (c) 1999 - 2001, Artur Merke <amerke@ira.uka.de> 
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _RWBIN_H_
#define _RWBIN_H_

#include <iostream>

#ifndef NATIVE_INT_FORMAT
#include <netinet/in.h>
#endif

inline void rbin(std::istream & in, char & val) {
  in.read(&val, sizeof(char));
}

inline void rbin(std::istream & in,unsigned char & val) {
  in.read((char*)&val, sizeof(unsigned char));
}

inline void rbin(std::istream & in, int & val) {
#ifdef NATIVE_INT_FORMAT
  in.read((char*)&val, sizeof( int));
#else
  unsigned long nval;
  in.read((char*)&nval, sizeof(unsigned long));
  val = ( int) ntohl (nval);
#endif
}

inline void rbin(std::istream & in, long & val) {
#ifdef NATIVE_INT_FORMAT
  in.read((char*)&val, sizeof( long));
#else
  unsigned long nval;
  in.read((char*)&nval, sizeof(unsigned long));
  val = ( long) ntohl (nval);
#endif
}

inline void rbin(std::istream & in, short & val) {
#ifdef NATIVE_INT_FORMAT
  in.read((char*)&val, sizeof( short));
#else
  unsigned short nval;
  in.read((char*)&nval, sizeof(unsigned short));
  val = ( short) ntohs (nval);
#endif
}

inline void rbin(std::istream & in,unsigned int & val) {
#ifdef NATIVE_INT_FORMAT
  in.read((char*)&val, sizeof(unsigned int));
#else
  unsigned long nval;
  in.read((char*)&nval, sizeof(unsigned long));
  val = (unsigned int) ntohl (nval);
#endif
}

inline void rbin(std::istream & in,unsigned long & val) {
#ifdef NATIVE_INT_FORMAT
  in.read((char*)&val, sizeof(unsigned long));
#else
  unsigned long nval;
  in.read((char*)&nval, sizeof(unsigned long));
  val = ntohl (nval);
#endif
}

inline void rbin(std::istream & in,unsigned short & val) {
#ifdef NATIVE_INT_FORMAT
  in.read((char*)&val, sizeof(unsigned short));
#else
  unsigned short nval;
  in.read((char*)&nval, sizeof(unsigned short));
  val = ntohs (nval);
#endif
}

inline void rbin(std::istream & in,bool & val) {
#ifdef NATIVE_INT_FORMAT
  in.read((char*)&val, sizeof(bool));
#else
  unsigned char charCast;
  in.read((char*)&charCast, sizeof(unsigned char));
  val = (bool) charCast;
#endif
}

inline void rbin(std::istream & in,float & val) {
  in.read((char*)&val, sizeof(float));
}

inline void rbin(std::istream & in,double & val) {
  in.read((char*)&val, sizeof(double));
}

inline void rbin(std::istream & in,long double & val) {
  in.read((char*)&val, sizeof(long double));
}

/******************************************************************************/

inline void wbin(std::ostream & out,  char val) {
  out.write(&val, sizeof( char));
}

inline void wbin(std::ostream & out, unsigned char val) {
  out.write((const char*) &val, sizeof(unsigned char));
}

inline void wbin(std::ostream & out,  int val) {
#ifdef NATIVE_INT_FORMAT
  out.write((const char*) &val, sizeof( int));
#else
  unsigned long nval = htonl (val);
  out.write((const char*) &nval, sizeof(unsigned long));
#endif
}

inline void wbin(std::ostream & out,  long val) {
#ifdef NATIVE_INT_FORMAT
  out.write((const char*) &val, sizeof( long));
#else
  unsigned long nval = htonl (val);
  out.write((const char*) &nval, sizeof(unsigned long));
#endif
}

inline void wbin(std::ostream & out,  short val) {
#ifdef NATIVE_INT_FORMAT
  out.write((const char*) &val, sizeof( short));
#else
  unsigned short nval = htons (val);
  out.write((const char*) &nval, sizeof(unsigned short));
#endif
}

inline void wbin(std::ostream & out, unsigned int val) {
#ifdef NATIVE_INT_FORMAT
  out.write((const char*) &val, sizeof(unsigned int));
#else
  unsigned long nval = htonl (val);
  out.write((const char*) &nval, sizeof(unsigned long));
#endif
}

inline void wbin(std::ostream & out, unsigned long val) {
#ifdef NATIVE_INT_FORMAT
  out.write((const char*) &val, sizeof(unsigned long));
#else
  unsigned long nval = htonl (val);
  out.write((const char*) &nval, sizeof(unsigned long));
#endif
}

inline void wbin(std::ostream & out, unsigned short val) {
#ifdef NATIVE_INT_FORMAT
  out.write((const char*) &val, sizeof(unsigned short));
#else
  unsigned short nval = htons (val);
  out.write((const char*) &nval, sizeof(unsigned short));
#endif
}

inline void wbin(std::ostream & out, bool val) {
#ifdef NATIVE_INT_FORMAT
  out.write((const char*) &val, sizeof(bool));
#else
  unsigned char charCast = (unsigned char) val;
  out.write((const char*) &charCast, sizeof(unsigned char));
#endif
}

inline void wbin(std::ostream & out, float val) {
  out.write((const char*) &val, sizeof(float));
}

inline void wbin(std::ostream & out, double val) {
  out.write((const char*) &val, sizeof(double));
}

inline void wbin(std::ostream & out, long double val) {
  out.write((const char*) &val, sizeof(long double));
}

#endif 


#if 0
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/*************      T E S T     ***************************************/
#include <fstream.h>

main() {
  char c= 'C',c2;
  double d= 3.14,d2;
  int i= -254,i2;


  ofstream out("xxx.tmp");
  wbin(out,c);
  wbin(out,d);
  wbin(out,i);
  out.close();

  ifstream in("xxx.tmp");
  rbin(in,c2);
  rbin(in,d2);
  rbin(in,i2);
  in.close();

  ofstream out2("xxx2.tmp");
  wbin(out2,c2);
  wbin(out2,d2);
  wbin(out2,i2);
  out2.close();
  
  return 1;
}

#endif
