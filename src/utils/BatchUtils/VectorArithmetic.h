#ifndef _vectorarithmetic_h_
#define _vectorarithmetic_h_

#include <vector>
#include <iostream>

#include <cassert>


//#define USE_CBLAS

namespace BatchUtils {
  
  using namespace std;
  
  using std::vector;
  using std::ostream;
  using std::istream;
  
  struct HyperBox {
    vector<double> low, high;
    HyperBox() {}
    HyperBox(const vector<double>& low, const vector<double>& high) 
    : low(low), high(high) 
    { assert(low.size() == high.size() && low.size() > 0); }
  };
  
  class VectorArithmetic {
  public:
    template<class T> static void setTo(vector<T>& x1, const T& scalar);
    
#ifdef XCode
#pragma mark -
#pragma mark Equality and Distance
#endif
    
    /** test for equality of two double vectors */
    static bool equal(const vector<double>& x1, const vector<double>& x2);
    /** test for equality of a double array and a double vector assuming same
     length */
    static bool equal(const double* x1, const vector<double>& x2);
    /** test for equality of a double array and a double vector assuming same
     length */
    static bool equal(const vector<double>& x1, const double* x2);
    
    /** euclidian distance between two vectors */
    static double distance(const vector<double>& x1, const vector<double>& x2);  
    /** euclidian distance between two double vectors of the same length, where
     the first vector is represented by an array of doubles and the second
     vector is represented by an STL-vector. */
    static double distance(const double* x1, const vector<double>& x2); 
    /** euclidian distance between two double vectors of the same length, where
     the second vector is represented by an array of doubles and the first
     vector is represented by an STL-vector. */
    static double distance(const vector<double>& x1, const double* x2); 
    
    /** distance between a vector and the multi-dimensional rectangle */
    static double distance (const HyperBox&, const vector<double>& x);
    /** distance between a vector and the multi-dimensional rectangle */
    static double distance (const vector<double>& x, const HyperBox& box) { return distance(box, x); }
    
#ifdef XCode
#pragma mark -
#pragma mark Addition and Subtraction
#endif
   
    /** addition of two double vectors of equal length */
    friend const vector<double> operator+(const vector<double>& v1, const vector<double>& x2);
    /** addition of a vector of doubles to the first argument. */
    friend vector<double>& operator+=(vector<double>& v1, const vector<double> &x2);
    /** subtraction of double vectors of equal length */
    friend const vector<double> operator-(const vector<double>& v1, const vector<double>& x2);
    /** subtracts a vector of doubles from the first argument */
    friend vector<double>& operator-=(vector<double>& v1, const vector<double> &x2);
    
#ifdef XCode
#pragma mark -
#pragma mark Scalar Multiplication
#endif
    
    /** scalar multiplication for double vectors */
    friend const vector<double> operator*(const vector<double>& v1, double scalar);
    /** scalar multiplication for double vectors */
    friend const vector<double> operator*(double scalar, const vector<double>& v1);
    /** multiplies the double vecotr by a scalar */
    friend vector<double>& operator*=(vector<double>& v1, double scalar);  
    /** division of a double vector by a scalar */
    friend const vector<double> operator/(const vector<double>& v1, double scalar);
    /** divides a double vector by a scalar */
    friend vector<double>& operator/=(vector<double>& v1, double scalar);
  };
  
#ifdef XCode
#pragma mark -
#pragma mark Declarations and Inline Implementations
#endif

  /** writes a double vector to the given stream */
  std::ostream& operator<<(std::ostream& str, const std::vector<double>& v);
  /** reads a double vector from the given stream, where v.size() determines
   the number of doubles read from the stream */
  std::istream& operator>>(std::istream& str, std::vector<double>& v);
  
  
  template<class T>
  inline void VectorArithmetic::setTo(vector<T>& x1, const T& scalar)
  {
    for (unsigned int i=0; i < x1.size(); i++) {
      x1[i] = scalar;
    }
  }
  
  const vector<double> operator+(const vector<double>& v1, const vector<double>& x2);
  vector<double>& operator+=(vector<double>& v1, const vector<double> &x2);
  const vector<double> operator-(const vector<double>& v1, const vector<double>& x2);
  vector<double>& operator-=(vector<double>& v1, const vector<double> &x2);
  
  const vector<double> operator*(const vector<double>& v1, double scalar);
  const vector<double> operator*(double scalar, const vector<double>& v1);
  vector<double>& operator*=(vector<double>& v1, double scalar);  
  const vector<double> operator/(const vector<double>& v1, double scalar);
  vector<double>& operator/=(vector<double>& v1, double scalar);
}
#endif