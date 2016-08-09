#include "VectorArithmetic.h"

#include <cmath>
#include <iostream>
#include <fstream>
#include "random.h"

#ifdef USE_CBLAS
#include <gsl/gsl_cblas.h>
#endif


namespace BatchUtils {
  
  using namespace std;
  
  const vector<double> operator+(const vector<double>& v1, const vector<double>& v2)
  {
    assert (v1.size() == v2.size());
    vector<double> v = v2;
#ifdef USE_CBLAS
    cblas_daxpy(v1.size(), 1., &v1[0], 1, &v[0], 1);
#else
    for (unsigned int i=0; i < v.size(); i++) {
      v[i] = v1[i] + v2[i];
    }
#endif
    return v;
  }
  
  vector<double>& operator+=(vector<double>& v1, const vector<double> &v2)
  {
    assert (v1.size() == v2.size());
#ifdef USE_CBLAS
    cblas_daxpy(v1.size(), 1., &v2[0], 1, &v1[0], 1);
#else
    for (unsigned int i=0; i < v1.size(); i++) {
      v1[i] += v2[i];
    }
#endif
    return v1;
  }
  
  
  const vector<double> operator-(const vector<double>& v1, const vector<double>& v2)
  {
    assert (v1.size() == v2.size());
    vector<double> v = v2;
#ifdef USE_CBLAS
    cblas_daxpy(v1.size(), 1., &v1[0], 1, &v[0], 1);
#else
    for (unsigned int i=0; i < v.size(); i++) {
      v[i] = v1[i] - v2[i];
    }
#endif
    return v;
  }
  
  vector<double>& operator-=(vector<double>& v1, const vector<double> &v2)
  {
    assert (v1.size() == v2.size());
#ifdef USE_CBLAS
    cblas_daxpy(v1.size(), -1., &v2[0], 1, &v1[0], 1);
#else
    for (unsigned int i=0; i < v1.size(); i++) {
      v1[i] -= v2[i];
    }
#endif
    return v1;
  }
  
  const vector<double> operator*(double scalar, const vector<double>& v1) { return v1*scalar; };
  
  
  const vector<double> operator*(const vector<double>& v1, double scalar)
  {
    vector<double> v; v.resize(v1.size());
    for (unsigned int i=0; i < v.size(); i++) {
      v[i] = v1[i] * scalar;
    }
    return v;
  }
  
  vector<double>& operator*=(vector<double>& v1, double scalar)  
  {
    for (unsigned int i=0; i < v1.size(); i++) {
      v1[i] *= scalar;
    }
    return v1;
  }
  
  const vector<double> operator/(const vector<double>& v1, double scalar)
  {
    vector<double> v; v.resize(v1.size());
    for (unsigned int i=0; i < v.size(); i++) {
      v[i] = v1[i] / scalar;
    }
    return v;
  }
  
  vector<double>& operator/=(vector<double>& v1, double scalar)
  {
    for (unsigned int i=0; i < v1.size(); i++) {
      v1[i] /= scalar;
    }
    return v1;
  }
  
  
  double VectorArithmetic::distance(const vector<double>& x1, const vector<double>& x2) 
  {
#ifdef USE_CBLAS
    vector<double> v = x2;
    cblas_daxpy(x1.size(), -1., &x1[0], 1, &v[0], 1);
    return cblas_dnrm2(v.size(), &v[0], 1);
#else 
    assert(x1.size() == x2.size());
    double acc = 0.;
    for (unsigned int i=0; i < x1.size(); i++) {
      acc += (x1[i] - x2[i]) * (x1[i] - x2[i]);
    }
    return sqrt(acc);
#endif
  }
  

  double VectorArithmetic::distance(const double* x1, const vector<double>& x2) 
  {
#ifdef USE_CBLAS
    vector<double> v = x2;
    cblas_daxpy(x2.size(), -1., x1, 1, &v[0], 1);
    return cblas_dnrm2(v.size(), &v[0], 1);
#else
    double acc = 0.;
    for (unsigned int i=0; i < x2.size(); i++) {
      acc += (x1[i] - x2[i]) * (x1[i] - x2[i]);
    }
    return sqrt(acc);
#endif
  }
  

  double VectorArithmetic::distance(const vector<double>& x1, const double* x2) 
  {
    return VectorArithmetic::distance(x2,x1);
  }
  

  double VectorArithmetic::distance (const HyperBox& box, const vector<double>& x)
  {
    assert (box.low.size() == x.size());
    double distance = 0.;
    for (unsigned int i=0; i < x.size(); i++) {
      if (x[i] < box.low[i]) distance += (x[i]-box.low[i]) * (x[i]-box.low[i]);
      if (x[i] > box.high[i]) distance += (x[i]-box.high[i]) * (x[i]-box.high[i]);
    }
    return sqrt(distance);
  }
  
  
  bool VectorArithmetic::equal(const vector<double>& x1, const vector<double>& x2) 
  {
    assert(x1.size() == x2.size());
    for (unsigned int i=0; i < x1.size(); i++) {
      if (x1[i] != x2[i]) return false;
    }
    return true;
  }
  
  bool VectorArithmetic::equal(const double* x1, const vector<double>& x2) 
  {
    for (unsigned int i=0; i < x2.size(); i++) {
      if (x1[i] != x2[i]) return false;
    }
    return true;
  }
  
  bool VectorArithmetic::equal(const vector<double>& x1, const double* x2) 
  {
    return VectorArithmetic::equal(x2,x1);
  }
  
  ostream& operator<<(ostream& str, const vector<double>& v)
  {
    for (unsigned int i=0; i < v.size(); i++) {
      str << v[i] << (i < v.size()-1 ? " " : ""); 
    }
    return str;
  }
  
  istream& operator>>(istream& str, vector<double>& v)
  {
    for (unsigned int i=0; i < v.size(); i++) {
      str >> v[i];
    }
    return str;
  }
}
