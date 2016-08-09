#ifndef _Utils_random_h_
#define _Utils_random_h_

#include <cstdlib>
#include <cmath>

namespace Util {
  
  // Verschiedene Zufallszahlengeneratoren
  
  /** seed setzen */
  inline void random_seed (const unsigned int& s) throw () {
  std::srand(s); }
  
  /** gleichverteilte Zufallszahl in [0,1] */
  inline double urandom () throw () {
  return static_cast<double>(std::rand())/RAND_MAX; }
  
  /** gleichverteilte Zufallszahl in [f,t] */
  inline double urandom (const double& f, const double& t) throw () {
  return (t-f)*urandom ()+f; }
  
  /** Bernoulli-Experiment mit Erfolgswahrscheinlichkeit p */
  inline bool brandom (const double& p) throw () {
  return (urandom()<p); }
  
  /** Standard-normalverteilte Zufallszah aus zwei unabhängigen, gleichverteilten
      Zufallszahlen nach der Box-Muller-Methode berechnen */
  inline double nrandom () throw () {
    double u1 = urandom();
    double u2 = urandom();
    return (std::sqrt(-2.0*std::log(u1))*cos(6.2831853071*u2));
  }
  
  /** Normalverteilte Zufallszahl N(mu,sigma^2) */
  inline double nrandom (const double& mu, const double& sigma) throw () {
  return nrandom()*sigma+mu; }
  




  /* random procedures by Jan Peters */
  void set_seed(const int seed);
  double gaussian(double value, double std);
  extern int        seed;
}

#endif
