#ifndef __ONLINESTAT_H__
#define __ONLINESTAT_H__

/** Online algorithm for computing mean and variance. */
class OnlineStat
{
public:
	OnlineStat () {reset();}
	virtual ~OnlineStat () {};
  /** Reset all values to zero. */
	void reset() {mean = m2 = n = 0.0;}
  /** Append value to statistics. */
	void append(double x)
  {
    ++n;
    double delta = x-mean;
    mean += delta/n;
    m2 += delta*(x-mean);
  }
  /** Return current variance. */
	double get_var() { return m2/n; }
  /** Return current mean. */
	double get_mean() { return mean; }
  /** Return sample size. */
	double get_n() { return n; }
  /** Return variance * n. */
	double get_n_times_var() { return m2; }
private:
	double mean;
	double n;
	double m2;
};

#endif /* __ONLINESTAT_H__ */