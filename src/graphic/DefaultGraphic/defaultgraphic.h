#ifndef _DEFAULTGRAPHIC_H_
#define _DEFAULTGRAPHIC_H_

#include "graphic.h"
/** Default class for graphical output. 
  * This class is used if no graphical output is needed
  * \ingroup GRAPHIC */
class DefaultGraphic : public Graphic {
  virtual bool notify(const double*, const double*,const double *, const double *, 
		      const long, const long,
		      const float, const float, const long) { return true; }
  virtual bool init(int, int, int, int, double, const char *fname=0) { return true;}
  virtual bool deinit() { return true; } //terminate  
};

REGISTER_GRAPHIC(DefaultGraphic, "Default graphic module. Simply does nothing.")


#endif
