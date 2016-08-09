#include "BatchData.h"

#include <cmath>
#include <iostream>
#include <assert.h>
#include <fstream>


namespace BatchUtils {

  using namespace std;
  
  XUiX::XUiX(const XUX& xux, const vector<vector<double> >& all_actions)
    : X(xux.x), xnext(xux.xnext), r(xux.r), isTerminal(xux.isTerminal)
  {
    bool isSame=false;
    for (unsigned int i=0; i < all_actions.size(); i++) {
      isSame = true;
      for (unsigned int d=0; d < all_actions[i].size(); d++) {
        if (all_actions[i][d] != xux.u[d]) {
          isSame = false;
          break;
        }
      }
      if (isSame) {
        ui = i;
        break;
      }
    }
    if (!isSame) {
      throw(exception());
    }
  }
  
  
  XUiT::XUiT(const XUT& xut, const vector<vector<double> >& all_actions)
  : XT(xut.x, xut.target)
  {
    bool isSame=false;
    for (unsigned int i=0; i < all_actions.size(); i++) {
      isSame = true;
      for (unsigned int d=0; d < all_actions[i].size(); d++) {
        if (all_actions[i][d] != xut.u[d]) {
          isSame = false;
          break;
        }
      }
      if (isSame) {
        ui = i;
        break;
      }
    }
    if (!isSame) {
      throw(exception());
    }
  }
  
  
  void translate(const vector<XUT>& src, vector<XUiT>& trg, const vector<vector<double> >& actions)
  {
    trg.clear();
    for (unsigned int i=0; i < src.size(); i++) {
      trg.push_back(XUiT(src[i], actions));
    }
  }
  
  
  void translate(const vector<XUX>& src, vector<XUiX>& trg, const vector<vector<double> >& actions)
  {
    trg.clear();
    for (unsigned int i=0; i < src.size(); i++) {
      trg.push_back(XUiX(src[i], actions));
    }
  }

  
}


