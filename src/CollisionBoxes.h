/*
    Fast collision detector

    Created 2019-01 by Richard Barnes (richard.barnes@berkeley.edu)
*/

#ifndef __collision_boxes_h_
#define __collision_boxes_h_

#include "Robot.h"
#include <cmath>
#include <cassert>
#include <unordered_map>

namespace KiloSim
{

class CollisionBoxes {
 private:
  const int cddx[9] = {0, -1, -1,  0,  1, 1, 1, 0, -1};
  const int cddy[9] = {0,  0, -1, -1, -1, 0, 1, 1,  1};  
  const int PSIZE = 4;
  typedef std::vector<int> ivec;
  ivec   agent_positions;
  ivec   cells_used;
  double radius;  //Collision radius
  int    bwidth;  //Width in bins
  int    bheight; //Height in bins

  ivec ret_vec; //Vector of return values

 public:
  CollisionBoxes() = default;

  CollisionBoxes(const double width0, const double height0, const double radius0){
    radius  = radius0;
    bwidth  = std::ceil(width0/radius);
    bheight = std::ceil(height0/radius);

    agent_positions.resize(PSIZE*bwidth*bheight,-1);
  }

  template<class T>
  void update(const std::vector<T> &agents){
    //Clear bins of their occupants
    for(const auto &p: cells_used)
      agent_positions[p] = -1;
    cells_used.clear();

    for(unsigned int a=0;a<agents.size();a++){
      const int binx = agents[a].x/radius;
      const int biny = agents[a].y/radius;
      const int idx0 = biny*bwidth+binx;
      int idx=idx0;
      for(;idx<=idx0+PSIZE;idx++)
        if(agent_positions[idx]==-1)
          break;
      assert(idx!=idx0+PSIZE);
      agent_positions[idx] = a;
      cells_used.emplace_back(idx);
    }
  }

  const ivec& operator()(const double x, const double y) {
    ret_vec.clear();
    const int cbinx = x/radius;
    const int cbiny = y/radius;

    for(unsigned int nbi=0;nbi<=8;nbi++){
      const int binx = cbinx+cddx[nbi];
      const int biny = cbiny+cddy[nbi];

      if(binx<0 || biny<y || binx==bwidth || biny==bheight)
        continue;

      const auto idx0 = &agent_positions[PSIZE*(biny*bwidth+binx)];
      for(auto idx=idx0;idx<idx0+PSIZE;idx++)
        if(*idx!=-1)
          ret_vec.emplace_back(*idx);
    }

    return ret_vec;
  }
};

}

#endif