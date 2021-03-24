#include "arena_path_search/jps_utils.h"

constexpr int JPS2DNeib::nsz[3][2];

JPS2DNeib::JPS2DNeib() {
  int id = 0;
  for(int dy = -1; dy <= 1; ++dy) {
    for(int dx = -1; dx <= 1; ++dx) {
      int norm1 = std::abs(dx) + std::abs(dy);
      for(int dev = 0; dev < nsz[norm1][0]; ++dev)
        Neib(dx,dy,norm1,dev, ns[id][0][dev], ns[id][1][dev]);
      for(int dev = 0; dev < nsz[norm1][1]; ++dev)
      {
        FNeib(dx,dy,norm1,dev,
            f1[id][0][dev],f1[id][1][dev],
            f2[id][0][dev],f2[id][1][dev]);
      }
      id ++;
    }
  }
}

void JPS2DNeib::print() {
  for(int dx = -1; dx <= 1; dx++) {
    for(int dy = -1; dy <= 1; dy++) {
      int id = (dx+1)+3*(dy+1);
      printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
      for(unsigned int i = 0; i < sizeof(f1[id][0])/sizeof(f1[id][0][0]); i++)
        printf("                f1: [%d, %d]\n", f1[id][0][i], f1[id][1][i]);
    }
  }
}

void JPS2DNeib::Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; return;
        case 1: tx=-1; ty=0; return;
        case 2: tx=0; ty=1; return;
        case 3: tx=1; ty=1; return;
        case 4: tx=-1; ty=1; return;
        case 5: tx=0; ty=-1; return;
        case 6: tx=1; ty=-1; return;
        case 7: tx=-1; ty=-1; return;
     }
    case 1:
      tx = dx; ty = dy; return;
    case 2:
      switch(dev)
      {
        case 0: tx = dx; ty = 0; return;
        case 1: tx = 0; ty = dy; return;
        case 2: tx = dx; ty = dy; return;
      }
  }
}

void JPS2DNeib::FNeib( int dx, int dy, int norm1, int dev,
                       int& fx, int& fy, int& nx, int& ny)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; break;
        case 1: fx= 0; fy= -1;  break;
      }

      // switch order if different direction
      if(dx == 0)
        fx = fy, fy = 0;

      nx = dx + fx; ny = dy + fy;
      return;
    case 2:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0;
          nx = -dx; ny = dy;
          return;
        case 1:
          fx = 0; fy = -dy;
          nx = dx; ny = -dy;
          return;
      }
  }
}
