#ifndef _JPS_UTILS_H_
#define _JPS_UTILS_H_

#include <iostream>


struct JPS2DNeib;
typedef JPS2DNeib *JPS2DNeibPtr;

struct JPS2DNeib {
    // for each (dx,dy) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    int ns[9][2][8];
    int f1[9][2][2];
    int f2[9][2][2];
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        8 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced
    static constexpr int nsz[3][2] = {{8, 0}, {1, 2}, {3, 2}};

    void print();
    JPS2DNeib();
    private:
    void Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty);
    void FNeib(int dx, int dy, int norm1, int dev,
        int& fx, int& fy, int& nx, int& ny);

    //~JPS2DNeib(){};   // cannot add that because of distruction in jps.cpp, don't know why
};


#endif