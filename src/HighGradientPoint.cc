//
// Created by michalnowicki on 17.10.17.
//

#include "HighGradientPoint.h"

namespace ORB_SLAM2 {
    long unsigned int HighGradientPoint::nextId=0;

    HighGradientPoint::HighGradientPoint(double u, double v, double invDepth) :
            u(u), v(v), invDepth(invDepth) {
        id = nextId++;
    }
}