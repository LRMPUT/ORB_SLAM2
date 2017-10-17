//
// Created by michalnowicki on 17.10.17.
//

#ifndef ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_HIGHGRADIENTPOINT_H
#define ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_HIGHGRADIENTPOINT_H

namespace ORB_SLAM2 {
    class HighGradientPoint {

    public:
        HighGradientPoint(double u, double v, double invDepth);

        long unsigned int id;
        static long unsigned int nextId;

        double u, v;
        double invDepth;

    };
}

#endif //ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_HIGHGRADIENTPOINT_H
