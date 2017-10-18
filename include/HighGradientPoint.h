//
// Created by michalnowicki on 17.10.17.
//

#ifndef ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_HIGHGRADIENTPOINT_H
#define ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_HIGHGRADIENTPOINT_H

#include "KeyFrame.h"

namespace ORB_SLAM2 {

    class KeyFrame;


    class HighGradientPoint {

    public:
        HighGradientPoint(double u, double v, double invDepth);

        cv::Mat getGlobalPosition();

        static list<HighGradientPoint*> DistributeOctTree(KeyFrame* currentKF,
                                                          list<HighGradientPoint*> vToDistributeKeys, const int &minX,
                                                          const int &maxX, const int &minY, const int &maxY, const int &N);




        long unsigned int id;
        static long unsigned int nextId;

        double u, v;
        double invDepth;

        double curKF_u, curKF_v;

        KeyFrame* refKF;

    };

    class ExtractorNodeHG
    {
    public:
        ExtractorNodeHG():bNoMore(false){}

        void DivideNode(ExtractorNodeHG &n1, ExtractorNodeHG &n2, ExtractorNodeHG &n3, ExtractorNodeHG &n4);

        std::vector<HighGradientPoint*> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNodeHG>::iterator lit;
        bool bNoMore;
    };
}

#endif //ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_HIGHGRADIENTPOINT_H
