//
// Author: Michal Nowicki
//

#ifndef CORNEREDGEHARRIS_H
#define CORNEREDGEHARRIS_H

namespace ORB_SLAM2 {
    void cornerEdgeHarrisExtractor(cv::InputArray _image, cv::OutputArray _corners, cv::OutputArray _lambdasVectors,
                                   int maxCorners, double qualityLevel, double minDistance,
                                   cv::InputArray _mask, int blockSize, double harrisK);
}
#endif //CORNEREDGEHARRIS_H
