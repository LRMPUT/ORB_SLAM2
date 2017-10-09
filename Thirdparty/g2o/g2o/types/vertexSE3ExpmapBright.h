//
// Author: Michal Nowicki
//

#ifndef ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_VERTEXSE3EXPMAPBRIGHT_H
#define ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_VERTEXSE3EXPMAPBRIGHT_H

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "../core/base_multi_edge.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include <Eigen/Geometry>


namespace g2o {

    using namespace Eigen;
    typedef Matrix<double, 10, 1> Vector10d;

    struct SE3QuatBright {
        SE3Quat se3quat;
        double aL, bL;
        double aR, bR;
    };

    class VertexSE3ExpmapBright : public BaseVertex<10, SE3QuatBright> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSE3ExpmapBright();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        virtual void setToOriginImpl() {
            SE3QuatBright est;
            est.se3quat = SE3Quat();
            est.aL = 0;
            est.bL = 0;
            est.aR = 0;
            est.bR = 0;
            _estimate = est;
        }

        virtual void oplusImpl(const double *update_) {
            Eigen::Map<const Vector10d> update(update_);
            Eigen::Map<const Vector6d> updatePose(update.head(6).data());

            SE3QuatBright est = estimate();
            est.se3quat = SE3Quat::exp(updatePose) * est.se3quat;
            est.aL = est.aL + update[6];
            est.bL = est.bL + update[7];
            est.aR = est.aR + update[8];
            est.bR = est.bR + update[9];
            setEstimate(est);
        }
    };
}

#endif //ORB_SLAM2_PHOTOMETRIC_OPTIMIZATION_VERTEXSE3EXPMAPBRIGHT_H
