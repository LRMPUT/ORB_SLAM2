//
// Author: Michal Nowicki
//

#ifndef ORB_SLAM2_TYPES_SIX_DOF_PHOTO_H
#define ORB_SLAM2_TYPES_SIX_DOF_PHOTO_H

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "../core/base_multi_edge.h"
#include "../types/types_six_dof_expmap.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include <Eigen/Geometry>
#include <Eigen/Core>

namespace g2o {
    using namespace std;

    typedef Eigen::Matrix<double,9,1,Eigen::ColMajor> Vector9D;

    struct imgStr {
        std::vector< std::vector< float> > image;
        std::vector< std::vector< Eigen::Vector2f > > gradient;
    };

    class EdgeInverseDepthPatch : public g2o::BaseMultiEdge<9, Vector9D> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeInverseDepthPatch()  {
            resizeParameters(1);
            installParameter(_cam, 0);

            //   x
            //  x x
            // x x x - neighbours used in optimization
            //  x x
            //   x
            neighbours.push_back(make_pair(0,0));
            neighbours.push_back(make_pair(0,2));
            neighbours.push_back(make_pair(1,1));
            neighbours.push_back(make_pair(2,0));
            neighbours.push_back(make_pair(1,-1));
            neighbours.push_back(make_pair(0,-2));
            neighbours.push_back(make_pair(-1,-1));
            neighbours.push_back(make_pair(-2,0));
            neighbours.push_back(make_pair(-1,1));
        }

        void setAdditionalData(imgStr *imageAnchor,
                               imgStr *imageObs,
                               double _baseline, double _pyramidScale) {
            imgAnchor = imageAnchor;
            imgObs = imageObs;
            baseline = _baseline;
            pyramidScale = _pyramidScale;
        }

        virtual bool read  (std::istream& is);
        virtual bool write (std::ostream& os) const;
        void computeError();
//        virtual void linearizeOplus ();


        inline Eigen::Matrix<double, 1, 2> d_inten_d_proj(const double u, const double v) ;
        inline Matrix<double, 2, 3, Eigen::ColMajor> d_proj_d_y(const double &fx, const double &fy, const Vector3D &xyz);
        inline Matrix<double, 3, 6, Eigen::ColMajor> d_expy_d_y(const Vector3D &y);
        inline Matrix<double, 3, 1, Eigen::ColMajor> d_Tinvpsi_d_psi(const SE3Quat &T, const Vector3D &psi);

        bool isDepthPositive();

        CameraParameters * _cam;

    private:
        inline Vector3D invert_depth(const Vector3D &x) {
            return unproject2d(x.head<2>()) / x[2];
        }

        Vector3d unproject2d(const Vector2d &v) {
            Vector3d res;
            res(0) = v(0);
            res(1) = v(1);
            res(2) = 1;
            return res;
        }

        inline double getSubpixImageValue(double u, double v, std::vector< std::vector< float> > &image);

        std::vector< std::pair<double, double> > neighbours;

        double baseline; // Stereo offset
        double pyramidScale;
        imgStr *imgAnchor;
        imgStr *imgObs;
    };
}
#endif //ORB_SLAM2_TYPES_SIX_DOF_PHOTO_H
