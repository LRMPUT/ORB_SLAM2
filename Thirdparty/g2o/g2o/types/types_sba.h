// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Code modified by Michal Nowicki

#ifndef G2O_SBA_TYPES
#define G2O_SBA_TYPES

#include "../core/base_vertex.h"
#include "../core/eigen_types.h"

#include <Eigen/Geometry>
#include <iostream>

namespace g2o {

/**
 * \brief Point vertex, XYZ
 */
 class VertexSBAPointXYZ : public BaseVertex<3, Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    VertexSBAPointXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
      _estimate.fill(0.);
    }

    virtual void oplusImpl(const double* update)
    {
      Eigen::Map<const Vector3d> v(update);
      _estimate += v;
    }
};

    class CameraParameters : public g2o::Parameter {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        CameraParameters();

        CameraParameters(double focal_length_x,
                         double focal_length_y,
                         const Vector2D &principle_point,
                         double baseline)
                : focal_length_x(focal_length_x),
                  focal_length_y(focal_length_y),
                  principle_point(principle_point),
                  baseline(baseline) {}

        Vector2D cam_map(const Vector3D &trans_xyz) const;

        Vector2D mostcam_map(const Vector3D &trans_xyz, const double base) const;

        Vector3D stereocam_uvu_map(const Vector3D &trans_xyz) const;

        virtual bool read(std::istream &is) {
            is >> focal_length_x;
            is >> focal_length_y;
            is >> principle_point[0];
            is >> principle_point[1];
            is >> baseline;
            return true;
        }

        virtual bool write(std::ostream &os) const {
            os << focal_length_x << " ";
            os << focal_length_y << " ";
            os << principle_point.x() << " ";
            os << principle_point.y() << " ";
            os << baseline << " ";
            return true;
        }

        double focal_length_x, focal_length_y;
        Vector2D principle_point;
        double baseline;
    };

    /*
     * \brief Point vertex, single inverse depth
     */
    class VertexSBAPointInvD : public BaseVertex<1, double>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSBAPointInvD();
        bool read(std::istream& is);
        bool write(std::ostream& os) const;

        virtual void setToOriginImpl() {
            _estimate = 0;
        }

        virtual void oplusImpl(const double* update)
        {
            _estimate += update[0];
        }

        // u,v of the initial recognition
        double u0, v0;
    };


} // end namespace

#endif // SBA_TYPES
