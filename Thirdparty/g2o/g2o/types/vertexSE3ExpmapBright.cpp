//
// Author: Michal Nowicki
//

#include "vertexSE3ExpmapBright.h"

namespace g2o {

    VertexSE3ExpmapBright::VertexSE3ExpmapBright() : BaseVertex<10, SE3QuatBright>() {
    }

    bool VertexSE3ExpmapBright::read(std::istream &is) {
        Vector7d est;
        for (int i = 0; i < 7; i++)
            is >> est[i];
        SE3Quat cam2world;
        cam2world.fromVector(est);


        SE3QuatBright fullEstimate;
        fullEstimate.se3quat = cam2world.inverse();
        is >> fullEstimate.aL;
        is >> fullEstimate.bL;
        is >> fullEstimate.aR;
        is >> fullEstimate.bR;

        setEstimate(fullEstimate);
        return true;
    }

    bool VertexSE3ExpmapBright::write(std::ostream &os) const {
        SE3QuatBright est = estimate();

        SE3Quat cam2world(est.se3quat.inverse());
        for (int i = 0; i < 7; i++)
            os << cam2world[i] << " ";
        os << est.aL << " " << est.bL << " " << est.aR << " " << est.bR << " ";
        return os.good();
    }

}

