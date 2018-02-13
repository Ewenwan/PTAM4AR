#ifndef __MathUtility_H
#define __MathUtility_H

#include <Eigen/Core>
#include <sophus/se3.h>

namespace cg {

class MathUtility
{
public:
    inline static Eigen::Vector3d Homogeneous(Eigen::Vector2d v2d)
    {
        Eigen::Vector3d v3d;
        v3d.segment(0,2) = v2d;
        v3d[2] = 1.0;
        return v3d;
    }
    inline static Eigen::Vector2d InHomogeneous(Eigen::Vector3d v3d)
    {
        Eigen::Vector2d v2d;
        v2d[0] = v3d[0]/v3d[2];
        v2d[1] = v3d[1]/v3d[2];
        return v2d;
    }

    // Finds 3d coords of point in reference frame B from two z=1 plane projections
    inline static Eigen::Vector3d Triangulate(const Sophus::SE3 &se3AfromB, const Eigen::Vector2d &v2A, const Eigen::Vector2d &v2B)
    {
        Eigen::Matrix4d PDash = se3AfromB.matrix();

        Eigen::Matrix4d A;
        A(0,0) = -1.0; A(0,1) =  0.0; A(0,2) = v2B[0]; A(0,3) = 0.0;
        A(1,0) =  0.0; A(1,1) = -1.0; A(1,2) = v2B[1]; A(1,3) = 0.0;
        A.row(2) = v2A[0] * PDash.row(2) - PDash.row(0);
        A.row(3) = v2A[1] * PDash.row(2) - PDash.row(1);

        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix4d m4V = svd.matrixV();
        Eigen::Vector4d v4V = m4V.col(3);//column vector corresponding to the smallest singular value

        if(v4V[3] < 1e-8)
            v4V[3] = 0.00001;
        return v4V.hnormalized();
    }
};

}


#endif
