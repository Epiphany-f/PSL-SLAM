#ifndef ORB_SLAM2_LILEDGE_H
#define ORB_SLAM2_LILEDGE_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace g2o;
namespace types_six_dof_expmap {
    void init();
}
using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;

class EdgeLILProjectXYZOnlyPose:public BaseUnaryEdge<2, Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeLILProjectXYZOnlyPose() {}
    virtual void computeError()
    {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(Xw));
    }

//     virtual void linearizeOplus()
//     {
//         VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
//         Vector3d xyz_trans = vi->estimate().map(Xw);
//
//         double x = xyz_trans[0];
//         double y = xyz_trans[1];
//         double invz = 1.0/xyz_trans[2];
//         double invz_2 = invz * invz;
//
//
//         // 1*6 jacobian
// //        _jacobianOplusXi(0,0) = fy*ly + fx*lx*x*y*invz_2 + fy*ly*y*y*invz_2;
// //        _jacobianOplusXi(0,1) = -fx*lx - fx*lx*x*x*invz_2 - fy*ly*x*y*invz_2;
// //        _jacobianOplusXi(0,2) = fx*lx*y*invz - fy*ly*x*invz;
// //        _jacobianOplusXi(0,3) = -fx*lx*invz;
// //        _jacobianOplusXi(0.4) = -fy*ly*invz;
// //        _jacobianOplusXi(0,5) = (fx*lx*x+fy*ly*y)*invz_2;
//
//         _jacobianOplusXi(0,0) =  fx*y*invz ;
//         _jacobianOplusXi(0,1) = -fx - fx*x*x*invz_2;
//         _jacobianOplusXi(0,2) = fx*x*y*invz_2 ;
//         _jacobianOplusXi(0,3) = fx*x*invz_2;
//         _jacobianOplusXi(0,4) = 0;
//         _jacobianOplusXi(0,5) = -(fx*invz);
//
//         _jacobianOplusXi(1,0) = -fy*x*invz ;
//         _jacobianOplusXi(1,1) =  -fy*x*y*invz_2;
//         _jacobianOplusXi(1,2) = fy + fy*y*y*invz_2;
//         _jacobianOplusXi(1,3) = fy*y*invz_2;
//         _jacobianOplusXi(1,4) = fy*invz;
//         _jacobianOplusXi(1,5) = 0;
//
//     }

    bool read(std::istream& is)
    {
        for(int i=0; i<3; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if(i!=j)
                    information()(j,i) = information()(i,j);
            }
        }
        return true;
    }

    bool write(std::ostream& os) const
    {
        for(int i=0; i<3; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                os << " " << information()(i,j);
            }
        }
        return os.good();
    }

    Vector2d project2d(const Vector3d& v)
    {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector2d cam_project(const Vector3d& trans_xyz)
    {
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    Vector3d Xw;    //交点坐标
    double fx, fy, cx, cy;  //相机内参数    
};


class EdgeLILProjectXYZ : public BaseBinaryEdge<2, Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeLILProjectXYZ() {}

    virtual void computeError()
    {
        const VertexSE3Expmap* v1 = static_cast<VertexSE3Expmap *>(_vertices[0]);
        const VertexSBAPointXYZ* v2 = static_cast<VertexSBAPointXYZ*>(_vertices[1]);

        Vector2d obs(_measurement);
        _error = obs-cam_project(v1->estimate().map(Xw));
    }

//     virtual void linearizeOplus()
//     {
//         // 位姿顶点
//         VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
//         SE3Quat T(vj->estimate());
//         Vector3d xyz_trans = T.map(Xw);    //线段端点的世界坐标系转换到相机坐标系下
//
//         // 线段端点顶点
// //        VertexLinePointXYZ *vj = static_cast<VertexLinePointXYZ *>(_vertices[1]);
//         VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
//         Vector3d xyz = vi->estimate();
//
//         double x = xyz_trans[0];
//         double y = xyz_trans[1];
//         double invz = 1.0/xyz_trans[2];
//         double invz_2 = invz * invz;
//
//
//         // 3*6 jacobian
//         // 1.这是最开始推导雅克比时，有负号的
// //        _jacobianOplusXj(0,0) = fy*ly + fx*lx*x*y*invz_2 + fy*ly*y*y*invz_2;
// //        _jacobianOplusXj(0,1) = -fx*lx - fx*lx*x*x*invz_2 - fy*ly*x*y*invz_2;
// //        _jacobianOplusXj(0,2) = fx*lx*y*invz - fy*ly*x*invz;
// //        _jacobianOplusXj(0,3) = -fx*lx*invz;
// //        _jacobianOplusXj(0.4) = -fy*ly*invz;
// //        _jacobianOplusXj(0,5) = (fx*lx*x+fy*ly*y)*invz_2;
//
//         // 雅克比没有负号的
//         _jacobianOplusXj(0,0) =  fx*y*invz ;
//         _jacobianOplusXj(0,1) = -fx - fx*x*x*invz_2;
//         _jacobianOplusXj(0,2) = fx*x*y*invz_2 ;
//         _jacobianOplusXj(0,3) = fx*x*invz_2;
//         _jacobianOplusXj(0,4) = 0;
//         _jacobianOplusXj(0,5) = -(fx*invz);
//
//         _jacobianOplusXj(1,0) = -fy*x*invz ;
//         _jacobianOplusXj(1,1) =  -fy*x*y*invz_2;
//         _jacobianOplusXj(1,2) = fy + fy*y*y*invz_2;
//         _jacobianOplusXj(1,3) = fy*y*invz_2;
//         _jacobianOplusXj(1,4) = fy*invz;
//         _jacobianOplusXj(1,5) = 0;
//
//         Matrix<double, 2, 3, Eigen::ColMajor> tmp;
//         tmp.setZero();
//         tmp(0,0) = -fx;
//         tmp(0,1) = 0;
//         tmp(0,2) = fx*x*invz;
//         tmp(1,0) = 0;
//         tmp(1,1) = -fy*invz;
//         tmp(1,2) = fy*y*invz;
//
//         Matrix<double, 3, 3> R;
//         R = T.rotation().toRotationMatrix();
//
// //        _jacobianOplusXi = -1. * invz * tmp * R;
//         _jacobianOplusXi = 1. * invz * tmp * R;
//     }

    bool read(std::istream& is)
    {
        for(int i=0; i<3; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if(i!=j)
                    information()(j,i) = information()(i,j);
            }
        }
        return true;
    }

    bool write(std::ostream& os) const
    {
        for(int i=0; i<3; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                os << " " << information()(i,j);
            }
        }
        return os.good();
    }

    Vector2d project2d(const Vector3d& v)
    {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector2d cam_project(const Vector3d& trans_xyz)
    {
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    Vector3d Xw;    //MapLine的一个端点在世界坐标系的位置
    double fx, fy, cx, cy;  //相机内参数
};

#endif

