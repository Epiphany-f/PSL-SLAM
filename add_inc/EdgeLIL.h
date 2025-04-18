#ifndef ORB_SLAM2_EDGELIL_H
#define ORB_SLAM2_EDGELIL_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/eigen_types.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/stuff/misc.h"
#include "add_inc/VertexLIL.h"


#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace g2o;
namespace types_six_dof_expmap {
    void init();
}
using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;

class EdgeLILProjectXYZOnlyPoseNew : public BaseUnaryEdge<6, Vector8d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeLILProjectXYZOnlyPoseNew() {}

    virtual void computeError()
    {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Vector3d obs_line1 = _measurement.segment<3>(0);
        Vector3d obs_line2 = _measurement.segment<3>(3);
        Vector2d obs_ins = _measurement.segment<2>(6);
        Eigen::Matrix<double,2,3> Matrix23_line1;
        Eigen::Matrix<double,2,3> Matrix23_line2;
        Matrix23_line1(0,0) = cam_project(v1->estimate().map(Xw1_s))(0);
        Matrix23_line1(0,1) = cam_project(v1->estimate().map(Xw1_s))(1);
        Matrix23_line1(0,2) = 1.0;

        Matrix23_line1(1,0) = cam_project(v1->estimate().map(Xw1_e))(0);
        Matrix23_line1(1,1) = cam_project(v1->estimate().map(Xw1_e))(1);
        Matrix23_line1(1,2) = 1.0;

        Vector2d proj = cam_project(v1->estimate().map(Xw_ins));    //MapLine端点在像素平面上的投影

        Matrix23_line2(0,0) = cam_project(v1->estimate().map(Xw2_s))(0);
        Matrix23_line2(0,1) = cam_project(v1->estimate().map(Xw2_s))(1);
        Matrix23_line2(0,2) = 1.0;

        Matrix23_line2(1,0) = cam_project(v1->estimate().map(Xw2_e))(0);
        Matrix23_line2(1,1) = cam_project(v1->estimate().map(Xw2_e))(1);
        Matrix23_line2(1,2) = 1.0;

        Vector2d err_ins = obs_ins - proj;
        Vector2d err_line1 = Matrix23_line1 * obs_line1;
        Vector2d err_line2 = Matrix23_line2 * obs_line2;

        _error.segment<2>(0) = err_line1;
        _error.segment<2>(2) = err_line2;
        _error.segment<2>(4) = err_ins;

    }
    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw1_s))(2)>0.0 && (v1->estimate().map(Xw1_e))(2)>0.0 && (v1->estimate().map(Xw2_s))(2)>0.0 && (v1->estimate().map(Xw2_e))(2)>0.0 && (v1->estimate().map(Xw_ins))(2)>0.0;
    }

    void linearizeOplus(){
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans1_s = vi->estimate().map(Xw1_s);
        Eigen::Vector3d xyz_trans1_e = vi->estimate().map(Xw1_e);
        Eigen::Vector3d xyz_trans2_s = vi->estimate().map(Xw2_s);
        Eigen::Vector3d xyz_trans2_e = vi->estimate().map(Xw2_e);
        Eigen::Vector3d xyz_trans_ins = vi->estimate().map(Xw_ins);

        double x1_s = xyz_trans1_s[0];
        double y1_s = xyz_trans1_s[1];
        double invz1_s = 1.0/xyz_trans1_s[2];
        double invz1_s_2 = invz1_s*invz1_s;

        double x1_e = xyz_trans1_e[0];
        double y1_e = xyz_trans1_e[1];
        double invz1_e = 1.0/xyz_trans1_e[2];
        double invz1_e_2 = invz1_e*invz1_e;

        double l1_0 = obs_temp1(0);
        double l1_1 = obs_temp1(1);

        double x2_s = xyz_trans2_s[0];
        double y2_s = xyz_trans2_s[1];
        double invz2_s = 1.0/xyz_trans2_s[2];
        double invz2_s_2 = invz2_s*invz2_s;

        double x2_e = xyz_trans2_e[0];
        double y2_e = xyz_trans2_e[1];
        double invz2_e = 1.0/xyz_trans2_e[2];
        double invz2_e_2 = invz2_e*invz2_e;

        double l2_0 = obs_temp2(0);
        double l2_1 = obs_temp2(1);

        double x = xyz_trans_ins[0];
        double y = xyz_trans_ins[1];
        double invz = 1.0/xyz_trans_ins[2];
        double invz_2 = invz * invz;

        _jacobianOplusXi(0,0) = -fx*x1_s*y1_s*invz1_s_2*l1_0-fy*(1+y1_s*y1_s*invz1_s_2)*l1_1;
        _jacobianOplusXi(0,1) = fx*(1+x1_s*x1_s*invz1_s_2)*l1_0+fy*x1_s*y1_s*invz1_s_2*l1_1;
        _jacobianOplusXi(0,2) = -fx*y1_s*invz1_s*l1_0+fy*x1_s*invz1_s*l1_1;
        _jacobianOplusXi(0,3) = fx*invz1_s*l1_0;
        _jacobianOplusXi(0,4) = fy*invz1_s*l1_1;
        _jacobianOplusXi(0,5) = (-fx*x1_s*l1_0-fy*y1_s*l1_1)*invz1_s_2;

        _jacobianOplusXi(1,0) = -fx*x1_e*y1_e*invz1_e_2*l1_0-fy*(1+y1_e*y1_e*invz1_e_2)*l1_1;
        _jacobianOplusXi(1,1) = fx*(1+x1_e*x1_e*invz1_e_2)*l1_0+fy*x1_e*y1_e*invz1_e_2*l1_1;
        _jacobianOplusXi(1,2) = -fx*y1_e*invz1_e*l1_0+fy*x1_e*invz1_e*l1_1;
        _jacobianOplusXi(1,3) = fx*invz1_e*l1_0;
        _jacobianOplusXi(1,4) = fy*invz1_e*l1_1;
        _jacobianOplusXi(1,5) = (-fx*x1_e*l1_0-fy*y1_e*l1_1)*invz1_e_2;

        _jacobianOplusXi(2,0) = -fx*x2_s*y2_s*invz2_s_2*l2_0-fy*(1+y2_s*y2_s*invz2_s_2)*l2_1;
        _jacobianOplusXi(2,1) = fx*(1+x2_s*x2_s*invz2_s_2)*l2_0+fy*x2_s*y2_s*invz2_s_2*l2_1;
        _jacobianOplusXi(2,2) = -fx*y2_s*invz2_s*l2_0+fy*x2_s*invz2_s*l2_1;
        _jacobianOplusXi(2,3) = fx*invz2_s*l2_0;
        _jacobianOplusXi(2,4) = fy*invz2_s*l2_1;
        _jacobianOplusXi(2,5) = (-fx*x2_s*l2_0-fy*y2_s*l2_1)*invz2_s_2;

        _jacobianOplusXi(3,0) = -fx*x2_e*y2_e*invz2_e_2*l2_0-fy*(1+y2_e*y2_e*invz2_e_2)*l2_1;
        _jacobianOplusXi(3,1) = fx*(1+x2_e*x2_e*invz2_e_2)*l2_0+fy*x2_e*y2_e*invz2_e_2*l2_1;
        _jacobianOplusXi(3,2) = -fx*y2_e*invz2_e*l2_0+fy*x2_e*invz2_e*l2_1;
        _jacobianOplusXi(3,3) = fx*invz2_e*l2_0;
        _jacobianOplusXi(3,4) = fy*invz2_e*l2_1;
        _jacobianOplusXi(3,5) = (-fx*x2_e*l2_0-fy*y2_e*l2_1)*invz2_e_2;

        _jacobianOplusXi(4,0) =  x*y*invz_2*fx;
        _jacobianOplusXi(4,1) = -(1+(x*x*invz_2))*fx;
        _jacobianOplusXi(4,2) = y*invz*fx ;
        _jacobianOplusXi(4,3) = -fx*invz;
        _jacobianOplusXi(4,4) = 0;
        _jacobianOplusXi(4,5) = x*invz_2*fx;

        _jacobianOplusXi(5,0) = (1 + y*y*invz_2)*fy ;
        _jacobianOplusXi(5,1) =  -fy*x*y*invz_2;
        _jacobianOplusXi(5,2) = -fy*x*invz;
        _jacobianOplusXi(5,3) = 0;
        _jacobianOplusXi(5,4) = -fy*invz;
        _jacobianOplusXi(5,5) = fy*y*invz_2;
    }

    bool read(std::istream& is)
    {
        for(int i=0; i<8; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                is >> information()(i, j);
                if(i!=j)
                    information()(j,i) = information()(i,j);
            }
        }
        return true;
    }

    bool write(std::ostream& os) const
    {
        for(int i=0; i<8; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
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
    Eigen::Vector3d Xw1_s;
    Eigen::Vector3d Xw1_e;
    Eigen::Vector3d Xw_ins;
    Eigen::Vector3d Xw2_s;
    Eigen::Vector3d Xw2_e;
    Eigen::Vector3d obs_temp1;
    Eigen::Vector3d obs_temp2;
    double fx, fy, cx, cy;  //相机内参数
};


class EdgeLILSE3ProjectXYZ: public g2o::BaseBinaryEdge<6, Vector8d, g2o::VertexLIL, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeLILSE3ProjectXYZ() {}

    //bool read(std::istream& is);

    //bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexLIL* v2 = static_cast<const g2o::VertexLIL*>(_vertices[0]);
        Vector3d obs_line1 = _measurement.segment<3>(0);
        Vector3d obs_line2 = _measurement.segment<3>(3);
        Vector2d obs_ins = _measurement.segment<2>(6);
        Eigen::Matrix<double,2,3> Matrix23_line1;
        Eigen::Matrix<double,2,3> Matrix23_line2;
        Matrix23_line1(0,0) = cam_project(v1->estimate().map(v2->estimate().segment<3>(0)))(0);
        Matrix23_line1(0,1) = cam_project(v1->estimate().map(v2->estimate().segment<3>(0)))(1);
        Matrix23_line1(0,2) = 1.0;

        Matrix23_line1(1,0) = cam_project(v1->estimate().map(v2->estimate().segment<3>(3)))(0);
        Matrix23_line1(1,1) = cam_project(v1->estimate().map(v2->estimate().segment<3>(3)))(1);
        Matrix23_line1(1,2) = 1.0;


        Matrix23_line2(0,0) = cam_project(v1->estimate().map(v2->estimate().segment<3>(6)))(0);
        Matrix23_line2(0,1) = cam_project(v1->estimate().map(v2->estimate().segment<3>(6)))(1);
        Matrix23_line2(0,2) = 1.0;

        Matrix23_line2(1,0) = cam_project(v1->estimate().map(v2->estimate().segment<3>(9)))(0);
        Matrix23_line2(1,1) = cam_project(v1->estimate().map(v2->estimate().segment<3>(9)))(1);
        Matrix23_line2(1,2) = 1.0;

        Vector2d proj = cam_project(v1->estimate().map(v2->estimate().segment<3>(12)));    //MapLine端点在像素平面上的投影

        Vector2d err_ins = obs_ins - proj;
        Vector2d err_line1 = Matrix23_line1 * obs_line1;
        Vector2d err_line2 = Matrix23_line2 * obs_line2;

        _error.segment<2>(0) = err_line1;
        _error.segment<2>(2) = err_line2;
        _error.segment<2>(4) = err_ins;


    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexLIL* v2 = static_cast<const g2o::VertexLIL*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate().segment<3>(0)))(2)>0.0 && (v1->estimate().map(v2->estimate().segment<3>(3)))(2)>0.0 && (v1->estimate().map(v2->estimate().segment<3>(6)))(2)>0.0 && (v1->estimate().map(v2->estimate().segment<3>(9)))(2)>0.0 && (v1->estimate().map(v2->estimate().segment<3>(12)))(2)>0.0;
    }

    void linearizeOplus() {
        g2o::VertexSE3Expmap * vj= static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexLIL* vi = static_cast<g2o::VertexLIL*>(_vertices[0]);
        Eigen::Vector3d xyz1_s = vi->estimate().segment<3>(0);
        Eigen::Vector3d xyz_trans1_s = T.map(xyz1_s);
        Eigen::Vector3d xyz1_e = vi->estimate().segment<3>(3);
        Eigen::Vector3d xyz_trans1_e = T.map(xyz1_e);

        Eigen::Vector3d xyz2_s = vi->estimate().segment<3>(9);
        Eigen::Vector3d xyz_trans2_s = T.map(xyz2_s);
        Eigen::Vector3d xyz2_e = vi->estimate().segment<3>(9);
        Eigen::Vector3d xyz_trans2_e = T.map(xyz2_e);

        Eigen::Vector3d xyz_trans_ins = vi->estimate().segment<3>(12);
        Eigen::Vector3d xyz_trans = T.map(xyz_trans_ins);

        const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();


        double x1_s = xyz_trans1_s[0];
        double y1_s = xyz_trans1_s[1];
        double invz1_s = 1.0/xyz_trans1_s[2];
        double invz1_s_2 = invz1_s*invz1_s;

        double x1_e = xyz_trans1_e[0];
        double y1_e = xyz_trans1_e[1];
        double invz1_e = 1.0/xyz_trans1_e[2];
        double invz1_e_2 = invz1_e*invz1_e;

        double l1_0 = obs_temp1(0);
        double l1_1 = obs_temp1(1);

        double x2_s = xyz_trans2_s[0];
        double y2_s = xyz_trans2_s[1];
        double invz2_s = 1.0/xyz_trans2_s[2];
        double invz2_s_2 = invz2_s*invz2_s;

        double x2_e = xyz_trans2_e[0];
        double y2_e = xyz_trans2_e[1];
        double invz2_e = 1.0/xyz_trans2_e[2];
        double invz2_e_2 = invz2_e*invz2_e;

        double l2_0 = obs_temp2(0);
        double l2_1 = obs_temp2(1);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz * invz;

        _jacobianOplusXi(0,0) = fx*l1_0*invz1_s*R(0,0)+fy*l1_1*invz1_s*R(1,0)-(fx*x1_s*l1_0*invz1_s_2+fy*y1_s*l1_1*invz1_s_2)*R(2,0);
        _jacobianOplusXi(0,1) = fx*l1_0*invz1_s*R(0,1)+fy*l1_1*invz1_s*R(1,1)-(fx*x1_s*l1_0*invz1_s_2+fy*y1_s*l1_1*invz1_s_2)*R(2,1);
        _jacobianOplusXi(0,2) = fx*l1_0*invz1_s*R(0,2)+fy*l1_1*invz1_s*R(1,2)-(fx*x1_s*l1_0*invz1_s_2+fy*y1_s*l1_1*invz1_s_2)*R(2,2);

        _jacobianOplusXi(1,0) = fx*l1_0*invz1_e*R(0,0)+fy*l1_1*invz1_e*R(1,0)-(fx*x1_e*l1_0*invz1_e_2+fy*y1_e*l1_1*invz1_e_2)*R(2,0);
        _jacobianOplusXi(1,1) = fx*l1_0*invz1_e*R(0,1)+fy*l1_1*invz1_e*R(1,1)-(fx*x1_e*l1_0*invz1_e_2+fy*y1_e*l1_1*invz1_e_2)*R(2,1);
        _jacobianOplusXi(1,2) = fx*l1_0*invz1_e*R(0,2)+fy*l1_1*invz1_e*R(1,2)-(fx*x1_e*l1_0*invz1_e_2+fy*y1_e*l1_1*invz1_e_2)*R(2,2);

        _jacobianOplusXi(2,0) = fx*l2_0*invz2_s*R(0,0)+fy*l2_1*invz2_s*R(1,0)-(fx*x2_s*l2_0*invz2_s_2+fy*y2_s*l2_1*invz2_s_2)*R(2,0);
        _jacobianOplusXi(2,1) = fx*l2_0*invz2_s*R(0,1)+fy*l2_1*invz2_s*R(1,1)-(fx*x2_s*l2_0*invz2_s_2+fy*y2_s*l2_1*invz2_s_2)*R(2,1);
        _jacobianOplusXi(2,2) = fx*l2_0*invz2_s*R(0,2)+fy*l2_1*invz2_s*R(1,2)-(fx*x2_s*l2_0*invz2_s_2+fy*y2_s*l2_1*invz2_s_2)*R(2,2);

        _jacobianOplusXi(3,0) = fx*l2_0*invz2_e*R(0,0)+fy*l2_1*invz2_e*R(1,0)-(fx*x2_e*l2_0*invz2_e_2+fy*y2_e*l2_1*invz2_e_2)*R(2,0);
        _jacobianOplusXi(3,1) = fx*l2_0*invz2_e*R(0,1)+fy*l2_1*invz2_e*R(1,1)-(fx*x2_e*l2_0*invz2_e_2+fy*y2_e*l2_1*invz2_e_2)*R(2,1);
        _jacobianOplusXi(3,2) = fx*l2_0*invz2_e*R(0,2)+fy*l2_1*invz2_e*R(1,2)-(fx*x2_e*l2_0*invz2_e_2+fy*y2_e*l2_1*invz2_e_2)*R(2,2);

        _jacobianOplusXi(4,0) = -fx*invz*R(0,0) + fx*x*invz_2*R(2,0);
        _jacobianOplusXi(4,1) = -fx*R(0,1)*invz+fx*x*R(2,1)*invz_2;
        _jacobianOplusXi(4,2) = -fx*R(0,2)*invz+fx*x*R(2,2)*invz_2;

        _jacobianOplusXi(5,0) = -fy*invz*R(1,0) + fy*y*invz_2*R(2,0);
        _jacobianOplusXi(5,1) = -fy*invz*R(1,1) + fy*y*invz_2*R(2,1);
        _jacobianOplusXi(5,2) = -fy*invz*R(1,2) + fy*y*invz_2*R(2,2);

        _jacobianOplusXj(0,0) = -fx*x1_s*y1_s*invz1_s_2*l1_0-fy*(1+y1_s*y1_s*invz1_s_2)*l1_1;
        _jacobianOplusXj(0,1) = fx*(1+x1_s*x1_s*invz1_s_2)*l1_0+fy*x1_s*y1_s*invz1_s_2*l1_1;
        _jacobianOplusXj(0,2) = -fx*y1_s*invz1_s*l1_0+fy*x1_s*invz1_s*l1_1;
        _jacobianOplusXj(0,3) = fx*invz1_s*l1_0;
        _jacobianOplusXj(0,4) = fy*invz1_s*l1_1;
        _jacobianOplusXj(0,5) = (-fx*x1_s*l1_0-fy*y1_s*l1_1)*invz1_s_2;

        _jacobianOplusXj(1,0) = -fx*x1_e*y1_e*invz1_e_2*l1_0-fy*(1+y1_e*y1_e*invz1_e_2)*l1_1;
        _jacobianOplusXj(1,1) = fx*(1+x1_e*x1_e*invz1_e_2)*l1_0+fy*x1_e*y1_e*invz1_e_2*l1_1;
        _jacobianOplusXj(1,2) = -fx*y1_e*invz1_e*l1_0+fy*x1_e*invz1_e*l1_1;
        _jacobianOplusXj(1,3) = fx*invz1_e*l1_0;
        _jacobianOplusXj(1,4) = fy*invz1_e*l1_1;
        _jacobianOplusXj(1,5) = (-fx*x1_e*l1_0-fy*y1_e*l1_1)*invz1_e_2;

        _jacobianOplusXj(2,0) = -fx*x2_s*y2_s*invz2_s_2*l2_0-fy*(1+y2_s*y2_s*invz2_s_2)*l2_1;
        _jacobianOplusXj(2,1) = fx*(1+x2_s*x2_s*invz2_s_2)*l2_0+fy*x2_s*y2_s*invz2_s_2*l2_1;
        _jacobianOplusXj(2,2) = -fx*y2_s*invz2_s*l2_0+fy*x2_s*invz2_s*l2_1;
        _jacobianOplusXj(2,3) = fx*invz2_s*l2_0;
        _jacobianOplusXj(2,4) = fy*invz2_s*l2_1;
        _jacobianOplusXj(2,5) = (-fx*x2_s*l2_0-fy*y2_s*l2_1)*invz2_s_2;

        _jacobianOplusXj(3,0) = -fx*x2_e*y2_e*invz2_e_2*l2_0-fy*(1+y2_e*y2_e*invz2_e_2)*l2_1;
        _jacobianOplusXj(3,1) = fx*(1+x2_e*x2_e*invz2_e_2)*l2_0+fy*x2_e*y2_e*invz2_e_2*l2_1;
        _jacobianOplusXj(3,2) = -fx*y2_e*invz2_e*l2_0+fy*x2_e*invz2_e*l2_1;
        _jacobianOplusXj(3,3) = fx*invz2_e*l2_0;
        _jacobianOplusXj(3,4) = fy*invz2_e*l2_1;
        _jacobianOplusXj(3,5) = (-fx*x2_e*l2_0-fy*y2_e*l2_1)*invz2_e_2;

        _jacobianOplusXj(4,0) =  x*y*invz_2*fx;
        _jacobianOplusXj(4,1) = -(1+(x*x*invz_2))*fx;
        _jacobianOplusXj(4,2) = y*invz*fx ;
        _jacobianOplusXj(4,3) = -fx*invz;
        _jacobianOplusXj(4,4) = 0;
        _jacobianOplusXj(4,5) = x*invz_2*fx;

        _jacobianOplusXj(5,0) = (1 + y*y*invz_2)*fy ;
        _jacobianOplusXj(5,1) =  -fy*x*y*invz_2;
        _jacobianOplusXj(5,2) = -fy*x*invz;
        _jacobianOplusXj(5,3) = 0;
        _jacobianOplusXj(5,4) = -fy*invz;
        _jacobianOplusXj(5,5) = fy*y*invz_2;

    }

    bool read(std::istream& is)
    {
        for(int i=0; i<8; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                is >> information()(i, j);
                if(i!=j)
                    information()(j,i) = information()(i,j);
            }
        }
        return true;
    }

    bool write(std::ostream& os) const
    {
        for(int i=0; i<8; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
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
    Eigen::Vector3d Xw1_s;
    Eigen::Vector3d Xw1_e;
    Eigen::Vector3d Xw_ins;
    Eigen::Vector3d Xw2_s;
    Eigen::Vector3d Xw2_e;
    Eigen::Vector3d obs_temp1;
    Eigen::Vector3d obs_temp2;
    double fx, fy, cx, cy;
};

#endif