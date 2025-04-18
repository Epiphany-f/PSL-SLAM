//
// Created by jiajieshi on 22-9-7.
//

#ifndef ORB_SLAM2_TYPES_LINE_EXPMAP_H
#define ORB_SLAM2_TYPES_LINE_EXPMAP_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace Eigen;
using namespace g2o;

namespace g2o {
/**
 * TODO 构建自定义的空间线段顶点
 * 这里是为空间线段创建的顶点
 * 用普吕克坐标表示空间线段，但在优化及更新里面需要转换成正交表示
 * TODO 用正交表示表达线段，pair<so3, so2>？
 * 注意：自定义顶点必须包含以下四个方法（当然构造函数肯定算，就算是五个把）
 * _estimate类型是Vector6d，表示的是普吕克坐标[n, v]^T
 */
class VertexLinePlucker : public BaseVertex<6, Vector6d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexLinePlucker() {}

    // 下面这两个方法暂时也用不着
    virtual bool read(std::istream& is) {}
    virtual bool write(std::ostream& os) const {}

    // 这个方法表示顶点的重置函数，设定被优化变量的原始值
    virtual void setToOriginImpl() {
        _estimate.fill(0.);
    }

    // 这个顶点更新函数很重要，其中update表示的就是增量delta x（额，怎么是double）
    // 优化过程中计算出新的增量后，就是通过这个对增量进行调整
    // 也就是所谓的x = x + delta x
    // 在点特征里面点的空间坐标就只是单纯的加减就行
    // 在线特征里使用了正交表示之后就不一样了，需要转换成正交表示下的增量delta theta
    virtual void oplusImpl(const double* update) {
        // update
        Eigen::Map<const Vector6d> v(update);
        _estimate += v;
    }
};

/**
 * TODO 构建自定义的边，只优化位姿的
 * 这里是为线特征构建的一元边，主要用在仅估计相机位姿的情况
 * 最小化线特征距离误差e = [Ps^T * I / sqrt(l1^2 + l2^2), Pe^T * I / sqrt(l1^2 + l2^2)]T
 * _measurement维度4维，即图像上线段两个端点的二维坐标，因为只优化位姿，所以顶点就是位姿顶点
 * 和顶点一样，自定义边需要实现下面四个重要函数
 */
//    static const int Dimension = D;
//    typedef E Measurement;
//    typedef Matrix<double, D, 1> ErrorVector;
//    typedef Matrix<double, D, D> InformationType;
class  EdgeLineOnlyPose: public  BaseUnaryEdge<2, Vector4d, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeLineOnlyPose(){}

    bool read(std::istream& is) {}

    bool write(std::ostream& os) const {}

    // 误差计算的函数，很重要，顾名思义，就是决定误差是如何计算的
    // 在这里是直线的距离误差e = [Ps^T * I / sqrt(l1^2 + l2^2), Pe^T * I / sqrt(l1^2 + l2^2)]T
    void computeError() {
        // TODO 计算直线距离误差
        // 顶点，这里就一个相机位姿顶点
        const VertexSE3Expmap* v = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        // 观测值是线段端点坐标，[startx, starty, endx, endy]
        Eigen::Vector2d obs_start(_measurement(0), _measurement(1));
        Eigen::Vector2d obs_end(_measurement(2), _measurement(3));
        // 线特征的内参
        Eigen::Matrix<double, 3, 3> K_line;
        K_line << fy_,        0.0,       0.0,
                  0.0,        fx_,       0.0,
                  -fy_ * cx_, -fx_ * cy_, fx_ * fy_;
        // 求nc，通过v->estimate();
        // Rnw + t^Rv;
        Eigen::Matrix<double, 3, 3> Rcw = v->estimate().rotation().toRotationMatrix();
        Eigen::Matrix<double, 3, 3> tcw_hat = ToSkewsymmetricMatrix(v->estimate().translation());
        Eigen::Vector3d nc = Rcw * nw_ + tcw_hat * Rcw * vw_;
        // 求投影直线方程
        Eigen::Vector3d line_coef = K_line * nc;
//        line_coef.normalize();

        double sqrt_l12_l22 = sqrt(pow(line_coef(0), 2) + pow(line_coef(1), 2));

        // _error Eigen::Matrix<2, 1>
        _error(0) = (obs_start(0) * line_coef(0) + obs_start(1) * line_coef(1) + line_coef(2)) / sqrt_l12_l22;
        _error(1) = (obs_end(0) * line_coef(0) + obs_end(1) * line_coef(1) + line_coef(2)) / sqrt_l12_l22;
//        std::cout << "cur error = " << std::endl << _error.matrix() << std::endl;
    }

    // 很重要的函数，误差对优化变量的偏导数，即雅克比矩阵
    // 这个一元边就是直线距离误差对位姿增量的偏导数
    virtual void linearizeOplus() {
        // typename Matrix<double, 2, VertexSE3Expmap::Dimension>::AlignedMapType JacobianXiOplusType
        // Matrix<double, 2, 6>
        // 顶点，这里只有一个，相机位姿
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);

        // 观测值是线段端点坐标，[startx, starty, endx, endy]
        Eigen::Vector2d obs_start(_measurement(0), _measurement(1));
        Eigen::Vector2d obs_end(_measurement(2), _measurement(3));

        // 线特征的内参
        Eigen::Matrix<double, 3, 3> K_line;
        K_line << fy_,        0.0,       0.0,
                0.0,        fx_,       0.0,
                -fy_ * cx_, -fx_ * cy_, fx_ * fy_;
        // 求投影线段方程
        // 求nc，通过v->estimate();
        // Rnw + t^Rv;
        Eigen::Matrix<double, 3, 3> Rcw = vi->estimate().rotation().toRotationMatrix();
        Eigen::Matrix<double, 3, 3> tcw_hat = ToSkewsymmetricMatrix(vi->estimate().translation());
        Eigen::Vector3d nc = Rcw * nw_ + tcw_hat * Rcw * vw_;
        // 求投影直线方程
        Eigen::Vector3d line_coef = K_line * nc;
//        line_coef.normalize();

        // 误差关于投影线段方程的偏导数
        double ln = sqrt(pow(line_coef(0), 2) + pow(line_coef(1), 2));
        double e1 = obs_start(0) * line_coef(0) + obs_start(1) * line_coef(1) + line_coef(2);
        double e2 = obs_end(0) * line_coef(0) + obs_end(1) * line_coef(1) + line_coef(2);

        Eigen::Matrix<double, 2, 3> del_dI;
        del_dI(0, 0) = (obs_start(0) - (line_coef(0) * e1) / (ln * ln)) / ln;
        del_dI(0, 1) = (obs_start(1) - (line_coef(1) * e1) / (ln * ln)) / ln;
        del_dI(0, 2) = 1;

        del_dI(0, 0) = (obs_end(0) - (line_coef(0) * e2) / (ln * ln)) / ln;
        del_dI(0, 1) = (obs_end(1) - (line_coef(1) * e2) / (ln * ln)) / ln;
        del_dI(0, 2) = 1;


        // 投影线段关于相机下普吕克坐标的偏导数
        Eigen::Matrix<double, 3, 6> dI_dLc;
        dI_dLc << fy_,        0.0,       0.0,       0.0, 0.0, 0.0,
                  0.0,        fx_,       0.0,       0.0, 0.0, 0.0,
                  -fy_ * cx_, fx_ * cy_, fx_ * fy_, 0.0, 0.0, 0.0;

        // 相机下普吕克坐标关于位姿增量偏导数
        Eigen::Matrix<double, 6, 6> dLc_ddelta = Eigen::Matrix<double, 6, 6>::Zero();

        dLc_ddelta.block<3, 3>(0, 0) = -1.0 * ToSkewsymmetricMatrix(Rcw * vw_) -
                ToSkewsymmetricMatrix(tcw_hat * Rcw * vw_);
        dLc_ddelta.block<3, 3>(0, 3) = -1.0 * ToSkewsymmetricMatrix(Rcw * vw_);
        dLc_ddelta.block<3, 3>(3, 0) = -1.0 * ToSkewsymmetricMatrix(Rcw * vw_);

        // 最终的雅克比矩阵
        Eigen::Matrix<double, 2, 6> J = del_dI * dI_dLc * dLc_ddelta;
        _jacobianOplusXi(0, 0) = J(0, 0);
        _jacobianOplusXi(0, 1) = J(0, 1);
        _jacobianOplusXi(0, 2) = J(0, 2);
        _jacobianOplusXi(0, 3) = J(0, 3);
        _jacobianOplusXi(0, 4) = J(0, 4);
        _jacobianOplusXi(0, 5) = J(0, 5);

        _jacobianOplusXi(1, 0) = J(1, 0);
        _jacobianOplusXi(1, 1) = J(1, 1);
        _jacobianOplusXi(1, 2) = J(1, 2);
        _jacobianOplusXi(1, 3) = J(1, 3);
        _jacobianOplusXi(1, 4) = J(1, 4);
        _jacobianOplusXi(1, 5) = J(1, 5);

//        std::cout << std::endl;
//        std::cout << _jacobianOplusXi(0, 0) << " , " << _jacobianOplusXi(0, 1) << " , " << _jacobianOplusXi(0, 2) << " , "
//                  << _jacobianOplusXi(0, 3) << " , " << _jacobianOplusXi(0, 4) << " , " << _jacobianOplusXi(0, 5) << std::endl;
//        std::cout << _jacobianOplusXi(1, 0) << " , " << _jacobianOplusXi(1, 1) << " , " << _jacobianOplusXi(1, 2) << " , "
//                  << _jacobianOplusXi(1, 3) << " , " << _jacobianOplusXi(1, 4) << " , " << _jacobianOplusXi(1, 5) << std::endl;
//        std::cout << std::endl;
    }

    Eigen::Matrix3d ToSkewsymmetricMatrix(const Eigen::Vector3d& v) {
        Eigen::Matrix<double, 3, 3> v_hat;
        v_hat << 0.0,         -1.0 * v(2), v(1),
                 v(2),        0.0,         -1.0 * v(0),
                 -1.0 * v(1), v(0),        0.0;
        return v_hat;
    }

    // 空间直线的普吕克坐标，这里把法向量和方向向量分开写
    Vector3d nw_;
    Vector3d vw_;

    double fx_, fy_, cx_, cy_;
};


#if 0
/**
 * TODO 构建自定义的边
 * 这里是为线特征构建的二元边
 * 最小化线特征误差
 * Vertex：待优化当前帧的Tcw
 * Vertex：待优化的空间线段
 * measurement：MapLine在当前帧中的两个端点位置
 * InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *
 */
class EdgeLineProjectXYZ : public BaseBinaryEdge<3, Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeLineProjectXYZ() {}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    virtual void computeError()
    {
        const VertexSE3Expmap* v1 = static_cast<VertexSE3Expmap *>(_vertices[0]);
        const VertexSBAPointXYZ* v2 = static_cast<VertexSBAPointXYZ*>(_vertices[1]);
        //这里是取0还是1，我觉得要看Optimizer.c文件中怎么为误差边添加顶点的.todo 这里是重大bug吗！! 应该是0对应地图点，1对应关键帧位姿
        /** optimizer.cpp 549行附近这样写的，可见误差边上0对应线的端点,1对应的是关键帧位姿，所以作者上面0 1位置应该颠倒了吧！！！
         *
         *  EdgeLineProjectXYZ* e = new EdgeLineProjectXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(ids)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
         */

        Vector3d obs = _measurement;    //线段所在直线参数
        Vector2d proj = cam_project(v1->estimate().map(Xw));
        //todo BUG！误差边的两个顶点都是优化变量，计算误差的时候两个顶点的更新应该都会用到！ 参考types_six_dof_expmap.h第108行！  卧草，怎么处处有bug……
        //要改啊！！
        _error(0) = obs(0) * proj(0) + obs(1)*proj(1) + obs(2); //误差还是1维的
    }

    virtual void linearizeOplus()
    {
        // 位姿顶点
        VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        Vector3d xyz_trans = T.map(Xw);    //线段端点的世界坐标系转换到相机坐标系下  //todo sigh...这里也错了！不会用到Xw,请见types_six_dof_expmap.cpp中的Xw，这个变量只有在OnlyPose误差边中用到，因为空间点的位置是固定的。而这里空间点的位置要用估计 VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ*>(-vertices[0]), Xw = vi->estimate();  (tsde.c源文件112行)

        // 线段端点顶点
//        VertexLinePointXYZ *vj = static_cast<VertexLinePointXYZ *>(_vertices[1]);
        VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
        Vector3d xyz = vi->estimate();

        double x = xyz_trans[0];  //所以这些地方都要改，或者用xyz
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);

        // 3*6 jacobian
        // 1.这是最开始推导雅克比时，有负号的
//        _jacobianOplusXj(0,0) = fy*ly + fx*lx*x*y*invz_2 + fy*ly*y*y*invz_2;
//        _jacobianOplusXj(0,1) = -fx*lx - fx*lx*x*x*invz_2 - fy*ly*x*y*invz_2;
//        _jacobianOplusXj(0,2) = fx*lx*y*invz - fy*ly*x*invz;
//        _jacobianOplusXj(0,3) = -fx*lx*invz;
//        _jacobianOplusXj(0.4) = -fy*ly*invz;
//        _jacobianOplusXj(0,5) = (fx*lx*x+fy*ly*y)*invz_2;


        // 雅克比没有负号的。 注意这里的下标，只有位姿优化的时候是Xi，既有位姿又有地图点，点是Xi，位姿是Xj
        // 这一部分和EdgeLineProjectXYZOnlyPose中是一样的
        _jacobianOplusXj(0,0) = -fy*ly - fx*lx*x*y*invz_2 - fy*ly*y*y*invz_2;
        _jacobianOplusXj(0,1) = fx*lx + fx*lx*x*x*invz_2 + fy*ly*x*y*invz_2;
        _jacobianOplusXj(0,2) = -fx*lx*y*invz + fy*ly*x*invz;
        _jacobianOplusXj(0,3) = fx*lx*invz;
        _jacobianOplusXj(0.4) = fy*ly*invz;
        _jacobianOplusXj(0,5) = -(fx*lx*x+fy*ly*y)*invz_2;

        Matrix<double, 3, 3, Eigen::ColMajor> tmp;  //TODO 这里也有问题吧，不应该是一行三列吗，误差1维空间点3维，见tsde.c中124行
        tmp = Eigen::Matrix3d::Zero();
        tmp(0,0) = fx*lx;
        tmp(0,1) = fy*ly;
        tmp(0,2) = -(fx*lx*x+fy*ly*y)*invz;

        Matrix<double, 3, 3> R;
        R = T.rotation().toRotationMatrix();  //没毛病

//        _jacobianOplusXi = -1. * invz * tmp * R;
        _jacobianOplusXi = 1. * invz * tmp * R;  //这个公式应该没有问题
    }


    Vector2d cam_project(const Vector3d & trans_xyz) const;

    Vector3d Xw;
    double fx, fy, cx, cy;  //相机内参数
};
#endif

}



#endif //ORB_SLAM2_TYPES_LINE_EXPMAP_H
