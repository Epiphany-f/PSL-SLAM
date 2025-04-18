#ifndef ORB_SLAM2_VERTEXLIL_H
#define ORB_SLAM2_VERTEXLIL_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/eigen_types.h"
#include "Thirdparty/g2o/g2o/stuff/misc.h"


namespace g2o{
class VertexLIL : public BaseVertex<3,Matrix<double,15,1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexLIL();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate.fill(0.);
    }

    virtual void oplusImpl(const double* update)
    {
        Eigen::Map<const Matrix<double,15,1> > v(update);
        _estimate += v;
    }
};
}
#endif