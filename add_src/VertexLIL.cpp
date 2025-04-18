#include "add_inc/VertexLIL.h"

namespace g2o {
    VertexLIL::VertexLIL(){
    }

    bool VertexLIL::read(std::istream& is) {
        Matrix<double,15,1> lv;
        for (int i=0; i<15; i++)
            is >> _estimate[i];
        return true;
    }

    bool VertexLIL::write(std::ostream& os) const {
        Matrix<double,15,1> lv=estimate();
        for (int i=0; i<15; i++){
            os << lv[i] << " ";
        }
        return os.good();
    }
}