#ifndef INSECTLINEMATCH_H
#define INSECTLINEMATCH_H

#include "insectline.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace ORB_SLAM2 {
    class InsectLineMatch {
    public:
        InsectLineMatch(float dTh = 0.1, float aTh = 0.86);

        int SearchMapInsectline(Frame &pF, const std::vector<InsectLine*> &vpMapInsectline);

    protected:
        float dTh, aTh;
    };
}



#endif