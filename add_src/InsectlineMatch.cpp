#include "InsectlineMatch.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace ORB_SLAM2 {
    InsectLineMatch::InsectLineMatch(float dTh, float aTh):dTh(dTh), aTh(aTh) {}
    int InsectLineMatch::SearchMapInsectline(Frame &pF, const std::vector<InsectLine *> &vpMapInsectline) {
        pF.mbNewPlane = false;

        int nmatches = 0;

        for(int i = 0; i < pF.N_LJL; ++i) {
            Mat pM = pF.ComputeWorldPlane(i);

            int nlinei = pF.mvPlaneLineNo[i].first;
            int nlinej = pF.mvPlaneLineNo[i].second;

            Vector3d p3Dis = pF.mvLines3D[nlinei].first;
            Vector3d p3Die = pF.mvLines3D[nlinei].second;
            Vector3d p3Djs = pF.mvLines3D[nlinej].first;
            Vector3d p3Dje = pF.mvLines3D[nlinej].second;
            Vector3d intersection_point = pF.CrossPoint_3D[i];
            float ldTh = dTh;

            bool found = false;
            for(auto vpMapInsecs:vpMapInsectline) {
                if(vpMapInsecs->isBad())
                    continue;
                Mat pW = vpMapInsecs->GetWorldPos_plane();

                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);
                // associate plane
                if ((angle > aTh || angle < -aTh))
                {
                    float dis = pM.at<float>(3, 0) - pW.at<float>(3, 0);
                    float disis = (pW.at<float>(0, 0) * p3Dis.x() + pW.at<float>(1, 0) * p3Dis.y() + pW.at<float>(2, 0) * p3Dis.z() + pW.at<float>(3, 0));
                    float disie = (pW.at<float>(0, 0) * p3Die.x() + pW.at<float>(1, 0) * p3Die.y() + pW.at<float>(2, 0) * p3Die.z() + pW.at<float>(3, 0));
                    float disjs = (pW.at<float>(0, 0) * p3Djs.x() + pW.at<float>(1, 0) * p3Djs.y() + pW.at<float>(2, 0) * p3Djs.z() + pW.at<float>(3, 0));
                    float disje = (pW.at<float>(0, 0) * p3Dje.x() + pW.at<float>(1, 0) * p3Dje.y() + pW.at<float>(2, 0) * p3Dje.z() + pW.at<float>(3, 0));
                    float disins = (pW.at<float>(0, 0) * intersection_point.x() + pW.at<float>(1, 0) * intersection_point.y() + pW.at<float>(2, 0) * intersection_point.z() + pW.at<float>(3, 0));
                    dis = (disis + disie + disjs + disje + disins)/5;
                    if(std::abs(dis) < ldTh) {
                        ldTh = dis;
                        pF.mvpMapInsecs[i] = static_cast<InsectLine*>(nullptr);
                        pF.mvpMapInsecs[i] = vpMapInsecs;
                        found = true;
                        continue;
                    }
                }
            }
            if (found) {
                nmatches++;
            }
        }
        return nmatches;
    }

}