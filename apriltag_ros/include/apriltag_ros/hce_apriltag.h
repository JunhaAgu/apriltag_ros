#ifndef _HCE_APRILTAG_H_
#define _HCE_APRILTAG_H_

namespace INTRINSIC{

    const double Intrinsic_fx = 893.4800446716952;
    const double Intrinsic_fy = 893.2138912525431;
    const double Intrinsic_cx = 529.8848649123668;
    const double Intrinsic_cy = 395.1783894490890;

    const double Intrinsic_distCoeffs_k1 = 0.155682618969427; //Radial Distortion  
    const double Intrinsic_distCoeffs_k2 = 0.118825328777770; //Radial Distortion  
    const double Intrinsic_distCoeffs_p1 = 5.831317355223568e-05; //Tangential Distortion
    const double Intrinsic_distCoeffs_p2 = 0.001254306950798; //Tangential Distortion

}

#endif
