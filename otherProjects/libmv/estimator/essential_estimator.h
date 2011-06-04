/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
//Added _CORE_ in define to remind us it's not libmv full library.
#ifndef LIBMV_CORE_ESSENTIAL_ESTIMATOR_H
#define LIBMV_CORE_ESSENTIAL_ESTIMATOR_H

#include "../numeric/libmv_types.h"
#include "../numeric/conditioning.h"
//EssentialFromRt need RelativeCameraMotion, so:
#include "../estimator/projection_estimator.h"

#include <vector>

namespace libmv
{
  namespace essential_matrix
  {
    ////////////////   How can we create an essential matrix:  //////////////
    //Using fundamental matrix and camera intra parameters, compute the Essential matrix
    void EssentialFromFundamental(const Mat3 &F,const Mat3 &K1,const Mat3 &K2,Mat3 *E);
    //Using rotation and translation of two camera, compute the Essential matrix
    void EssentialFromRt(const Mat3 &R1,const Vec3 &t1,const Mat3 &R2,const Vec3 &t2,Mat3 *E);

    ///////////////  What can we do with a essential matrix:  //////////////

    //create 4 rotation/translation possibilities extracted from essential matrix
    void MotionFromEssential(const Mat3 &E, std::vector<Mat3> *Rs, std::vector<Vec3> *ts);
    // Choose one of the four possible motion solutions from an essential matrix.
    int MotionFromEssentialChooseSolution(...);
    int MotionFromEssentialAndCorrespondence(...);
    // Computes at most 10 candidate essential matrix solutions from 5 correspondences.
    void FivePointsRelativePose(const Mat2X &x1, const Mat2X &x2, std::vector<Mat3> *E);
  }
}

#endif