/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
//Added _CORE_ in define to remind us it's not libmv full library.
#ifndef LIBMV_CORE_ESSENTIAL_ESTIMATOR_H
#define LIBMV_CORE_ESSENTIAL_ESTIMATOR_H

#include "../numeric/libmv_types.h"
#include "../numeric/vector.h"
#include "../numeric/conditioning.h"
//EssentialFromRt need RelativeCameraMotion, so:
#include "../estimator/projection_estimator.h"

namespace libmv
{
  namespace essential_matrix
  {
    ////////////////   How can we create an essential matrix:  //////////////
    //Using fundamental matrix and camera intra parameters, compute the Essential matrix
    void _LIBMV_DLL_ EssentialFromFundamental(const Mat3 &F,const Mat3 &K1,const Mat3 &K2,Mat3 *E);
    //Using rotation and translation of two camera, compute the Essential matrix
    void _LIBMV_DLL_ EssentialFromRt(const Mat3 &R1,const Vec3 &t1,const Mat3 &R2,const Vec3 &t2,Mat3 *E);

    ///////////////  What can we do with a essential matrix:  //////////////

    //create 4 rotation/translation possibilities extracted from essential matrix
    void _LIBMV_DLL_ MotionFromEssential(const Mat3 &E, vector<Mat3> *Rs, vector<Vec3> *ts);
    // Choose one of the four possible motion solutions from an essential matrix.
    int _LIBMV_DLL_ MotionFromEssentialChooseSolution(const vector<Mat3> &Rs,
      const vector<Vec3> &ts, const Mat3 &K1, const Vec2 &x1,
      const Mat3 &K2, const Vec2 &x2);
    bool _LIBMV_DLL_ MotionFromEssentialAndCorrespondence(const Mat3 &E,
      const Mat3 &K1, const Vec2 &x1,
      const Mat3 &K2, const Vec2 &x2,
      Mat3 *R, Vec3 *t);
    // Computes at most 10 candidate essential matrix solutions from 5 correspondences.
    void _LIBMV_DLL_ FivePointsRelativePose(const Mat2X &x1, const Mat2X &x2, vector<Mat3> *E);
  }
}

#endif