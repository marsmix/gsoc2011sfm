/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
//Added _CORE_ in define to remind us it's not libmv full library.
#ifndef LIBMV_CORE_PROJECTION_ESTIMATOR_H
#define LIBMV_CORE_PROJECTION_ESTIMATOR_H

#include "../numeric/libmv_types.h"
#include "../numeric/vector.h"
#include "../numeric/projection.h"
#include <vector>

namespace libmv
{
  namespace projection_estimation
  {

    //compute the relative motion between two cameras
    void RelativeCameraMotion(const Mat3 &R1, const Vec3 &t1,
      const Mat3 &R2, const Vec3 &t2, Mat3 *R, Vec3 *t);

    ///////////////  How can we triangulate points:  ////////////////
    // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
    // output, X, is a homogeneous four vectors.
    // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
    // output, X, is a homogeneous four vectors.
    template<typename T>
    void NViewTriangulate(const Matrix<T, 2, Dynamic> &x,
      const vector<Matrix<T, 3, 4> > &Ps, Matrix<T, 4, 1> *X);

    // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
    // output, X, is a homogeneous four vectors.
    // This method uses the algebraic distance approximation.
    // Note that this method works better when the 2D points are normalized
    // with an isotopic normalization.
    template<typename T>
    void NViewTriangulateAlgebraic(const Matrix<T, 2, Dynamic> &x,
      const vector<Matrix<T, 3, 4> > &Ps, Matrix<T, 4, 1> *X);
    //using two cameras and two 2D points, compute the 3D corresponding point:
    void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
      const Mat34 &P2, const Vec2 &x2, Vec4 *X_homogeneous);
    //Like previous but create 3D euclidean point:
    void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
      const Mat34 &P2, const Vec2 &x2, Vec3 *X_euclidean);
  }
}

#endif