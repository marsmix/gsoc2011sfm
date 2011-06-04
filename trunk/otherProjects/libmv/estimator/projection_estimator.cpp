/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
#include "projection_estimator.h"

namespace libmv
{
  namespace projection_estimation
  {

    void RelativeCameraMotion(const Mat3 &R1, const Vec3 &t1,
      const Mat3 &R2, const Vec3 &t2, Mat3 *R, Vec3 *t)
    {
        *R = R2 * R1.transpose();
        *t = t2 - (*R) * t1;
    }

    template<typename T>
    void NViewTriangulate(const Matrix<T, 2, Dynamic> &x,
      const vector<Matrix<T, 3, 4> > &Ps, Matrix<T, 4, 1> *X)
    {
      int nviews = x.cols();
      assert(nviews == Ps.size());

      Matrix<T, Dynamic, Dynamic> design(3*nviews, 4 + nviews);
      design.setConstant(0.0);
      for (int i = 0; i < nviews; i++) {
        design.template block<3, 4>(3*i, 0) = -Ps[i];
        design(3*i + 0, 4 + i) = x(0, i);
        design(3*i + 1, 4 + i) = x(1, i);
        design(3*i + 2, 4 + i) = 1.0;
      }
      Matrix<T, Dynamic, 1>  X_and_alphas;
      Nullspace(&design, &X_and_alphas);
      X->resize(4);
      *X = X_and_alphas.head(4);
    }

    template<typename T>
    void NViewTriangulateAlgebraic(const Matrix<T, 2, Dynamic> &x,
      const vector<Matrix<T, 3, 4> > &Ps, Matrix<T, 4, 1> *X)
    {
      int nviews = x.cols();
      assert(nviews == Ps.size());

      Matrix<T, Dynamic, 4> design(2*nviews, 4);
      for (int i = 0; i < nviews; i++) {
        design.template block<2, 4>(2*i, 0) = SkewMatMinimal(x.col(i)) * Ps[i];
      }
      X->resize(4);
      Nullspace(&design, X);
    }
    // HZ 12.2 pag.312
    void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
      const Mat34 &P2, const Vec2 &x2, Vec4 *X_homogeneous) {
        Mat4 design;
        for (int i = 0; i < 4; ++i)
        {
          design(0,i) = x1(0) * P1(2,i) - P1(0,i);
          design(1,i) = x1(1) * P1(2,i) - P1(1,i);
          design(2,i) = x2(0) * P2(2,i) - P2(0,i);
          design(3,i) = x2(1) * P2(2,i) - P2(1,i);
        }
        Nullspace(&design, X_homogeneous);
    }

    void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
      const Mat34 &P2, const Vec2 &x2, Vec3 *X_euclidean)
    {
      Vec4 X_homogeneous;
      TriangulateDLT(P1, x1, P2, x2, &X_homogeneous);
      HomogeneousToEuclidean(X_homogeneous, X_euclidean);
    }
  }
}