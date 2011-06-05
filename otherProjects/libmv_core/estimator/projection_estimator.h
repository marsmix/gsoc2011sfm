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
    void _LIBMV_DLL_ RelativeCameraMotion(const Mat3 &R1, const Vec3 &t1,
      const Mat3 &R2, const Vec3 &t2, Mat3 *R, Vec3 *t);

    ///////////////  How can we triangulate points:  ////////////////
    // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
    // output, X, is a homogeneous four vectors.
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

    // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
    // output, X, is a homogeneous four vectors.
    // This method uses the algebraic distance approximation.
    // Note that this method works better when the 2D points are normalized
    // with an isotopic normalization.
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
    //using two cameras and two 2D points, compute the 3D corresponding point:
    void _LIBMV_DLL_ TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
      const Mat34 &P2, const Vec2 &x2, Vec4 *X_homogeneous);
    //Like previous but create 3D euclidean point:
    void _LIBMV_DLL_ TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
      const Mat34 &P2, const Vec2 &x2, Vec3 *X_euclidean);

    void _LIBMV_DLL_ TwoViewTriangulationByPlanes(const Vec3 &x1, const Vec3 &x2,
      const Mat34 &P, const Mat3 &E, Vec4 *X);

    void _LIBMV_DLL_ TwoViewTriangulationByPlanes(const Vec2 &x1, const Vec2 &x2,
      const Mat34 &P,const Mat3 &E, Vec3 *X);

    void _LIBMV_DLL_ TwoViewTriangulationIdeal(const Vec3 &x1, const Vec3 &x2,
      const Mat34 &P, const Mat3 &E, Vec4 *X);

    void _LIBMV_DLL_ TwoViewTriangulationIdeal(const Vec2 &x1, const Vec2 &x2,
      const Mat34 &P, const Mat3 &E, Vec3 *X);
  
    /**
    * Compute the projection matrix from a set of 3D points X and their
    * is is useful if a point cloud is reconstructed.
    * Agorithm is the standard DLT as described in Hartley & Zisserman, page 179.
    */
    template<typename T>
    void Resection(const Matrix<T, 2, Dynamic> &x,
      const Matrix<T, 4, Dynamic> &X,
      Matrix<T, 3, 4> *P) {
        int N = x.cols();
        assert(X.cols() == N);

        Matrix<T, Dynamic, 12> design(2*N, 12);
        design.setZero();
        for (int i = 0; i < N; i++) {
          T xi = x(0, i);
          T yi = x(1, i);
          // See equation (7.2) on page 179 of H&Z.
          design.template block<1,4>(2*i,     4) =    -X.col(i).transpose();
          design.template block<1,4>(2*i,     8) =  yi*X.col(i).transpose();
          design.template block<1,4>(2*i + 1, 0) =     X.col(i).transpose();
          design.template block<1,4>(2*i + 1, 8) = -xi*X.col(i).transpose();
        }
        Matrix<T, 12, 1> p;
        Nullspace(&design, &p);
        reshape(p, 3, 4, P);
    }

    enum eLibmvResectionMethod
    {
      eRESECTION_ANSAR_DANIILIDIS,
      eRESECTION_EPNP,
    };

    /**
    * Computes the extrinsic parameters, R and t for a calibrated camera
    * from 4 or more 3D points and their images.
    *
    * \param x_camera          Image points in normalized camera coordinates
    *                          e.g. x_camera=inv(K)*x_image
    * \param X_world           3D points in the world coordinate system
    * \param R                 Solution for the camera rotation matrix
    * \param t                 Solution for the camera translation vector
    * \param eResectionMethod  Resection method
    */
    void _LIBMV_DLL_ EuclideanResection(const Mat2X &x_camera, 
      const Mat3X &X_world,
      Mat3 *R, Vec3 *t,
      eLibmvResectionMethod eResectionMethod = eRESECTION_EPNP );

    /**
    * Computes the extrinsic parameters, R and t for a calibrated camera
    * from 4 or more 3D points and their images.
    *
    * \param x_image           Image points in normalized camera coordinates
    * \param X_world           3D points in the world coordinate system
    * \param K                 Intrinsic parameters camera matrix
    * \param R                 Solution for the camera rotation matrix
    * \param t                 Solution for the camera translation vector
    * \param eResectionMethod  Resection method
    */
    void _LIBMV_DLL_ EuclideanResection(const Mat &x_image, 
      const Mat3X &X_world,
      const Mat3 &K, Mat3 *R, Vec3 *t,
      eLibmvResectionMethod eResectionMethod = eRESECTION_EPNP );

    /**
    * The absolute orientation algorithm recovers the transformation
    * between a set of 3D points, X and Xp such that:
    *
    *           Xp = R*X + t
    *
    * The recovery of the absolute orientation is implemented after this
    * article: Horn, Hilden, "Closed-form solution of absolute
    * orientation using orthonormal matrices"
    */
    void _LIBMV_DLL_ AbsoluteOrientation(const Mat3X &X,
      const Mat3X &Xp, Mat3 *R, Vec3 *t);

    /**
    * Computes the extrinsic parameters, R and t for a calibrated camera
    * from 4 or more 3D points and their images.
    *
    * \param x_camera Image points in normalized camera coordinates,
    *       e.g. x_camera=inv(K)*x_image
    * \param X_world 3D points in the world coordinate system
    * \param R       Solution for the camera rotation matrix
    * \param t       Solution for the camera translation vector
    *
    * This is the algorithm described in:
    * "Linear Pose Estimation from Points or Lines", by Ansar, A. and
    *  Daniilidis, PAMI 2003. vol. 25, no. 5
    */
    void _LIBMV_DLL_ EuclideanResectionAnsarDaniilidis(const Mat2X &x_camera, 
      const Mat3X &X_world, Mat3 *R, Vec3 *t);
    /**
    * Computes the extrinsic parameters, R and t for a calibrated camera
    * from 4 or more 3D points and their images.
    *
    * \param x_camera Image points in normalized camera coordinates,
    *       e.g. x_camera=inv(K)*x_image
    * \param X_world 3D points in the world coordinate system
    * \param R       Solution for the camera rotation matrix
    * \param t       Solution for the camera translation vector
    *
    * This is the algorithm described in:
    * "{EP$n$P: An Accurate $O(n)$ Solution to the P$n$P Problem", by V. Lepetit
    * and F. Moreno-Noguer and P. Fua, IJCV 2009. vol. 81, no. 2
    * \note: the non-linear optimization is not implemented here.
    */
    void _LIBMV_DLL_ EuclideanResectionEPnP(const Mat2X &x_camera, const Mat3X &X_world, 
      Mat3 *R, Vec3 *t);
  }
}

#endif