//Use some libmv's code algorithms as the core lib is not yet defined.
//Once the lib is out the functions will be removed!

//TrackPoints::triangulate is libmv::NViewTriangulate

// Copyright (c) 2009 libmv authors.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// 
// Copyright (c) 2007, 2008 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//
// Matrix and vector classes, based on Eigen2.
//
// Avoid using Eigen2 classes directly; instead typedef them here.

#ifndef LIBMV_NUMERIC_NUMERIC_H
#define LIBMV_NUMERIC_NUMERIC_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

#if _WIN32 || __APPLE__
void static sincos (double x, double *sinx, double *cosx) {
  *sinx = sin(x);
  *cosx = cos(x);
}
#endif //_WIN32 || __APPLE__

#if _WIN32
inline long lround(double d) {
  return (long)(d>0 ? d+0.5 : ceil(d-0.5));
}
inline int round(double d) {
  return (d>0) ? int(d+0.5) : int(d-0.5);
}
typedef unsigned int uint;
#endif //_WIN32

namespace libmv {

  typedef Eigen::MatrixXd Mat;
  typedef Eigen::VectorXd Vec;

  typedef Eigen::MatrixXf Matf;
  typedef Eigen::VectorXf Vecf;

  typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> Matu;
  typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> Vecu;
  typedef Eigen::Matrix<unsigned int, 2, 1> Vec2u;

  typedef Eigen::Matrix<double, 2, 2> Mat2;
  typedef Eigen::Matrix<double, 2, 3> Mat23;
  typedef Eigen::Matrix<double, 3, 3> Mat3;
  typedef Eigen::Matrix<double, 3, 4> Mat34;
  typedef Eigen::Matrix<double, 3, 5> Mat35;
  typedef Eigen::Matrix<double, 4, 1> Mat41;
  typedef Eigen::Matrix<double, 4, 3> Mat43;
  typedef Eigen::Matrix<double, 4, 4> Mat4;
  typedef Eigen::Matrix<double, 4, 6> Mat46;
  typedef Eigen::Matrix<float, 2, 2> Mat2f;
  typedef Eigen::Matrix<float, 2, 3> Mat23f;
  typedef Eigen::Matrix<float, 3, 3> Mat3f;
  typedef Eigen::Matrix<float, 3, 4> Mat34f;
  typedef Eigen::Matrix<float, 3, 5> Mat35f;
  typedef Eigen::Matrix<float, 4, 3> Mat43f;
  typedef Eigen::Matrix<float, 4, 4> Mat4f;
  typedef Eigen::Matrix<float, 4, 6> Mat46f;

  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RMat3;
  typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> RMat4;

  typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
  typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
  typedef Eigen::Matrix<double, 4, Eigen::Dynamic> Mat4X;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 2> MatX2;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatX3;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 4> MatX4;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 5> MatX5;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatX6;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 7> MatX7;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 8> MatX8;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 9> MatX9;
  typedef Eigen::Matrix<double, Eigen::Dynamic,15> MatX15;
  typedef Eigen::Matrix<double, Eigen::Dynamic,16> MatX16;

  typedef Eigen::Vector2d Vec2;
  typedef Eigen::Vector3d Vec3;
  typedef Eigen::Vector4d Vec4;
  typedef Eigen::Matrix<double, 5, 1>  Vec5;
  typedef Eigen::Matrix<double, 6, 1>  Vec6;
  typedef Eigen::Matrix<double, 7, 1>  Vec7;
  typedef Eigen::Matrix<double, 8, 1>  Vec8;
  typedef Eigen::Matrix<double, 9, 1>  Vec9;
  typedef Eigen::Matrix<double, 10, 1> Vec10;
  typedef Eigen::Matrix<double, 11, 1> Vec11;
  typedef Eigen::Matrix<double, 12, 1> Vec12;
  typedef Eigen::Matrix<double, 13, 1> Vec13;
  typedef Eigen::Matrix<double, 14, 1> Vec14;
  typedef Eigen::Matrix<double, 15, 1> Vec15;
  typedef Eigen::Matrix<double, 16, 1> Vec16;
  typedef Eigen::Matrix<double, 17, 1> Vec17;
  typedef Eigen::Matrix<double, 18, 1> Vec18;
  typedef Eigen::Matrix<double, 19, 1> Vec19;
  typedef Eigen::Matrix<double, 20, 1> Vec20;

  typedef Eigen::Vector2f Vec2f;
  typedef Eigen::Vector3f Vec3f;
  typedef Eigen::Vector4f Vec4f;

  typedef Eigen::VectorXi VecXi;

  typedef Eigen::Vector2i Vec2i;
  typedef Eigen::Vector3i Vec3i;
  typedef Eigen::Vector4i Vec4i;

  typedef Eigen::Matrix<float,
    Eigen::Dynamic,
    Eigen::Dynamic,
    Eigen::RowMajor> RMatf;

  typedef Eigen::NumTraits<double> EigenDouble;
  
  // Solve the linear system Ax = 0 via SVD. Store the solution in x, such that
  // ||x|| = 1.0. Return the singluar value corresponding to the solution.
  // Destroys A and resizes x if necessary.
  // TODO(maclean): Take the SVD of the transpose instead of this zero padding.
  template <typename TMat, typename TVec>
  double Nullspace(TMat *A, TVec *nullspace) {
    Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
    (*nullspace) = svd.matrixV().col(A->cols()-1);
    if (A->rows() >= A->cols())
      return svd.singularValues()(A->cols()-1);
    else
      return 0.0;
  }

  // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
  // output, X, is a homogeneous four vectors.
  template<typename T>
  void NViewTriangulate(const Matrix<T, 2, Dynamic> &x,
    const vector<Matrix<T, 3, 4> > &Ps,
    Matrix<T, 4, 1> *X) {
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


}

#endif