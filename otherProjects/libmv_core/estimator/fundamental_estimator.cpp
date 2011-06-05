/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
#include "fundamental_estimator.h"

namespace libmv
{
  namespace fundamental_matrix
  {
    //function used by FundamentalFromProjections
    void EliminateRow(const Mat34 &P, int row, Mat *X)
    {
      X->resize(2,4);
      int first_row = (row + 1) % 3;
      int second_row = (row + 2) % 3;
      for (int i = 0; i < 4; ++i)
      {
        (*X)(0, i) = P(first_row, i);
        (*X)(1, i) = P(second_row, i);
      }
    }

    // Addapted from vgg_F_from_P.
    void FundamentalFromProjections(const Mat34 &P1, const Mat34 &P2, Mat3 *F)
    {
      Mat X[3], Y[3], XY;

      for (int i = 0; i < 3; ++i) {
        EliminateRow(P1, i, X + i);
        EliminateRow(P2, i, Y + i);
      }

      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          VerticalStack(X[j], Y[i], &XY);
          (*F)(i, j) = XY.determinant();
        }
      }
    }

    // HZ 9.6 pag 257 (formula 9.12)
    // Or http://ai.stanford.edu/~birch/projective/node20.html
    void FundamentalFromEssential(const Mat3 &E,
      const Mat3 &K1,const Mat3 &K2,Mat3 *F)
    {
        *F = K2.inverse().transpose() * E * K1.inverse();
    }

    // HZ 11.1 pag.279 (x1 = x, x2 = x')
    // http://www.cs.unc.edu/~marc/tutorial/node54.html
    double EightPointSolver(const Mat &x1, const Mat &x2, Mat3 *F)
    {
      assert(2 == x1.rows());
      assert(8 <= x1.cols());
      assert(x1.rows() == x2.rows());
      assert(x1.cols() == x2.cols());

      int n = x1.cols();
      Mat A(n, 9);
      for (int i = 0; i < n; ++i)
      {
        A(i, 0) = x2(0, i) * x1(0, i);
        A(i, 1) = x2(0, i) * x1(1, i);
        A(i, 2) = x2(0, i);
        A(i, 3) = x2(1, i) * x1(0, i);
        A(i, 4) = x2(1, i) * x1(1, i);
        A(i, 5) = x2(1, i);
        A(i, 6) = x1(0, i);
        A(i, 7) = x1(1, i);
        A(i, 8) = 1;
      }

      Vec9 f;
      double smaller_singular_value = Nullspace(&A, &f);
      *F = Map<RMat3>(f.data());
      return smaller_singular_value;
    }

    // HZ 11.2 pag.281 (x1 = x, x2 = x')
    double NormalizedEightPointSolver(const Mat &x1,
      const Mat &x2,Mat3 *F)
    {
        assert(2 == x1.rows());
        assert(8 <= x1.cols());
        assert(x1.rows() == x2.rows());
        assert(x1.cols() == x2.cols());

        // Normalize the data.
        Mat3 T1, T2;
        PreconditionerFromPoints(x1, &T1);
        PreconditionerFromPoints(x2, &T2);
        Mat x1_normalized, x2_normalized;
        ApplyTransformationToPoints(x1, T1, &x1_normalized);
        ApplyTransformationToPoints(x2, T2, &x2_normalized);

        // Estimate the fundamental matrix.
        double smaller_singular_value =
          EightPointSolver(x1_normalized, x2_normalized, F);
        EnforceFundamentalRank2Constraint(F);

        // Denormalize the fundamental matrix.
        *F = T2.transpose() * (*F) * T1;

        return smaller_singular_value;
    }

    // Seven-point algorithm.
    // http://www.cs.unc.edu/~marc/tutorial/node55.html
    double FundamentalFrom7CorrespondencesLinear(const Mat &x1,
      const Mat &x2, std::vector<Mat3> *F)
    {
        assert(2 == x1.rows());
        assert(7 == x1.cols());
        assert(x1.rows() == x2.rows());
        assert(x1.cols() == x2.cols());

        // Build a 9 x n matrix from point matches, where each row is equivalent to
        // the equation x'T*F*x = 0 for a single correspondence pair (x', x). The
        // domain of the matrix is a 9 element vector corresponding to F. The
        // nullspace should be rank two; the two dimensions correspond to the set of
        // F matrices satisfying the epipolar geometry.
        Matrix<double, 7, 9> A;
        for (int ii = 0; ii < 7; ++ii) {
          A(ii, 0) = x1(0, ii) * x2(0, ii);  // 0 represents x coords,
          A(ii, 1) = x1(1, ii) * x2(0, ii);  // 1 represents y coords.
          A(ii, 2) = x2(0, ii);
          A(ii, 3) = x1(0, ii) * x2(1, ii);
          A(ii, 4) = x1(1, ii) * x2(1, ii);
          A(ii, 5) = x2(1, ii);
          A(ii, 6) = x1(0, ii);
          A(ii, 7) = x1(1, ii);
          A(ii, 8) = 1.0;
        }

        // Find the two F matrices in the nullspace of A.
        Vec9 f1, f2;
        double s = Nullspace2(&A, &f1, &f2);
        Mat3 F1 = Map<RMat3>(f1.data());
        Mat3 F2 = Map<RMat3>(f2.data());

        // Then, use the condition det(F) = 0 to determine F. In other words, solve
        // det(F1 + a*F2) = 0 for a.
        double a = F1(0, 0), j = F2(0, 0),
          b = F1(0, 1), k = F2(0, 1),
          c = F1(0, 2), l = F2(0, 2),
          d = F1(1, 0), m = F2(1, 0),
          e = F1(1, 1), n = F2(1, 1),
          f = F1(1, 2), o = F2(1, 2),
          g = F1(2, 0), p = F2(2, 0),
          h = F1(2, 1), q = F2(2, 1),
          i = F1(2, 2), r = F2(2, 2);

        // Run fundamental_7point_coeffs.py to get the below coefficients.
        // The coefficients are in ascending powers of alpha, i.e. P[N]*x^N.
        double P[4] = {
          a*e*i + b*f*g + c*d*h - a*f*h - b*d*i - c*e*g,
          a*e*r + a*i*n + b*f*p + b*g*o + c*d*q + c*h*m + d*h*l + e*i*j + f*g*k -
          a*f*q - a*h*o - b*d*r - b*i*m - c*e*p - c*g*n - d*i*k - e*g*l - f*h*j,
          a*n*r + b*o*p + c*m*q + d*l*q + e*j*r + f*k*p + g*k*o + h*l*m + i*j*n -
          a*o*q - b*m*r - c*n*p - d*k*r - e*l*p - f*j*q - g*l*n - h*j*o - i*k*m,
          j*n*r + k*o*p + l*m*q - j*o*q - k*m*r - l*n*p,
        };

        // Solve for the roots of P[3]*x^3 + P[2]*x^2 + P[1]*x + P[0] = 0.
        double roots[3];
        int num_roots = SolveCubicPolynomial(P, roots);

        // Build the fundamental matrix for each solution.
        for (int kk = 0; kk < num_roots; ++kk)  {
          F->push_back(F1 + roots[kk] * F2);
        }
        return s;
    }

    double FundamentalFromCorrespondences7Point(const Mat &x1,
      const Mat &x2, std::vector<Mat3> *F)
    {
        assert(2 == x1.rows());
        assert(7 <= x1.cols());
        assert(x1.rows() == x2.rows());
        assert(x1.cols() == x2.cols());

        // Normalize the data.
        Mat3 T1, T2;
        PreconditionerFromPoints(x1, &T1);
        PreconditionerFromPoints(x2, &T2);
        Mat x1_normalized, x2_normalized;
        ApplyTransformationToPoints(x1, T1, &x1_normalized);
        ApplyTransformationToPoints(x2, T2, &x2_normalized);

        // Estimate the fundamental matrix.
        double smaller_singular_value =
          FundamentalFrom7CorrespondencesLinear(x1_normalized, x2_normalized, &(*F));

        for(unsigned int k=0; k < F->size(); ++k) {
          Mat3 & Fmat = (*F)[k];
          // Denormalize the fundamental matrix.
          Fmat = T2.transpose() * Fmat * T1;
        }
        return smaller_singular_value;
    };

    //////////////////////////////////////////////////////////////////////////
    ///////////////  What can we do with a fundamental matrix:  //////////////

    // HZ 11.1.1 pag.280
    void EnforceFundamentalRank2Constraint(Mat3 *F)
    {
      Eigen::JacobiSVD<Mat3> USV(*F);
      Vec3 d = USV.singularValues();
      d(2) = 0.0;
      *F = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();
    };

    void NormalizeFundamental(const Mat3 &F, Mat3 *F_normalized)
    {
      *F_normalized = F / FrobeniusNorm(F);
      if((*F_normalized)(2,2) < 0) {
        *F_normalized *= -1;
      }
    }
    //Create two projection matrix from fundamental matrix:
    void ProjectionsFromFundamental(const Mat3 &F, Mat34 *P1, Mat34 *P2)
    {
      *P1 << Mat3::Identity(), Vec3::Zero();
      Vec3 e2;
      Mat3 Ft = F.transpose();
      Nullspace(&Ft, &e2);
      *P2 << CrossProductMatrix(e2) * F, e2;
    }

    // Approximation of reprojection error; page 287 of HZ equation 11.9. This
    // avoids triangulating the point, relying only on the entries in F.
    double SampsonDistance2(const Mat &F, const Vec2 &x1, const Vec2 &x2)
    {
      Vec3 x(x1(0), x1(1), 1.0);
      Vec3 y(x2(0), x2(1), 1.0);

      Vec3 F_x = F * x;
      Vec3 Ft_y = F.transpose() * y;
      double y_F_x = y.dot(F_x);

      return Square(y_F_x) / (  F_x.head<2>().squaredNorm()
        + Ft_y.head<2>().squaredNorm());
    }
    //compute the position of the two epipoles
    void EpipolesFromFundamental(const Mat3 &F, Vec3 *e1, Vec3 *e2)
    {
      Mat3 Fp = F;
      double s1 = Nullspace(&Fp, e1);  // Left nullspace.
      Fp = F;
      TransposeInPlace(&Fp);
      double s2 = Nullspace(&Fp, e2);  // Rigth nullspace.
      // TODO(keir): Check that s1 and s2 are small.
      (void) s1;
      (void) s2;
    }


    // Make a transformation that forces the second component of x to zero.
    // sx + cy = 0              s = -y / r
    // cx - sy > 0      ===>    c =  x / r
    // s^2 + c^2 = 1            r = |(x, y)|
    void RotationToEliminateY(const Vec3 &x, Mat3 *T)
    {
      double r = sqrt(Square(x(0)) + Square(x(1)));
      double c =  x(0) / r;
      double s = -x(1) / r;
      *T << c, -s, 0,
        s,  c, 0,
        0,  0, 1;
    }
    // Rotate each image to cause the y component of both epipoles to become zero.
    // When this happens, the fundamental matrix takes on a special form.
    // 
    // In the original image, the fundamental property is x2'Fx1 = 0 for all x1 and
    // x2 that are corresponding scene points. Transforming the image we have
    //
    //   (T2x2)' F_rotated (T1x1) = 0.
    //
    // Thus, F_rotated = T2 F T1'.
    void FundamentalAlignEpipolesToXAxis(const Mat3 &F, Mat3 *F_rotated)
    {
      Vec3 e1, e2;
      EpipolesFromFundamental(F, &e1, &e2);
      Mat3 T1, T2, T2_F;
      RotationToEliminateY(e1, &T1);
      RotationToEliminateY(e2, &T2);
      T2_F = T2 * F;
      *F_rotated = T2 * F * T1.transpose();
    }
    // Given a fundamental matrix of two cameras and their principal points,
    // computes the fundamental matrix corresponding to the same cameras with the
    // principal points shifted to a new location.
    void FundamentalShiftPrincipalPoints(const Mat3 &F,
      const Vec2 &p1,const Vec2 &p1_new, const Vec2 &p2,
      const Vec2 &p2_new,  Mat3 *F_new)
    {
        Mat3 T1, T2;
        T1 << 1, 0, p1_new(0) - p1(0),
          0, 1, p1_new(1) - p1(1),
          0, 0,                 1;
        T2 << 1, 0, p2_new(0) - p2(0),
          0, 1, p2_new(1) - p2(1),
          0, 0,                 1;
        *F_new = T2.inverse().transpose() * F * T1.inverse();
    }
    // Find focal from fundamental using Hartleys method.
    void FocalFromFundamental(const Mat3 &F,
      const Vec2 &principal_point1, const Vec2 &principal_point2,
      double *f1, double *f2)
    {
        Mat3 F_shifted, F_rotated;
        Vec2 zero2;
        zero2 << 0, 0;
        FundamentalShiftPrincipalPoints(F,
          principal_point1, zero2,
          principal_point2, zero2,
          &F_shifted);

        FundamentalAlignEpipolesToXAxis(F_shifted, &F_rotated);

        Vec3 e1, e2;
        EpipolesFromFundamental(F_rotated, &e1, &e2);

        Mat3 T1, T2;
        T1 << 1 / e1(2), 0,          0,
          0, 1,          0,
          0, 0, -1 / e1(0);
        T2 << 1 / e2(2), 0,          0,
          0, 1,          0,
          0, 0, -1 / e2(0);

        Mat3 A = T2 * F_rotated * T1;

        double a = A(0,0);
        double b = A(0,1);
        double c = A(1,0);
        double d = A(1,1);

        // TODO(pau) Should check we are not dividing by 0.
        double f1_square = - (a * c * Square(e1(0)))
          / (a * c * Square(e1(2)) + b * d);
        double f2_square = - (a * b * Square(e2(0)))
          / (a * b * Square(e2(2)) + c * d);

        // TODO(keir) deterimne a sensible thing to do in this case.
        assert(f1_square > 0.);
        assert(f2_square > 0.);
        *f1 = sqrt(f1_square);
        *f2 = sqrt(f2_square);
    }

    // Find focal from fundamental using Golden ratio search on reproj error on x1 and x2
    void FocalFromFundamentalExhaustive(const Mat3 &F,
      const Vec2 &principal_point, const Mat2X &x1, const Mat2X &x2,
      double min_focal, double max_focal, int n_samples,double *focal)
    {
      FocalReprojectionError error(F, principal_point, x1, x2);

        // Exhaustive search.
        int best_focal = 0;
        double best_error = std::numeric_limits<double>::max();

        for (int i = 0; i < n_samples; ++i) {
          double f = Lerp(i, 0, min_focal, n_samples - 1, max_focal);
          double e = error(f);

          if (e < best_error) {
            best_error = e;
            best_focal = i;
          }
          VLOG(3) << "focal: " << f << "  error: " << e << "\n";
        }  

        // Golden ration search.
        double a = Lerp(best_focal - 1, 0, min_focal, n_samples - 1, max_focal);
        double b = Lerp(best_focal + 1, 0, min_focal, n_samples - 1, max_focal);

        *focal = GoldenRatioSearch(error, a, b, 1e-8, 99999);
    }

    double FocalReprojectionError::operator()(double focal) const {
      Mat3 K;
      K << focal,      0, principal_point_(0),
        0, focal, principal_point_(1),
        0,     0,                  1;

      Mat3 E;
      essential_matrix::EssentialFromFundamental(F_, K, K, &E);

      Mat3 R;
      Vec3 t;
      essential_matrix::MotionFromEssentialAndCorrespondence(E, K,
        x1_.col(0), K, x2_.col(0), &R, &t);

      libmv::vector<Mat34> Ps(2);
      P_From_KRt(K, Mat3::Identity(), Vec3::Zero(), &Ps[0]);
      P_From_KRt(K, R, t, &Ps[1]);

      double error = 0;
      for (int j = 0; j < x1_.cols(); ++j) {
        Vec4 X;
        Mat2X x(2,2);
        x.col(0) = x1_.col(j);
        x.col(1) = x2_.col(j);
        projection_estimation::NViewTriangulate(x, Ps, &X);
        Vec3 x1_reproj = Ps[0] * X;
        Vec3 x2_reproj = Ps[1] * X;
        double threshold = 1.;
        double d1 = Depth(Mat3::Identity(), Vec3::Zero(), X);
        double d2 = Depth(R, t, X);
        if (d1 < 0 || d2 < 0) {
          error += 2 * Square(threshold);
        } else {
          error += std::min(Square(threshold),
            (x1_.col(j) - HomogeneousToEuclidean(x1_reproj)).squaredNorm());
          error += std::min(Square(threshold),
            (x2_.col(j) - HomogeneousToEuclidean(x2_reproj)).squaredNorm());
        }
      }
      return error;
    }
  }
}