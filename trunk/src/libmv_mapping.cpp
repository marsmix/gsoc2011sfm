#include "libmv_mapping.h"



namespace libmv {


  // HZ 12.2 pag.312
  void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
    const Mat34 &P2, const Vec2 &x2,
    Vec4 *X_homogeneous) {
      Mat4 design;
      for (int i = 0; i < 4; ++i) {
        design(0,i) = x1(0) * P1(2,i) - P1(0,i);
        design(1,i) = x1(1) * P1(2,i) - P1(1,i);
        design(2,i) = x2(0) * P2(2,i) - P2(0,i);
        design(3,i) = x2(1) * P2(2,i) - P2(1,i);
      }
      Nullspace(&design, X_homogeneous);
  }

  void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
    const Mat34 &P2, const Vec2 &x2,
    Vec3 *X_euclidean) {
      Vec4 X_homogeneous;
      TriangulateDLT(P1, x1, P2, x2, &X_homogeneous);
      double w = X_homogeneous(3);
      *X_euclidean << X_homogeneous(0) / w, X_homogeneous(1) / w, X_homogeneous(2) / w;
  }

  //////////////////////////////////////////////////////////////////////////
  //from fundamental.cc:

  double Depth(const Mat3 &R, const Vec3 &t, const Vec3 &X) {
    return (R*X)(2) + t(2);
  }

  double Depth(const Mat3 &R, const Vec3 &t, const Vec4 &X) {
    Vec3 Xe = X.head<3>() / X(3);
    return Depth(R, t, Xe);
  }

  // HZ 9.6 pag 257 (formula 9.12)
  void EssentialFromFundamental(const Mat3 &F,
    const Mat3 &K1,
    const Mat3 &K2,
    Mat3 *E) {
      *E = K2.transpose() * F * K1;
  }

  void P_From_KRt(const Mat3 &K, const Mat3 &R, const Vec3 &t, Mat34 *P) {
    P->block<3, 3>(0, 0) = R;
    P->col(3) = t;
    (*P) = K * (*P);
  }

  // HZ 9.6 pag 259 (Result 9.19)
  void MotionFromEssential(const Mat3 &E,
    std::vector<Mat3> *Rs,
    std::vector<Vec3> *ts) {
      Eigen::JacobiSVD<Mat3> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Mat3 U =  USV.matrixU();
      Vec3 d =  USV.singularValues();
      Mat3 Vt = USV.matrixV().transpose();

      // Last column of U is undetermined since d = (a a 0).
      if (U.determinant() < 0) {
        U.col(2) *= -1;
      }
      // Last row of Vt is undetermined since d = (a a 0).
      if (Vt.determinant() < 0) {
        Vt.row(2) *= -1;
      }

      Mat3 W;
      W << 0, -1,  0,
        1,  0,  0,
        0,  0,  1;

      Mat3 U_W_Vt = U * W * Vt;
      Mat3 U_Wt_Vt = U * W.transpose() * Vt;

      Rs->resize(4);
      ts->resize(4);
      (*Rs)[0] = U_W_Vt;  (*ts)[0] =  U.col(2);
      (*Rs)[1] = U_W_Vt;  (*ts)[1] = -U.col(2);
      (*Rs)[2] = U_Wt_Vt; (*ts)[2] =  U.col(2);
      (*Rs)[3] = U_Wt_Vt; (*ts)[3] = -U.col(2);
  }

  // HZ 9.6 pag 259 (9.6.3 Geometrical interpretation of the 4 solutions)
  int MotionFromEssentialChooseSolution(const std::vector<Mat3> &Rs,
    const std::vector<Vec3> &ts,
    const Mat3 &K1,
    const Vec2 &x1,
    const Mat3 &K2,
    const Vec2 &x2) {
      assert(Rs.size() == 4);
      assert(ts.size() == 4);

      Mat34 P1, P2;
      Mat3 R1;
      Vec3 t1;
      R1.setIdentity();
      t1.setZero();
      P_From_KRt(K1, R1, t1, &P1);
      for (int i = 0; i < 4; ++i) {
        const Mat3 &R2 = Rs[i];
        const Vec3 &t2 = ts[i];
        P_From_KRt(K2, R2, t2, &P2);
        Vec3 X;
        TriangulateDLT(P1, x1, P2, x2, &X);
        double d1 = Depth(R1, t1, X);
        double d2 = Depth(R2, t2, X);
        // Test if point is front to the two cameras.
        if (d1 > 0 && d2 > 0) {
          return i;
        }
      }
      return -1;
  }

  bool MotionFromEssentialAndCorrespondence(const Mat3 &E,
    const Mat3 &K1, const Vec2 &x1,
    const Mat3 &K2, const Vec2 &x2,
    Mat3 *R, Vec3 *t) {
      std::vector<Mat3> Rs;
      std::vector<Vec3> ts;
      MotionFromEssential(E, &Rs, &ts);
      int solution = MotionFromEssentialChooseSolution(Rs, ts, K1, x1, K2, x2);
      if (solution >= 0) {
        *R = Rs[solution];
        *t = ts[solution];
        return true;
      } else {
        return false;
      }
  }

  enum {
    coef_xxx,
    coef_xxy,
    coef_xyy,
    coef_yyy,
    coef_xxz,
    coef_xyz,
    coef_yyz,
    coef_xzz,
    coef_yzz,
    coef_zzz,
    coef_xx,
    coef_xy,
    coef_yy,
    coef_xz,
    coef_yz,
    coef_zz,
    coef_x,
    coef_y,
    coef_z,
    coef_1
  };

  Mat FivePointsNullspaceBasis(const Mat2X &x1, const Mat2X &x2) {
    Matrix<double, 9, 9> A;
    A.setZero();  // Make A square until Eigen supports rectangular SVD.
    EncodeEpipolarEquation(x1, x2, &A);
    Eigen::JacobiSVD<Matrix<double, 9, 9> > svd;
    return svd.compute(A, Eigen::ComputeFullV).matrixV().topRightCorner<9,4>();
  }

  Vec o1(const Vec &a, const Vec &b) {
    Vec res = Vec::Zero(20);

    res(coef_xx) = a(coef_x) * b(coef_x);
    res(coef_xy) = a(coef_x) * b(coef_y)
      + a(coef_y) * b(coef_x);
    res(coef_xz) = a(coef_x) * b(coef_z)
      + a(coef_z) * b(coef_x);
    res(coef_yy) = a(coef_y) * b(coef_y);
    res(coef_yz) = a(coef_y) * b(coef_z)
      + a(coef_z) * b(coef_y);
    res(coef_zz) = a(coef_z) * b(coef_z);
    res(coef_x)  = a(coef_x) * b(coef_1)
      + a(coef_1) * b(coef_x);
    res(coef_y)  = a(coef_y) * b(coef_1)
      + a(coef_1) * b(coef_y);
    res(coef_z)  = a(coef_z) * b(coef_1)
      + a(coef_1) * b(coef_z);
    res(coef_1)  = a(coef_1) * b(coef_1);

    return res;
  }

  Vec o2(const Vec &a, const Vec &b) {
    Vec res(20);

    res(coef_xxx) = a(coef_xx) * b(coef_x);
    res(coef_xxy) = a(coef_xx) * b(coef_y)
      + a(coef_xy) * b(coef_x);
    res(coef_xxz) = a(coef_xx) * b(coef_z)
      + a(coef_xz) * b(coef_x);
    res(coef_xyy) = a(coef_xy) * b(coef_y)
      + a(coef_yy) * b(coef_x);
    res(coef_xyz) = a(coef_xy) * b(coef_z)
      + a(coef_yz) * b(coef_x)
      + a(coef_xz) * b(coef_y);
    res(coef_xzz) = a(coef_xz) * b(coef_z)
      + a(coef_zz) * b(coef_x);
    res(coef_yyy) = a(coef_yy) * b(coef_y);
    res(coef_yyz) = a(coef_yy) * b(coef_z)
      + a(coef_yz) * b(coef_y);
    res(coef_yzz) = a(coef_yz) * b(coef_z)
      + a(coef_zz) * b(coef_y);
    res(coef_zzz) = a(coef_zz) * b(coef_z);
    res(coef_xx)  = a(coef_xx) * b(coef_1)
      + a(coef_x)  * b(coef_x);
    res(coef_xy)  = a(coef_xy) * b(coef_1)
      + a(coef_x)  * b(coef_y)
      + a(coef_y)  * b(coef_x);
    res(coef_xz)  = a(coef_xz) * b(coef_1)
      + a(coef_x)  * b(coef_z)
      + a(coef_z)  * b(coef_x);
    res(coef_yy)  = a(coef_yy) * b(coef_1)
      + a(coef_y)  * b(coef_y);
    res(coef_yz)  = a(coef_yz) * b(coef_1)
      + a(coef_y)  * b(coef_z)
      + a(coef_z)  * b(coef_y);
    res(coef_zz)  = a(coef_zz) * b(coef_1)
      + a(coef_z)  * b(coef_z);
    res(coef_x)   = a(coef_x)  * b(coef_1)
      + a(coef_1)  * b(coef_x);
    res(coef_y)   = a(coef_y)  * b(coef_1)
      + a(coef_1)  * b(coef_y);
    res(coef_z)   = a(coef_z)  * b(coef_1)
      + a(coef_1)  * b(coef_z);
    res(coef_1)   = a(coef_1)  * b(coef_1);

    return res;
  }

  // Builds the polynomial constraint matrix M.
  Mat FivePointsPolynomialConstraints(const Mat &E_basis) {
    // Build the polynomial form of E (equation (8) in Stewenius et al. [1])
    Vec E[3][3];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        E[i][j] = Vec::Zero(20);
        E[i][j](coef_x) = E_basis(3 * i + j, 0);
        E[i][j](coef_y) = E_basis(3 * i + j, 1);
        E[i][j](coef_z) = E_basis(3 * i + j, 2);
        E[i][j](coef_1) = E_basis(3 * i + j, 3);
      }
    }

    // The constraint matrix.
    Mat M(10, 20);
    int mrow = 0;

    // Determinant constraint det(E) = 0; equation (19) of Nister [2].
    M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0]) + 
      o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1]) + 
      o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

    // Cubic singular values constraint.
    // Equation (20).
    Vec EET[3][3];
    for (int i = 0; i < 3; ++i) {    // Since EET is symmetric, we only compute
      for (int j = 0; j < 3; ++j) {  // its upper triangular part.
        if (i <= j) {
          EET[i][j] = o1(E[i][0], E[j][0])
            + o1(E[i][1], E[j][1])
            + o1(E[i][2], E[j][2]);
        } else {
          EET[i][j] = EET[j][i];
        }
      }
    }

    // Equation (21).
    Vec (&L)[3][3] = EET;
    Vec trace  = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
    for (int i = 0; i < 3; ++i) {
      L[i][i] -= trace;
    }

    // Equation (23).
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        Vec LEij = o2(L[i][0], E[0][j])
          + o2(L[i][1], E[1][j])
          + o2(L[i][2], E[2][j]);
        M.row(mrow++) = LEij;
      }
    }

    return M;
  }

  // Gauss--Jordan elimination for the constraint matrix.
  void FivePointsGaussJordan(Mat *Mp) {
    Mat &M = *Mp;

    // Gauss Elimination.
    for (int i = 0; i < 10; ++i) {
      M.row(i) /= M(i,i);
      for (int j = i + 1; j < 10; ++j) {
        M.row(j) = M.row(j) / M(j,i) - M.row(i);
      }
    }

    // Backsubstitution.
    for (int i = 9; i >= 0; --i) {
      for (int j = 0; j < i; ++j) {
        M.row(j) = M.row(j) - M(j,i) * M.row(i);
      }
    }
  }

  void FivePointsRelativePose(const Mat2X &x1,
    const Mat2X &x2,
    vector<Mat3> *Es) {
      // Step 1: Nullspace exrtraction.
      Mat E_basis = FivePointsNullspaceBasis(x1, x2);

      // Step 2: Constraint expansion.
      Mat M = FivePointsPolynomialConstraints(E_basis);

      // Step 3: Gauss-Jordan elimination.
      FivePointsGaussJordan(&M);

      // For the next steps, follow the matlab code given in Stewenius et al [1].

      // Build the action matrix.
      Mat B = M.topRightCorner<10,10>();
      Mat At = Mat::Zero(10,10);
      At.row(0) = -B.row(0);
      At.row(1) = -B.row(1);
      At.row(2) = -B.row(2);
      At.row(3) = -B.row(4);
      At.row(4) = -B.row(5);
      At.row(5) = -B.row(7);
      At(6,0) = 1;
      At(7,1) = 1;
      At(8,3) = 1;
      At(9,6) = 1;

      // Compute the solutions from action matrix's eigenvectors.
      Eigen::EigenSolver<Mat> es(At);
      typedef Eigen::EigenSolver<Mat>::EigenvectorsType Matc;
      Matc V = es.eigenvectors();
      Matc solutions(4, 10);
      solutions.row(0) = V.row(6).array() / V.row(9).array();
      solutions.row(1) = V.row(7).array() / V.row(9).array();
      solutions.row(2) = V.row(8).array() / V.row(9).array();
      solutions.row(3).setOnes();

      // Get the ten candidate E matrices in vector form.
      Matc Evec = E_basis * solutions;

      // Build the essential matrices for the real solutions.
      Es->reserve(10);
      for (int s = 0; s < 10; ++s) {
        Evec.col(s) /= Evec.col(s).norm();
        bool is_real = true;
        for (int i = 0; i < 9; ++i) {
          if (Evec(i, s).imag() != 0) {
            is_real = false;
            break;
          }
        }
        if (is_real) {
          Mat3 E;
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
              E(i, j) = Evec(3 * i + j, s).real();
            }
          }
          Es->push_back(E);
        }
      }
  }

  // HZ 9.6 pag 257 (formula 9.12)
  // Or http://ai.stanford.edu/~birch/projective/node20.html
  void FundamentalFromEssential(const Mat3 &E,
    const Mat3 &K1,
    const Mat3 &K2,
    Mat3 *F)  {
      *F = K2.inverse().transpose() * E * K1.inverse();
  }
  // Approximation of reprojection error; page 287 of HZ equation 11.9. This
  // avoids triangulating the point, relying only on the entries in F.
  double SampsonDistance2(const Mat &F, const Mat2X &x1, const Mat2X &x2) {
    double error_total= 0.0;
    Vec2 x1_i,x2_i;
    unsigned int n_points = x1.rows();
    for( unsigned int i = 0; i < n_points ; ++i)
    {
      x1_i = x1.col(i);
      x2_i = x2.col(i);

      error_total += SampsonDistance2(F, x1_i, x2_i);
    }

    return error_total;
  }
}