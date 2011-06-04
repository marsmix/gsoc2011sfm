/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
#include "essential_estimator.h"

namespace libmv
{
  namespace essential_matrix
  {
    // HZ 9.6 pag 257 (formula 9.12)
    void EssentialFromFundamental(const Mat3 &F, const Mat3 &K1,
      const Mat3 &K2, Mat3 *E)
    {
        *E = K2.transpose() * F * K1;
    }

    // HZ 9.6 pag 257
    void EssentialFromRt(const Mat3 &R1, const Vec3 &t1,
      const Mat3 &R2, const Vec3 &t2, Mat3 *E)
    {
        Mat3 R;
        Vec3 t;
        projection_estimation::RelativeCameraMotion(R1, t1, R2, t2, &R, &t);
        Mat3 Tx = CrossProductMatrix(t);
        *E = Tx * R;
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
          projection_estimation::TriangulateDLT(P1, x1, P2, x2, &X);
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
      const Mat3 &K1,
      const Vec2 &x1,
      const Mat3 &K2,
      const Vec2 &x2,
      Mat3 *R,
      Vec3 *t) {
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
  }
}