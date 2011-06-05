/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
#include "autocalibration_estimator.h"

namespace libmv
{
  namespace autocalibration
  {
    void K_From_AbsoluteConic(const Mat3 &W, Mat3 *K) {
      // To compute upper-triangular Cholesky, we flip the indices of the input
      // matrix, compute lower-triangular Cholesky, and then unflip the result.
      Mat3 dual = W.inverse();
      Mat3 flipped_dual;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          flipped_dual(i,j) = dual(2 - i, 2 - j);
        }
      }

      Eigen::LLT<Mat3> llt(flipped_dual);
      Mat3 L = llt.matrixL();

      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          (*K)(i,j) = L(2 - i, 2 - j);
        }
      }

      // Resolve sign ambiguities assuming positive diagonal.
      for (int j = 0; j < 3; ++j) {
        if ((*K)(j, j) < 0) {
          for (int i = 0; i < 3; ++i) {
            (*K)(i, j) = -(*K)(i, j);
          }
        }
      }
    }

    int AutoCalibrationLinear::AddProjection(const Mat34 &P,
      double width, double height)
    {
        Mat34 P_normalized;
        NormalizeProjection(P, width, height, &P_normalized);

        AddProjectionConstraints(P_normalized);

        // Store input
        projections_.push_back(P_normalized);
        widths_.push_back(width);
        heights_.push_back(height);

        return projections_.size() - 1;
    }

    // TODO(pau): make this generic and move it to numeric.h
    static void SortEigenVectors(const Vec &values,
      const Mat &vectors, Vec *sorted_values, Mat *sorted_vectors)
    {
        // Compute eigenvalues order.
        std::pair<double, int> order[4];
        for (int i = 0; i < 4; ++i) {
          order[i].first = -values(i);
          order[i].second = i;
        }
        std::sort(order, order + 4);

        sorted_values->resize(4);
        sorted_vectors->resize(4,4);
        for (int i = 0; i < 4; ++i) {
          (*sorted_values)(i) = values[order[i].second];
          sorted_vectors->col(i) = vectors.col(order[i].second);
        }
    }

    Mat4 AutoCalibrationLinear::MetricTransformation()
    {
      // Compute the dual absolute quadric, Q.
      Mat A(constraints_.size(), 10);
      for (int i = 0; i < A.rows(); ++i) {
        A.row(i) = constraints_[i];
      }
      Vec q;
      Nullspace(&A, &q);
      Mat4 Q = AbsoluteQuadricMatFromVec(q);
      // TODO(pau) force rank 3.

      // Compute a transformation to a metric frame by decomposing Q.
      Eigen::SelfAdjointEigenSolver<Mat4> eigen_solver(Q);

      // Eigen values should be possitive,
      Vec temp_values = eigen_solver.eigenvalues();
      if (temp_values.sum() < 0) {
        temp_values = -temp_values;
      }

      // and sorted, so that last one is 0.
      Vec eigenvalues;
      Mat eigenvectors;
      SortEigenVectors(temp_values, eigen_solver.eigenvectors(),
        &eigenvalues, &eigenvectors);
      LOG(INFO) << "Q\n" << Q << "\n";
      LOG(INFO) << "eigen values\n" << eigenvalues << "\n";
      LOG(INFO) << "eigen vectors\n" << eigenvectors << "\n";

      // Compute the transformation from the eigen descomposition.  See last
      // paragraph of page 3 in
      //   "Autocalibration and the absolute quadric" by B. Triggs.
      eigenvalues(3) = 1;
      eigenvalues = eigenvalues.array().sqrt();
      Mat H = eigenvectors * eigenvalues.asDiagonal();
      return H;
    }

    void AutoCalibrationLinear::AddProjectionConstraints(const Mat34 &P)
    {
      double nu = 1;

      // Non-extreme focal lenght.
      constraints_.push_back((wc(P, 0, 0) - wc(P, 2, 2)) / 9 / nu);
      constraints_.push_back((wc(P, 1, 1) - wc(P, 2, 2)) / 9 / nu);

      // Aspect ratio is near 1.
      constraints_.push_back((wc(P, 0, 0) - wc(P, 1, 1)) / 0.2 / nu);

      // No skew and principal point near 0,0.
      // Note that there is a typo in the Pollefeys' paper: the 0.01 is not at the
      // correct equation.
      constraints_.push_back(wc(P, 0, 1) / 0.01 / nu);
      constraints_.push_back(wc(P, 0, 2) / 0.1 / nu);
      constraints_.push_back(wc(P, 1, 2) / 0.1 / nu);
    }

    Vec AutoCalibrationLinear::wc(const Mat34 &P, int i, int j)
    {
      Vec constraint(10);
      for (int k = 0; k < 10; ++k) {
        Vec q = Vec::Zero(10);
        q(k) = 1;
        Mat4 Q = AbsoluteQuadricMatFromVec(q);

        Mat3 w = P * Q * P.transpose();

        constraint(k) = w(i, j);
      }
      return constraint;
    }

    Mat4 AutoCalibrationLinear::AbsoluteQuadricMatFromVec(const Vec &q)
    {
      Mat4 Q;
      Q << q(0), q(1), q(2), q(3),
        q(1), q(4), q(5), q(6),
        q(2), q(5), q(7), q(8),
        q(3), q(6), q(8), q(9);
      return Q;
    }

    void AutoCalibrationLinear::NormalizeProjection(const Mat34 &P,
      double width, double height, Mat34 *P_new)
    {
        Mat3 T;
        T << width + height,              0,  (width - 1) / 2,
          0, width + height, (height - 1) / 2,
          0,              0,                1;
        *P_new = T.inverse() * P;
    }

    void AutoCalibrationLinear::DenormalizeProjection(const Mat34 &P,
      double width, double height, Mat34 *P_new)
    {
        Mat3 T;
        T << width + height,              0,  (width - 1) / 2,
          0, width + height, (height - 1) / 2,
          0,              0,                1;
        *P_new = T * P;
    }

    /** Calculate the last (sixth) point in projective 4 space. */
    static Vec4 CalcX6FromDesignMat(
      double a, double b, double c, double d, double e) {
        // This should match the matrix in step 6 above, equation (9) in [1].
        // The 6th world point is the nullspace of this matrix.
        Mat X6null(6,4);
        X6null << e-d,  0 ,  0 , a-b,
          e-c,  0 ,  a ,  0 ,
          d-c,  b ,  0 ,  0 ,
          0 ,  e ,  0 , a-c,
          0 , e-b, a-d,  0 ,
          0 ,  0 ,  d , b-c;
        Vec4 X6;
        Nullspace(&X6null, &X6);
        return X6;
    }

    // See paragraph after equation 16 in torr97robust for the equation used to
    // derive the following coefficients.
#define ACCUMULATE_CUBIC_COEFFICIENTS(x,y,z, sgn) \
  p = t1[x]*t1[y]; \
  q = t2[x]*t1[y] + t1[x]*t2[y]; \
  d += sgn *  p*t1[z]; \
  c += sgn * (q*t1[z] + p*t2[z]); \
  b += sgn * (t2[x]*t2[y]*t1[z] + q*t2[z]); \
  a += sgn *  t2[x]*t2[y]*t2[z];

    // TODO(keir): Break this up into smaller functions.
    // TODO(keir): Change 'points' from 2 x 6nviews to be 2n views x 6; this way it
    // can be directly passed from the robust estimation code without copying.
    void SixPointNView(const Mat2X &points,
      vector<SixPointReconstruction> *reconstructions) {

        int nviews = points.cols() / 6;

        // Convert to homogeneous coordinates.
        Mat3X hpoints(3, points.cols());
        hpoints.block(0, 0, 2, 6*nviews) = points;
        hpoints.row(2).setOnes();

        // See equation (7.2) p179 of HZ; this is the DLT for solving cameras.
        // Chose wi = 1, i.e. the homogeneous component of each image location is 1.
        // Note that As and Bs are on the heap to avoid blowing the stack for a large
        // number of views.
        Mat34 *As = new Mat34[nviews];
        Mat34 *Bs = new Mat34[nviews];
        Mat ws(nviews,5);

        for (int i = 0; i < nviews; ++i) {
          // Extract pencil of camera matrices.
          FivePointCameraPencil(hpoints.block(0, 6*i, 3, 5), As+i, Bs+i);

          // Calculate Q.
          Vec3 x6 = hpoints.col(6*i+5);
          Mat3 x6cross = CrossProductMatrix(x6);
          Mat4 Qa = As[i].transpose() * x6cross * Bs[i];
          Mat4 Q = Qa + Qa.transpose();

          // Read the coefficients w^i from Q and put into the ws matrix.
          ws(i,0) = Q(0,1);
          ws(i,1) = Q(0,2);
          ws(i,2) = Q(1,2);
          ws(i,3) = Q(1,3);
          ws(i,4) = Q(2,3);
        }
        Vec t1, t2;
        Nullspace2(&ws, &t1, &t2);

        // The svd gives us the basis for the nullspace of ws in which the t vector
        // lives, such that t = beta*t1+alpha*t2. However, there is a cubic
        // constraint on the elements of t, such that we can substitute and solve for
        // alpha. See equation (10) in [1].
        double a, b, c, d, p, q;
        a = b = c = d = 0;
        ACCUMULATE_CUBIC_COEFFICIENTS(0,1,3,  1);
        ACCUMULATE_CUBIC_COEFFICIENTS(0,1,4, -1);
        ACCUMULATE_CUBIC_COEFFICIENTS(0,2,4,  1);
        ACCUMULATE_CUBIC_COEFFICIENTS(0,3,4, -1);
        ACCUMULATE_CUBIC_COEFFICIENTS(1,2,3, -1);
        ACCUMULATE_CUBIC_COEFFICIENTS(1,3,4,  1);

        // TODO(keir): Handle case a = 0.
        // TODO(keir): Handle the case (a=b=c=d=0. If a=b=c=0 and d!=0, then alpha=0;
        // in that case, find beta instead.

        // Assume beta = 1.
        double a1 = b/a, b1 = c/a, c1 = d/a;
        a = a1;
        b = b1;
        c = c1;

        double alpha, alphas[3];
        int nroots = SolveCubicPolynomial(a, b, c, alphas+0, alphas+1, alphas+2);

        // Check each solution for alpha.
        reconstructions->resize(nroots);
        for (int ia=0; ia<nroots; ia++) {
          alpha = alphas[ia];

          double e;
          a = t1[0] + alpha*t2[0];
          b = t1[1] + alpha*t2[1];
          c = t1[2] + alpha*t2[2];
          d = t1[3] + alpha*t2[3];
          e = t1[4] + alpha*t2[4];

          SixPointReconstruction &pr = (*reconstructions)[ia];

          // The world position of the first five points, X1 through X5, are the
          // standard P^3 basis.
          pr.X.block<4, 4>(0, 0).setIdentity();
          pr.X.col(4).setOnes();

          // Find X6 from the chi vector.
          Vec4 Xp = CalcX6FromDesignMat(a, b, c, d, e);
          pr.X.col(5) = Xp;

          // Find P for each camera by finding suitable values of u,v.
          pr.P.resize(nviews);
          for (int i = 0; i < nviews; ++i) {
            // Project X6 with A and B. DO NOT NORMALIZE, it breaks the next step.
            Vec3 AX = As[i] * Xp;
            Vec3 BX = Bs[i] * Xp;

            // Find mu and nu with smallest algebraic error; see step 7. For exactly
            // six points, M will be rank 2. With more than 6 points, measurement
            // error will make M nonsingular.
            Mat3 M;
            M.col(0) = AX;
            M.col(1) = BX;
            M.col(2) = hpoints.col(6*i+5);  // x6.

            Vec3 munu;
            Nullspace(&M, &munu);
            double mu = munu[0];
            double nu = munu[1];

            pr.P[i] = mu*As[i] + nu*Bs[i];
          }
        }
        delete [] As;
        delete [] Bs;
    }

  }
}