//only function header of useful function.


namespace OpencvSfM
{
  namespace fundamental_matrix
  {

    ////////////// In fundamental.cc:

    //////////////////////////////////////////////////////////////////////////
    ////////////////  How can we create a fundamental matrix:  ///////////////

    //Create fundamental matrix from two projection matrix:
    void FundamentalFromProjections(const Mat34 &P1, const Mat34 &P2, Mat3 *F);
    //Using Essential matrix and camera intra parameters, compute the fundamental matrix
    void FundamentalFromEssential(const Mat3 &E,const Mat3 &K1,const Mat3 &K2,Mat3 *F);
    //8point algorithm without the rank-2 constraint
    double EightPointSolver(const Mat &x1, const Mat &x2, Mat3 *F);
    //8point algorithm with normalisation of points AND rank-2 constraint.
    double NormalizedEightPointSolver(const Mat &x1, const Mat &x2, Mat3 *F);
    // 7 points (minimal case, points coordinates must be normalized before):
    double FundamentalFrom7CorrespondencesLinear(const Mat &x1,const Mat &x2,vector<Mat3> *F);
    // 7 points (points coordinates must be in image space):
    double FundamentalFromCorrespondences7Point(const Mat &x1,const Mat &x2,vector<Mat3> *F);

    //////////////////////////////////////////////////////////////////////////
    ///////////////  What can we do with a fundamental matrix:  //////////////

    //rank-2 constraint on f matrix to have 2 points for epipoles
    void EnforceFundamentalRank2Constraint(Mat3 *F);
    //normalize fundamental using Frobenius Norm
    void NormalizeFundamental(const Mat3 &F, Mat3 *F_normalized);
    //Create two projection matrix from fundamental matrix:
    void ProjectionsFromFundamental(const Mat3 &F, Mat34 *P1, Mat34 *P2);
    // Approximation of reprojection error using epipolar line distance extracted from F
    double SampsonDistance2(const Mat &F, const Vec2 &x1, const Vec2 &x2);

    ////////////// In focal_from_fundamental.cc:

    //compute the position of the two epipoles
    void EpipolesFromFundamental(const Mat3 &F, Vec3 *e1, Vec3 *e2);
    // Find focal from fundamental using Hartleys method.
    void FocalFromFundamental(const Mat3 &F,const Vec2 &principal_point1,
      const Vec2 &principal_point2,double *f1,double *f2);
    // Find focal from fundamental using Golden ratio search on reproj error on x1 and x2
    void FocalFromFundamentalExhaustive(const Mat3 &F,
      const Vec2 &principal_point,const Mat2X &x1,const Mat2X &x2,
      double min_focal,double max_focal,int n_samples, double *focal);

  }

  namespace essential_matrix
  {

    ////////////// In fundamental.cc:

    ////////////////   How can we create an essential matrix:  //////////////
    //Using fundamental matrix and camera intra parameters, compute the Essential matrix
    void EssentialFromFundamental(const Mat3 &F,const Mat3 &K1,const Mat3 &K2,Mat3 *E);
    //Using rotation and translation of two camera, compute the Essential matrix
    void EssentialFromRt(const Mat3 &R1,const Vec3 &t1,const Mat3 &R2,const Vec3 &t2,Mat3 *E);

    ///////////////  What can we do with a essential matrix:  //////////////
    //create 4 rotation/translation possibilities extracted from essential matrix
    void MotionFromEssential(const Mat3 &E, vector<Mat3> *Rs, vector<Vec3> *ts);
    // Choose one of the four possible motion solutions from an essential matrix.
    int MotionFromEssentialChooseSolution(...);
    int MotionFromEssentialAndCorrespondence(...);

    ///////////// In five_point.cc:
    // Computes at most 10 candidate essential matrix solutions from 5 correspondences.
    void FivePointsRelativePose(const Mat2X &x1, const Mat2X &x2,vector<Mat3> *E);
  }

  namespace projection_estimation
  {

    ////////////// In nviewtriangulation.h

    ///////////////  How can we triangulate points:  ////////////////
    // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
    // output, X, is a homogeneous four vectors.
    void NViewTriangulate(const Mat &x,const vector<Mat> &Ps,Mat *X);
    void NViewTriangulateAlgebraic(const Mat &x,const vector<Mat> &Ps,Mat *X);

    ////////////// In nviewtriangulation.h

    ///////////////  How can we triangulate points:  ////////////////
    // x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras. The
    // output, X, is a homogeneous four vectors.
    void NViewTriangulate(const Mat &x,const vector<Mat> &Ps,Mat *X);
    void NViewTriangulateAlgebraic(const Mat &x,const vector<Mat> &Ps,Mat *X);
  }

  namespace autocalibration
  {

    ////////////// In autocalibration.h
    //See how this could be integrated into CameraPinhole.h


    // Create intra parameters using absolute conic:
    void K_From_AbsoluteConic(const Mat3 &W, Mat3 *K);
    //class to perform linear estimation of metric transformation H:
    //If {P, X} is a projective reconstruction, then {P H, H^{-1} X} is
    // a metric reconstruction.
    class AutoCalibrationLinear;
  }
}