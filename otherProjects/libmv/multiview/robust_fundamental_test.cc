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

#include <iostream>
#include <algorithm>
#define NOMINMAX
#include <windows.h>

#include "libmv/base/vector.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/robust_fundamental.h"
#include "libmv/multiview/robust_estimation.h"
#include "libmv/multiview/fundamental_kernel.h"
#include "libmv/multiview/fundamental_test_utils.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/test_data_sets.h"
#include "libmv/numeric/numeric.h"
#include "testing/testing.h"
#include "libmv/logging/logging.h"
#include "libmv/multiview/nviewtriangulation.h"
#include <fstream>
#include <string>

#include "libmv_core\two_view_solvers\TwoViewKernel.h"
#include "libmv_core\two_view_solvers\ErrorEstimator.h"


using namespace libmv;
namespace {
  /*
  TEST(RobustFundamental, FundamentalFromCorrespondences8PointRobust) {
    const int n = 16;
    Mat x1(2,n);
    x1 << 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4,   5,
      0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2,   5;

    Mat x2 = x1;
    for (int i = 0; i < n; ++i) {
      x2(0, i) += i % 2;  // Multiple horizontal disparities.
    }
    x2(0, n - 1) = 10;
    x2(1, n - 1) = 10;   // The outlier has vertical disparity.

    Mat3 F;
    vector<int> inliers;
    FundamentalFromCorrespondences8PointRobust(x1, x2, 0.1, &F, &inliers);

    LOG(INFO) << "F\n" << F << "\n";
    LOG(INFO) << "INLIERS " << inliers.size() << "\n";

    // F should be 0, 0,  0,
    //             0, 0, -1,
    //             0, 1,  0
    EXPECT_NEAR(0.0, F(0,0), 1e-8);
    EXPECT_NEAR(0.0, F(0,1), 1e-8);
    EXPECT_NEAR(0.0, F(0,2), 1e-8);
    EXPECT_NEAR(0.0, F(1,0), 1e-8);
    EXPECT_NEAR(0.0, F(1,1), 1e-8);
    EXPECT_NEAR(0.0, F(2,0), 1e-8);
    EXPECT_NEAR(0.0, F(2,2), 1e-8);
    EXPECT_NEAR(F(1,2), -F(2,1), 1e-8);

    EXPECT_EQ(n - 1, inliers.size());
  }

  TEST(RobustFundamental,
    FundamentalFromCorrespondences8PointRealisticNoOutliers) {
      TwoViewDataSet d = TwoRealisticCameras();

      Mat3 F_estimated;
      vector<int> inliers;
      FundamentalFromCorrespondences8PointRobust(d.x1, d.x2, 3.0,
        &F_estimated, &inliers);
      EXPECT_EQ(d.x1.cols(), inliers.size());

      // Normalize.
      Mat3 F_gt_norm, F_estimated_norm;
      NormalizeFundamental(d.F, &F_gt_norm);
      NormalizeFundamental(F_estimated, &F_estimated_norm);
      LOG(INFO) << "F_gt_norm =\n" << F_gt_norm;
      LOG(INFO) << "F_estimated_norm =\n" << F_estimated_norm;

      EXPECT_MATRIX_NEAR(F_gt_norm, F_estimated_norm, 1e-8);

      // Check fundamental properties.
      ExpectFundamentalProperties( F_estimated, d.x1, d.x2, 1e-8);
  }


  TEST(RobustFundamental, FundamentalFromCorrespondences8PointRealistic) {
    TwoViewDataSet d = TwoRealisticCameras();

    d.X = 3*Mat::Random(3, 50);
    LOG(INFO) << "X = \n" << d.X;

    Project(d.P1, d.X, &d.x1);
    Project(d.P2, d.X, &d.x2);
    LOG(INFO) << "x1 = \n" << d.x1;
    LOG(INFO) << "x2 = \n" << d.x2;

    Mat x1s, x2s;
    HorizontalStack(d.x1, 400*Mat::Random(2, 20), &x1s);
    HorizontalStack(d.x2, 400*Mat::Random(2, 20), &x2s);

    // Compute fundamental matrix from correspondences.
    Mat3 F_estimated;
    vector<int> inliers;
    FundamentalFromCorrespondences8PointRobust(x1s, x2s, 1,
      &F_estimated, &inliers);

    LOG(INFO) << "Number of inliers = " << inliers.size();
    EXPECT_LE(d.x1.cols(), inliers.size()); // Some outliers may be considered
    // inliers, that's fine.

    // Normalize.
    Mat3 F_gt_norm, F_estimated_norm;
    NormalizeFundamental(d.F, &F_gt_norm);
    NormalizeFundamental(F_estimated, &F_estimated_norm);
    LOG(INFO) << "F_gt_norm =\n" << F_gt_norm;
    LOG(INFO) << "F_estimated_norm =\n" << F_estimated_norm;

    // Compare with ground truth.
    EXPECT_MATRIX_NEAR(F_gt_norm, F_estimated_norm, 1e-8);

    // Check fundamental properties.
    ExpectFundamentalProperties( F_estimated, d.x1, d.x2, 1e-8);
  };
  */
  void test_FundamentalComputationTime() {
    double total1=0,total2=0,total3=0,total4=0;
    LARGE_INTEGER beforeT,afterT;
    int beforeT1;
    TwoViewDataSet d;
    Mat3 F_estimated,F_estimated1;
    for(int i=0;i<100;i++){
      d = TwoRealisticCameras();

      d.X = 3*Mat::Random(3, 1000);
      LOG(INFO) << "X = \n" << d.X;

      Project(d.P1, d.X, &d.x1);
      Project(d.P2, d.X, &d.x2);
      LOG(INFO) << "x1 = \n" << d.x1;
      LOG(INFO) << "x2 = \n" << d.x2;

      Mat x1s, x2s;
      HorizontalStack(d.x1, 400*Mat::Random(2, 500), &x1s);
      HorizontalStack(d.x2, 400*Mat::Random(2, 500), &x2s);

      // Compute fundamental matrix from correspondences.

      double threshold = 2;
      double best_score = HUGE_VAL;
      typedef fundamental::kernel::EightPointKernel Kernel;
      Kernel kernel(x1s, x2s);
      ErrorEstimator* sampsonError = new SampsonError(2);
      EightPointFundamentalKernel kernel1(x1s, x2s, sampsonError);

      best_score = HUGE_VAL;
      libmv::vector<int> inliers;
      libmv::vector<int> inliers1;
      MLEScorer<Kernel> mleScore(threshold);
      MLEScorerNew mleScorer(threshold);

      beforeT1=GetTickCount();
      QueryPerformanceCounter (&beforeT);
      F_estimated1 = Estimate(kernel, mleScore, &inliers, 
        &best_score, 1e-2);
      QueryPerformanceCounter (&afterT);
      total1+=((double)((__int64)afterT.QuadPart)-((__int64)beforeT.QuadPart));
      total3+=GetTickCount()-beforeT1;

      best_score = HUGE_VAL;
      
      beforeT1=GetTickCount();
      beforeT=afterT;
      F_estimated = EstimateClass(&kernel1, mleScorer, &inliers1,
        &best_score, 1e-2);
      QueryPerformanceCounter (&afterT);
      total2+=((double)((__int64)afterT.QuadPart)-((__int64)beforeT.QuadPart));
      total4+=GetTickCount()-beforeT1;
    }
    LARGE_INTEGER frequence;
    QueryPerformanceFrequency(&frequence);
    std::cout<<std::endl<<"Time used: "<<total1/(double)frequence.QuadPart<<std::endl;
    std::cout<<std::endl<<"Time used: "<<total2/(double)frequence.QuadPart<<std::endl;

    
    std::cout<<std::endl<<"Time used: "<<total3<<std::endl;
    std::cout<<std::endl<<"Time used: "<<total4<<std::endl;

    // Normalize.
    Mat3 F_gt_norm, F_estimated_norm,F_estimated_norm1;
    NormalizeFundamental(d.F, &F_gt_norm);
    NormalizeFundamental(F_estimated1, &F_estimated_norm1);
    NormalizeFundamental(F_estimated, &F_estimated_norm);

    // Compare with ground truth.
    EXPECT_MATRIX_NEAR(F_gt_norm, F_estimated_norm1, 1e-8);
    EXPECT_MATRIX_NEAR(F_gt_norm, F_estimated_norm, 1e-8);

    // Check fundamental properties.
    ExpectFundamentalProperties( F_estimated, d.x1, d.x2, 1e-8);
    ExpectFundamentalProperties( F_estimated_norm1, d.x1, d.x2, 1e-8);
  }

  void getIntraParams(NViewDataSet &data,int npoints)
  {
    int nviews = data.n;
    std::ifstream inPoints("logPoints.txt");
    std::string skeepWord;

    // Collect P matrices together.
    for (int j = 0; j < nviews; ++j) {
      double R[9];
      double T[3];
      double K[9];
      inPoints >> skeepWord;
      while( skeepWord.find("ProjectionsMat")==std::string::npos && !inPoints.eof() )
        inPoints >> skeepWord;
      inPoints >> K[0] >> K[1] >> K[2];
      inPoints >> K[3] >> K[4] >> K[5];
      inPoints >> K[6] >> K[7] >> K[8];
      data.K[j] <<K[0] , K[1] , K[2],
                  K[3] , K[4] , K[5],
                  K[6] , K[7] , K[8];

      inPoints >> R[0] >> R[1] >> R[2];
      inPoints >> R[3] >> R[4] >> R[5];
      inPoints >> R[6] >> R[7] >> R[8];
      data.R[j] <<R[0] , R[1] , R[2],
                  R[3] , R[4] , R[5],
                  R[6] , R[7] , R[8];

      inPoints >> T[0] >> T[1] >> T[2];
      data.t[j] <<T[0] , T[1] , T[2];
    }
    for (int i = 0; i < npoints; ++i) {
      inPoints >> skeepWord;
      while( skeepWord != "2D" && !inPoints.eof() )
        inPoints >> skeepWord;
      inPoints >> skeepWord;//skip Points word...
      inPoints >> data.X(0,i) >> data.X(1,i) >> data.X(2,i);
    }
    for (int i = 0; i < npoints; ++i) {
      std::cout << "2D points " << std::endl << data.X(0,i) << " "<<
        data.X(1,i) << " " << data.X(2,i) << std::endl;
    }
    //recompute projections:
    for (size_t i = 0; i < nviews; ++i) {
      data.x[i] = Project(data.P(i), data.X);
    }
  }

  void createTriangulationTest()
  {
    std::ofstream OutPoints("logPoints1.txt");
    int nviews = 5;
    int npoints = 6;
    NViewDataSet d = NRealisticCamerasFull(nviews, npoints);
    getIntraParams( d, npoints );

    // Collect P matrices together.
    vector<Mat34> Ps(nviews);
    for (int j = 0; j < nviews; ++j) {
      Ps[j] = d.P(j);
      OutPoints << "ProjectionsMat" << j << std::endl;
      OutPoints << d.K[j] << std::endl;
      OutPoints << d.R[j] << std::endl;
      OutPoints << d.t[j] << std::endl;
      OutPoints << "PMat" << std::endl;
      OutPoints << Ps[j] << std::endl;
      std::cout << "PMat" << std::endl;
      std::cout << Ps[j] << std::endl;
    }

    for (int i = 0; i < npoints; ++i) {
      OutPoints << "2D points " << std::endl << d.X(0,i) << " "<<
        d.X(1,i) << " " << d.X(2,i) << std::endl;
      // Collect the image of point i in each frame.
      Mat2X xs(2, nviews);
      // OutPoints << "  Image " << i << std::endl;
      for (int j = 0; j < nviews; ++j) {
        xs.col(j) = d.x[j].col(i);
        OutPoints << "    " << d.x[j].col(i)[0] << " " << d.x[j].col(i)[1] << std::endl;
      }
      Vec4 X;
      NViewTriangulate(xs, Ps, &X);

      // Check reprojection error. Should be nearly zero.
      for (int j = 0; j < nviews; ++j) {
        Vec3 x_reprojected = Ps[j]*X;
        x_reprojected /= x_reprojected(2);
        double error = (x_reprojected.head<2>() - xs.col(j)).norm();
        EXPECT_NEAR(error, 0.0, 1e-9);
      }
    }

  }/*
  TEST(RobustFundamental, FundamentalFromCorrespondences7PointRobust) {
    const int n = 16;
    Mat x1(2,n);
    x1 << 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5,
      0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 5;

    Mat x2 = x1;
    for (int i = 0; i < n; ++i) {
      x2(0, i) += i % 2;  // Multiple horizontal disparities.
    }
    x2(0, n - 1) = 10;
    x2(1, n - 1) = 10;   // The outlier has vertical disparity.

    Mat3 F;
    vector<int> inliers;
    FundamentalFromCorrespondences7PointRobust(x1, x2, 0.005, &F, &inliers);

    LOG(INFO) << "F\n" << F << "\n";
    LOG(INFO) << "INLIERS " << inliers.size() << "\n";

    // F should be similar to:
    // 0, -a,  -b,
    // a,  0,  -c,
    // b,  c,   0
    const double expectedPrecision = 1e-8;
    const double & ep = expectedPrecision;
    EXPECT_NEAR(0.0, F(0,0), ep);
    EXPECT_NEAR(0.0, F(1,1), ep);
    EXPECT_NEAR(0.0, F(2,2), ep);
    EXPECT_NEAR(F(0,1), -F(1,0), ep);
    EXPECT_NEAR(F(0,2), -F(2,0), ep);
    EXPECT_NEAR(F(1,2), -F(2,1), ep);

    EXPECT_EQ(n - 1, inliers.size());
    // 15 must not be in inliers indices list.
    EXPECT_EQ(std::find(inliers.begin(), inliers.end(), 15) == inliers.end() , true);
  }


  TEST(RobustFundamental, FundamentalFromCorrespondences7PointRealisticNoOutliers) {
    TwoViewDataSet d = TwoRealisticCameras();

    Mat3 F_estimated;
    vector<int> inliers;
    FundamentalFromCorrespondences7PointRobust(d.x1, d.x2, 3.0,
      &F_estimated, &inliers);
    EXPECT_EQ(d.x1.cols(), inliers.size());
    LG << "inliers number : " << inliers.size();

    // Normalize.
    Mat3 F_gt_norm, F_estimated_norm;
    NormalizeFundamental(d.F, &F_gt_norm);
    NormalizeFundamental(F_estimated, &F_estimated_norm);
    LOG(INFO) << "F_gt_norm =\n" << F_gt_norm;
    LOG(INFO) << "F_estimated_norm =\n" << F_estimated_norm;

    EXPECT_MATRIX_NEAR(F_gt_norm, F_estimated_norm, 1e-2);

    ExpectFundamentalProperties( F_estimated, d.x1, d.x2, 1e-6 );
  }*/

}; // namespace

#include "third_party/gflags/gflags.h"
#include "third_party/gtest/include/gtest/gtest.h"
#include "libmv/logging/logging.h"
#include <fstream>

// The following lines pull in the real gtest *.cc files.
#include "third_party/gtest/src/gtest.cc"
#include "third_party/gtest/src/gtest-death-test.cc"
#include "third_party/gtest/src/gtest-filepath.cc"
#include "third_party/gtest/src/gtest-port.cc"
#include "third_party/gtest/src/gtest-printers.cc"
#include "third_party/gtest/src/gtest-test-part.cc"
#include "third_party/gtest/src/gtest-typed-test.cc"


DEFINE_string(test_tmpdir, "/tmp", "Dir to use for temp files");

using namespace std;

int main(int argc, char **argv)
{
  /*
  TwoViewDataSet d = TwoRealisticCameras();
  Mat3 Rr,R2;
  Vec3 Tr,t2;
  RelativeCameraMotion(d.R1,d.t1,d.R2,d.t2,&Rr,&Tr);
  R2 = Rr * d.R1.transpose().inverse();
  t2 = Tr + Rr * d.t1;
  cout<<"d.R2"<<endl;
  cout<<d.R2<<endl;
  cout<<"d.t2"<<endl;
  cout<<d.t2<<endl;
  cout<<"R2"<<endl;
  cout<<R2<<endl;
  cout<<"t2"<<endl;
  cout<<t2<<endl;
  */
  std::ofstream Out("log.txt");
  std::clog.rdbuf(Out.rdbuf());
  createTriangulationTest();
  /*
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  //google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
  */
  //test_FundamentalComputationTime();
  return 0;
}