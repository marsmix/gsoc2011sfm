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

#ifndef LIBMV_MULTIVIEW_TEST_DATA_SETS_H_
#define LIBMV_MULTIVIEW_TEST_DATA_SETS_H_

#include "PointOfView.h"
#include "libmv_mapping.h"
#include "opencv2/core/core.hpp"

namespace libmv {

  cv::Mat HStack (const cv::Mat& lhs, const cv::Mat& rhs) {
    cv::Mat res(lhs.rows, lhs.cols+rhs.cols, lhs.type());

    cv::Mat mat_tmp=res(cv::Range::all(),cv::Range(0,lhs.cols));
    lhs.copyTo( left_tmp );

    mat_tmp = res(cv::Range::all(),cv::Range(lhs.cols, lhs.cols+rhs.cols));
    rhs.copyTo( mat_tmp );
    return res;
  };

  // An N-view metric dataset . An important difference between this
  // and the other reconstruction data types is that all points are seen by all
  // cameras.
  struct NViewDataSet {
    std::vector<OpencvSfM::PointOfView> cameras;
    std::vector<cv::Vec3d> C;   // Camera centers.
    std::vector<cv::Vec3d> X;     // 3D points.
    std::vector<std::vector<cv::Vec2d>> x;  // Projected points; may have noise added.
    //std::vector<std::vector<int>>  x_ids;// Indexes of points corresponding to the projections

    int n;  // Actual number of cameras.

    cv::Mat P(int i) {
      assert(i < n);
      return cameras[i].getProjectionMatrix();
    }
    void Reproject() {
      x.clear();
      for (int i = 0; i < n; ++i) {
        x.push_back(cameras[i].project3DPointsIntoImage(X));
      }
    }
    // TODO(keir): Add gaussian jitter functions.
  };

  struct nViewDatasetConfigator
  {
    /// Internal camera parameters
    int _fx;
    int _fy;
    int _cx;
    int _cy;

    /// Camera random position parameters
    double _dist;
    double _jitter_amount;

    nViewDatasetConfigator( int fx = 1000,  int fy = 1000,
      int cx = 500,   int cy  = 500,
      double distance = 1.5,
      double jitter_amount = 0.01 );
  };

  NViewDataSet NRealisticCamerasFull(int nviews, int npoints,
    const nViewDatasetConfigator
    config = nViewDatasetConfigator());

  // Generates sparse projections (not all points are projected)
  NViewDataSet NRealisticCamerasSparse(int nviews, int npoints,
    float view_ratio = 0.6,
    unsigned min_projections = 3,
    const nViewDatasetConfigator
    config = nViewDatasetConfigator());

} // namespace libmv

#endif  // LIBMV_MULTIVIEW_TEST_DATA_SETS_H_
