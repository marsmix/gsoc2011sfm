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

#include <cmath>

#include "libmv_mapping.h"
#include "test_data_sets.h"

namespace libmv {


  nViewDatasetConfigator::nViewDatasetConfigator( int fx ,  int fy,
    int cx,   int cy,
    double distance,
    double jitter_amount)
  {
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    _dist = distance;
    _jitter_amount = jitter_amount;
  }

  Mat3 LookAt(Vec3 cent) {
    Mat projTmp = cent;
    Eigen::Map<libmv::Vec3> eigen_tmp((double*)projTmp.data);
    Vec3 center = eigen_tmp.transpose();

    Vec3 zc = center.normalized();
    Vec3 xc = Vec3::UnitY().cross(zc).normalized();
    Vec3 yc = zc.cross(xc);
    Mat3 R;
    R.row(0) = xc;
    R.row(1) = yc;
    R.row(2) = zc;
    return R;
  }

  NViewDataSet NRealisticCamerasFull(int nviews, int npoints,
    const nViewDatasetConfigator config) {
      NViewDataSet d;
      d.n = nviews;
      d.cameras.resize(nviews);
      d.C.resize(nviews);
      d.x.resize(nviews);
      //d.x_ids.resize(nviews);

      cv::RNG& rng = cv::theRNG();
      for(int i=0; i<npoints; i++)
        d.X.push_back( cv::Vec3d( rng.uniform(.0, .0), rng.uniform(.0, 2.0),
        rng.uniform(.0, 2.0)));
      
      for (size_t i = 0; i < nviews; ++i) {
        cv::Vec3d camera_center, t;
        cv::Vec3d jitter( rng.uniform(.0, 2.0), rng.uniform(.0, 2.0),
          rng.uniform(.0, 2.0));
        double theta = i * 2 * M_PI / nviews;
        camera_center[0] = sin(theta);
        camera_center[1] = 0.0;
        camera_center[2] = cos(theta);

        camera_center *= config._dist;
        d.C[i] = camera_center;

        jitter *= config._jitter_amount / cv::norm(camera_center);
        cv::Vec3d lookdir = -camera_center + jitter;

        double K[] = {config._fx, 0,config._cx, 0, config._fy, config._cy,
          0, 0, 1};
        Mat3 R = LookAt(lookdir);

        d.t[i] = -d.R[i] * camera_center;
        d.x[i] = Project(d.P(i), d.X);
      }
      return d;
  }
}  // namespace libmv
