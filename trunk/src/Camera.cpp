#include "Camera.h"

using cv::Mat;
using cv::Vec3d;
using cv::Range;
using std::vector;

namespace OpencvSfM{

  Camera::Camera()
  {
  }


  Camera::~Camera(void)
  {
  }

  Mat Camera::computeProjectionMatrix(const Mat &rotation,const Vec3d &translation)
  {
    CV_Assert( !rotation.empty() && rotation.type()==CV_64F );
    //P = K [R|t]
    Mat Rt;
    Rt.create(3,4,CV_64F);
    //Copy R into Rt
    rotation.copyTo(Rt(Range::all(),Range(0,3)));
    //Copy t into Rt
    ((Mat)translation).copyTo(Rt.col(3));

    //compute K * Rt
    return Rt;
  }
}