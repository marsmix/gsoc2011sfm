#include "Camera.h"

using cv::Mat;
using cv::Vec;
using std::vector;

namespace OpencvSfM{

  Camera::Camera(Mat intra_params/*=Mat::eye(3, 3, CV_64F)*/,Vec<double, 6> radial_dist/*=Vec(0.0)*/,Vec<double, 2> tangential_dist/*=Vec(0.0,0.0)*/)
  {
    CV_Assert( intra_params.rows==3 && intra_params.cols==3 );

    this->intra_params_=intra_params;
    this->radial_dist_=radial_dist;
    this->tangential_dist_=tangential_dist;
    this->config_=0;
  }


  Camera::~Camera(void)
  {
  }

}