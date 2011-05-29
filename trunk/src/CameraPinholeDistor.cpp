#include "CameraPinholeDistor.h"

using std::vector;
using cv::Mat;
using cv::Vec2d;
using cv::Vec3d;
using cv::Vec4d;
using cv::Vec6d;

namespace OpencvSfM{

  CameraPinholeDistor::CameraPinholeDistor(Mat intra_params,Vec6d radial_dist,unsigned char nbRadialParam,cv::Vec2d tangential_dist,unsigned char wantedEstimation)
    :CameraPinhole(intra_params,wantedEstimation)
  {
    updateDistortionParameters(radial_dist,nbRadialParam,tangential_dist);
  }

  CameraPinholeDistor::CameraPinholeDistor(const vector<vector<cv::Point3f> >& objectPoints, const vector<vector<cv::Point2f> >& imagePoints, cv::Size imageSize, double aspectRatio/*=1.*/,
    Vec6d radial_dist, unsigned char nbRadialParam, Vec2d tangential_dist, unsigned char wantedEstimation)
    :CameraPinhole(objectPoints,imagePoints,imageSize,aspectRatio,wantedEstimation)
  {
    updateDistortionParameters(radial_dist,nbRadialParam,tangential_dist);
  }
  CameraPinholeDistor::~CameraPinholeDistor()
  {
    //TODO
  }

  void CameraPinholeDistor::updateDistortionParameters(const cv::Vec6d& radial_dist, unsigned char nbRadialParam,const cv::Vec2d& tangential_dist,unsigned char wantedEstimation)
  {
    this->estimation_needed_= ( this->estimation_needed_ & 0x0F ) | ( wantedEstimation & 0xF0 );
    this->radial_dist_=radial_dist;
    this->nb_radial_params_=nbRadialParam;
    this->tangential_dist_=tangential_dist;
    if(estimation_needed_&RADIAL_PARAM)
    {
      int nbParamTotal=nb_radial_params_ + ((estimation_needed_&TANGEANT_PARAM)>>4);
      distortionVector.create(nbParamTotal,1,CV_64F);//(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]])
      double * vals=(double*)distortionVector.data;
      vals[0]=radial_dist_[0];
      vals[1]=radial_dist_[1];
      if(estimation_needed_&TANGEANT_PARAM)
      {
        vals[2]=tangential_dist_[0];
        vals[3]=tangential_dist_[1];
        if(nbRadialParam>2)
        {
          vals[4]=radial_dist_[2];
          if(nbRadialParam>3)
          {
            vals[5]=radial_dist_[3];
            vals[6]=radial_dist_[4];
            vals[7]=radial_dist_[5];
          }
        }
      }
    }
  }

  vector<Vec4d> CameraPinholeDistor::convertFromImageTo3Dray(vector<Vec3d> points)
  {
    //TODO
    return vector<Vec4d>();
  }

  vector<Vec2d> CameraPinholeDistor::imageToNormImageCoordinates(vector<Vec2d> points)
  {
    vector<Vec2d> pointsNorm=CameraPinhole::imageToNormImageCoordinates(points);
    double* ptrIntraParam=(double*)intra_params_.data;

    vector<Vec2d>::iterator point_centered=pointsNorm.begin();
    vector<Vec2d>::iterator point=points.begin();
    while(point!=pointsNorm.end())
    {

      double u = (*point_centered)[0];
      double v = (*point_centered)[1];

      double uSized=u*ptrIntraParam[0];//(focal x)
      double vSized=v*ptrIntraParam[1];//(focal y)

      double radius = u * u + v * v;

      double coef_radial = 0;
      if (nb_radial_params_ > 0) {
        for (int i = nb_radial_params_ - 1; i >= 0; --i) {
          coef_radial = (coef_radial + radial_dist_[i]) * radius;
        }
      }

      (*point_centered)[0] = (*point)[0] + uSized * coef_radial;//update the point coordinate
      (*point_centered)[1] = (*point)[1] + vSized * coef_radial;//update the point coordinate

      if(estimation_needed_&TANGEANT_PARAM)
      {
        double radius_squared = radius * radius;
        double coef_tangential = 1;

        coef_tangential += tangential_dist_[0] * radius_squared + tangential_dist_[1] * radius_squared;

        (*point_centered)[0] += (tangential_dist_[0] * (radius_squared +
          2. * u * u) + 2. * tangential_dist_[1] *
          u * v) * coef_tangential;
        (*point_centered)[1] += (tangential_dist_[1] * (radius_squared +
          2. * v * v) + 2. * tangential_dist_[0] *
          u * v) * coef_tangential;
      }
    }
    return pointsNorm;
  }

  vector<Vec2d> CameraPinholeDistor::normImageToPixelCoordinates(std::vector<cv::Vec2d> points)
  {
    //TODO
  }
}