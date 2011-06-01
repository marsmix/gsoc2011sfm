#include "CameraPinhole.h"

using std::vector;
using cv::Mat;
using cv::Vec2d;
using cv::Vec3d;
using cv::Vec4d;
using cv::Range;

namespace OpencvSfM{

  CameraPinhole::CameraPinhole(Mat intra_params/*=Mat::eye(3, 3, CV_64F)*/,unsigned char wantedEstimation/*=FOCAL_PARAM|SKEW_PARAM|PRINCIPAL_POINT_PARAM*/)
  {
    CV_Assert( intra_params.rows==3 && intra_params.cols==3 );

    intra_params.convertTo(this->intra_params_,CV_64F);
    this->inv_intra_params_ = intra_params.inv();
    this->estimation_needed_ = wantedEstimation;
  }

  CameraPinhole::CameraPinhole(const vector<vector<cv::Point3f> >& objectPoints, const vector<vector<cv::Point2f> >& imagePoints, cv::Size imageSize, double aspectRatio/*=1.*/,unsigned char wantedEstimation)
  {
    Mat intra=initCameraMatrix2D(objectPoints,imagePoints,imageSize,aspectRatio);
    //to be sure that newParams has correct type:
    intra.convertTo(this->intra_params_,CV_64F);
    this->inv_intra_params_ = intra_params_.inv();
    this->estimation_needed_ = wantedEstimation;
  }
  CameraPinhole::~CameraPinhole()
  {
    //TODO
  }

  void CameraPinhole::updateIntrinsicMatrix(cv::Mat newParams,unsigned char intraValues/*=FOCAL_PARAM|SKEW_PARAM|PRINCIPAL_POINT_PARAM*/)
  {

    CV_Assert( !newParams.empty() || (newParams.rows==3) || (newParams.cols==3) );
    //to be sure that newParams has correct type:
    Mat correctMat;
    newParams.convertTo(correctMat,CV_64F);

    //now we can access to element using double array:
    double* ptrData=(double*)correctMat.data;
    double* ptrIntraParam=(double*)intra_params_.data;

    //first focal values:
    if(intraValues&FOCAL_PARAM)
    {
      ptrIntraParam[0]=ptrData[0];
      ptrIntraParam[4]=ptrData[4];
    }
    //skew param:
    if(intraValues&SKEW_PARAM)
      ptrIntraParam[1]=ptrData[1];

    //and principal point:
    if(intraValues&PRINCIPAL_POINT_PARAM)
    {
      ptrIntraParam[2]=ptrData[2];
      ptrIntraParam[5]=ptrData[5];
    }
  }


  vector<Vec4d> CameraPinhole::convertFromImageTo3Dray(vector<Vec3d> points)
  {
    //TODO
    return vector<Vec4d>();
  }

  vector<Vec2d> CameraPinhole::pixelToNormImageCoordinates(vector<Vec2d> points)
  {
    vector<Vec2d> newCoordinates;
    double* ptrIntraParam=(double*)inv_intra_params_.data;
    //for each 2D point, use inv_intra to compute 2D point:
    vector<Vec2d>::iterator point=points.begin();
    while(point!=points.end())
    {
      ///////////////////////////////////////////////////////////////////
      //Can be done using matrix multiplication like this:
      ///////////////////////////////////////////////////////////////////
      /*
      Vec3d pointNorm((*point)[0],(*point)[1],1);
      Mat pointNormImageCoordinate=inv_intra_params_ * Mat(pointNorm);
      double * dataPointImageCoordinate = (double *) pointNormImageCoordinate.data;
      Vec2d point2D(dataPointImageCoordinate[0]/dataPointImageCoordinate[2],dataPointImageCoordinate[1]/dataPointImageCoordinate[2]);
      newCoordinates.push_back(point2D);
      */

      ///////////////////////////////////////////////////////////////////
      //But this is faster (as intra matrix has only 5 values) :
      newCoordinates.push_back(Vec2d(ptrIntraParam[0] * (*point)[0] + ptrIntraParam[1] * (*point)[1] + ptrIntraParam[2],
        ptrIntraParam[4] * (*point)[1] + ptrIntraParam[5]));

      point++;
    }
    return newCoordinates;
  }

  vector<Vec2d> CameraPinhole::normImageToPixelCoordinates(std::vector<cv::Vec2d> points)
  {
    vector<Vec2d> newCoordinates;
    //for each 2D point, use intra params to compute 2D point:
    double* ptrIntraParam=(double*)intra_params_.data;

    vector<Vec2d>::iterator point=points.begin();
    while(point!=points.end())
    {
      ///////////////////////////////////////////////////////////////////
      //Same as pixelToNormImageCoordinates, faster than using matrix multiplication:
      newCoordinates.push_back(Vec2d(ptrIntraParam[0] * (*point)[0] + ptrIntraParam[1] * (*point)[1] + ptrIntraParam[2],
        ptrIntraParam[4] * (*point)[1] + ptrIntraParam[5]));

      point++;
    }
    return newCoordinates;
  }
}