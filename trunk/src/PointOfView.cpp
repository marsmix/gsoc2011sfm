#include "PointOfView.h"


using cv::Mat;
using cv::Vec4d;
using cv::Vec3d;
using cv::Vec2d;
using cv::Vec;
using cv::Ptr;
using std::vector;
using std::string;
using cv::imread;

namespace OpencvSfM{

  PointOfView::PointOfView(cv::Ptr<Camera> device, Mat rotation /*=Mat::eye(3, 3, CV_64F)*/, Vec3d translation /*=Vec(0.0,0.0,0.0)*/ )
  {
    CV_Assert( rotation.rows==3 && rotation.cols==3 );
    CV_Assert( !device.empty() );

    this->device_=device;
    this->rotation_=rotation;///<Rotation matrix R
    this->translation_=translation;///<Translation vector t

    this->projection_matrix_=device_->computeProjectionMatrix(this->rotation_,this->translation_);

    this->config_=0;//everything should be estimated...

    //as we are a new point of view related to a device, we should add our address into device_:
    device_->pointsOfView_.push_back(this);
  };

  PointOfView::~PointOfView(void)
  {
    this->projection_matrix_.release();

    //remove the reference in device_->pointsOfView_:
    vector<PointOfView*>::iterator ourRef=device_->pointsOfView_.begin();
    bool isFound=false;
    while(!isFound && ourRef != device_->pointsOfView_.end())
    {
      //be aware of NULL pointor:
      if( (*ourRef) == this )
      {
        isFound=true;
        (*ourRef)=NULL;
      }
      ourRef++;
    }
  }

  vector<Vec2d> PointOfView::project3DPointsIntoImage(vector<Vec3d> points)
  {
    //TODO: this version don't take into account the distortions of camera!
    //Maybe add a function in Camera like undistortPointwhich do nothing except for CameraPinholeDistor
    //and use this function to correct reprojected point...
    if(projection_matrix_.empty())
    {
      //compute projection matrix:
      this->projection_matrix_=device_->computeProjectionMatrix(this->rotation_,this->translation_);
    }
    //for each 3D point, use projection_matrix_ to compute 2D point:
    vector<Vec2d> pointsOut;
    vector<Vec3d>::iterator point=points.begin();
    Mat mat3DNorm(4,1,CV_64F);
    double* point3DNorm=(double*)mat3DNorm.data;
    Mat mat2DNorm(3,1,CV_64F);
    double* point2DNorm=(double*)mat2DNorm.data;
    while(point!=points.end())
    {
      point3DNorm[0] = (*point)[0];
      point3DNorm[1] = (*point)[1];
      point3DNorm[2] = (*point)[2];
      point3DNorm[3] = 1;
      //Projection:
      mat2DNorm = ( this->projection_matrix_ * mat3DNorm);

      pointsOut.push_back(Vec2d(point2DNorm[0]/point2DNorm[2],point2DNorm[1]/point2DNorm[2]));

      point++;
    }
    return pointsOut;
  }

  bool PointOfView::pointInFrontOfCamera(cv::Vec4d point)
  {
    if(projection_matrix_.empty())
    {
      //compute projection matrix:
      this->projection_matrix_=device_->computeProjectionMatrix(this->rotation_,this->translation_);
    }

    Mat pointTranspose= (Mat) point;
    double condition_1 = this->projection_matrix_.row(2).dot(pointTranspose.t()) * point[3];
    double condition_2 = point[2] * point[3];
    if( condition_1 > 0 && condition_2 > 0 )
      return true;
    else
      return false;
  }
}