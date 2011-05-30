#include "PointOfView.h"


using cv::Mat;
using cv::Vec3d;
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

    this->projection_matrix_=device_->computeProjectionMatrix(rotation,translation);

    this->config_=0;//everything should be estimated...

    //as we are a new point of view related to a device, we should add our address into device_:
    device_->pointsOfView_.push_back(this);
  };

  PointOfView::~PointOfView(void)
  {
    this->device_.release();
    this->rotation_.release();
    //this->translation_.release();//release don't exist...

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
}