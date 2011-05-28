#include "FieldOfView.h"


using cv::Mat;
using cv::Vec;
using std::vector;
using std::string;
using cv::imread;

namespace OpencvSfM{

  FieldOfView::FieldOfView()
  {
    this->config_=MASK_FIELD_NOT_LOADED;//not really loaded...

    this->pointsInfo_=0;//We don't have points for now...
    this->matches_=0;//We don't have matches for now...
    this->extern_position_=0;//We don't have a bundle adjustement working for now...
    this->points3D_=0;//We don't have 3D points for now...
    this->device_=0;//We don't have device for nom...

    this->rotation_=Mat::eye(3, 3, CV_64F);
    this->translation_=Vec<double, 3>(0.0,0.0,0.0);
    this->projection_matrix_=Mat::eye(3, 4, CV_64F);//TODO : create this matrix using rotation_ and translation_ !
  }
  FieldOfView::FieldOfView(Camera *device,string imgFileName,Mat rotation /*=Mat::eye(3, 3, CV_64F)*/ ,Vec<double, 3> translation /*=Vec(0.0,0.0,0.0)*/ )
  {
    CV_Assert( rotation.rows==3 && rotation.cols==3 );
    CV_Assert( device!=0 );

    this->device_=device;
    this->image_=imread(imgFileName);
    this->rotation_=rotation;///<Rotation matrix R
    this->translation_=translation;///<Translation vector t
    this->projection_matrix_=Mat::eye(3, 4, CV_64F);//TODO : create this matrix using rotation_ and translation_ !

    this->pointsInfo_=0;//We don't have points for now...
    this->matches_=0;//We don't have matches for now...
    this->extern_position_=0;//We don't have a bundle adjustement working for now...
    this->points3D_=0;//We don't have 3D points for now...

    this->config_=0;//everything should be estimated...
  };

  FieldOfView::FieldOfView(Camera *device,Mat image,Mat rotation /*=Mat::eye(3, 3, CV_64F)*/ ,Vec<double, 3> translation /*=Vec(0.0,0.0,0.0)*/ )
  {
    CV_Assert( rotation.rows==3 && rotation.cols==3 );
    CV_Assert( ! image.empty() );

    this->device_=device;
    this->image_=image;
    this->rotation_=rotation;///<Rotation matrix R
    this->translation_=translation;///<Translation vector t
    this->projection_matrix_=Mat::eye(3, 4, CV_64F);//TODO : create this matrix using rotation_ and translation_ !

    this->pointsInfo_=0;//We don't have points for now...
    this->matches_=0;//We don't have matches for now...
    this->extern_position_=0;//We don't have a bundle adjustement working for now...
    this->points3D_=0;//We don't have 3D points for now...

    this->config_=0;//everything should be estimated...
  }

  FieldOfView::FieldOfView(Camera *device,PointsToTrack *points,PointsMatcher *matches,Mat rotation /*=Mat::eye(3, 3, CV_64F)*/ ,Vec<double, 3> translation /*=Vec(0.0,0.0,0.0)*/ )
  {
    CV_Assert( device!=0 );
    CV_Assert( points!=0 );

    this->device_=device;
    //this->image_ will be created using default constructor, no needs to do something else...
    this->rotation_=rotation;///<Rotation matrix R
    this->translation_=translation;///<Translation vector t
    this->projection_matrix_=Mat::eye(3, 4, CV_64F);//TODO : create this matrix using rotation_ and translation_ !

    this->pointsInfo_=points;//We use points they give us...
    this->matches_=matches;//We don't have matches for now...
    this->extern_position_=0;//We don't have a bundle adjustement working for now...
    this->points3D_=0;//We don't have 3D points for now...

    this->config_=MASK_POINTS_OK;//everything should be estimated, except points
    if(matches!=0) this->config_|=MASK_MATCHES_OK;//and matches if given by the user...
  }

  FieldOfView::FieldOfView(FieldOfView& otherFieldOfView)
  {
    //TODO!!!!
  }
  FieldOfView::~FieldOfView(void)
  {
    //TODO!!!!
  }
}