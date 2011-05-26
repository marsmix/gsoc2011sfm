#include "PointsToTrackSIFT.h"

using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Ptr;
using cv::SIFT;
using cv::Mat;

namespace OpencvSfM{
    PointsToTrackSIFT::PointsToTrackSIFT(Mat imageToAnalyse,Mat maskOfAnalyse,double threshold, double edgeThreshold,int nOctaves,int nOctaveLayers,int firstOctave,int angleMode):
  imageToAnalyse_(imageToAnalyse),maskOfAnalyse_(maskOfAnalyse),sift_detector_(threshold, edgeThreshold,nOctaves,nOctaveLayers,firstOctave,angleMode)
  {
    //as SIFT algorithm work only with grey image, ensure that is the case:
    if( imageToAnalyse.empty() || imageToAnalyse.type() != CV_8UC1 )
      CV_Error( CV_StsBadArg, "imageToAnalyse is empty or has incorrect type" );
  }

  PointsToTrackSIFT::PointsToTrackSIFT(Mat imageToAnalyse,Mat maskOfAnalyse,double _magnification, bool _isNormalize,bool _recalculateAngles,int _nOctaves,int _nOctaveLayers,int _firstOctave,int _angleMode):
  imageToAnalyse_(imageToAnalyse),maskOfAnalyse_(maskOfAnalyse),sift_detector_(_magnification, _isNormalize,_recalculateAngles,_nOctaves,_nOctaveLayers,_firstOctave,_angleMode)
  {
    //as SIFT algorithm work only with grey image, ensure that is the case:
    if( imageToAnalyse.empty() || imageToAnalyse.type() != CV_8UC1 )
      CV_Error( CV_StsBadArg, "imageToAnalyse is empty or has incorrect type" );
  }


  PointsToTrackSIFT::PointsToTrackSIFT(Mat imageToAnalyse,Mat maskOfAnalyse,const SIFT::CommonParams& _commParams,const SIFT::DetectorParams& _detectorParams,const SIFT::DescriptorParams& _descriptorParams):
  imageToAnalyse_(imageToAnalyse),maskOfAnalyse_(maskOfAnalyse),sift_detector_(_commParams, _detectorParams,_descriptorParams)
  {
    //as SIFT algorithm work only with grey image, ensure that is the case:
    if( imageToAnalyse.empty() || imageToAnalyse.type() != CV_8UC1 )
      CV_Error( CV_StsBadArg, "imageToAnalyse is empty or has incorrect type" );
  }

  PointsToTrackSIFT::~PointsToTrackSIFT(void)
  {
    imageToAnalyse_.release();
    maskOfAnalyse_.release();
  }

  int PointsToTrackSIFT::computeKeypointsAndDesc(){
    sift_detector_(imageToAnalyse_,maskOfAnalyse_,this->keypoints_,this->descriptors_);
    return this->keypoints_.size();
  }

  int PointsToTrackSIFT::computeKeypoints(){
    sift_detector_(imageToAnalyse_,maskOfAnalyse_,this->keypoints_);
    return this->keypoints_.size();
  }

  void PointsToTrackSIFT::computeDescriptors(std::vector<cv::KeyPoint> keypoints){
    //add the keypoints to the end of our points vector:
    this->keypoints_.insert( this->keypoints_.end(),keypoints.begin(),keypoints.end());
    sift_detector_(imageToAnalyse_,maskOfAnalyse_,this->keypoints_,this->descriptors_,true);
  }

}