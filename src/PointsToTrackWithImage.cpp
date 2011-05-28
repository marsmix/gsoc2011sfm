#include "PointsToTrackWithImage.h"

using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Ptr;
using cv::SIFT;
using cv::Mat;
using std::string;

namespace OpencvSfM
{
  PointsToTrackWithImage::PointsToTrackWithImage(Mat imageToAnalyse,Mat maskOfAnalyse,Ptr<FeatureDetector> feature_detector,Ptr<DescriptorExtractor> descriptor_detector)
    :imageToAnalyse_(imageToAnalyse),maskOfAnalyse_(maskOfAnalyse),feature_detector_(feature_detector),descriptor_detector_(descriptor_detector)
  {
  }

  PointsToTrackWithImage::PointsToTrackWithImage(Mat imageToAnalyse,Mat maskOfAnalyse,string feature_detector, string descriptor_detector/*=""SIFT""*/)
    :imageToAnalyse_(imageToAnalyse),maskOfAnalyse_(maskOfAnalyse),
    feature_detector_(FeatureDetector::create(feature_detector)),descriptor_detector_(DescriptorExtractor::create(descriptor_detector))
  {
  }

  void PointsToTrackWithImage::setFeatureDetector(cv::Ptr<cv::FeatureDetector> feature_detector)
  {
    feature_detector_=feature_detector;
  }

  void PointsToTrackWithImage::setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor> descriptor_detector)
  {
    descriptor_detector_=descriptor_detector;
  }

  PointsToTrackWithImage::~PointsToTrackWithImage(void)
  {
    imageToAnalyse_.release();
    maskOfAnalyse_.release();
  }

  int PointsToTrackWithImage::computeKeypoints()
  {
    feature_detector_->detect(imageToAnalyse_,this->keypoints_,maskOfAnalyse_);
    return this->keypoints_.size();
  }

  void PointsToTrackWithImage::computeDescriptors()
  {
    descriptor_detector_->compute(imageToAnalyse_,this->keypoints_,this->descriptors_);
  }

}