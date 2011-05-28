#include "PointsMatcher.h"

using cv::Mat;
using cv::Ptr;
using std::vector;
using cv::KeyPoint;

namespace OpencvSfM{
  PointsMatcher::PointsMatcher( const Ptr<cv::DescriptorMatcher>& matcher )
    :matcher_( matcher )
  {
    CV_Assert( !matcher.empty() );
  }


  PointsMatcher::~PointsMatcher(void)
  {
    //TODO!!!!
  }

  void PointsMatcher::add( Ptr<PointsToTrack> pointCollection )
  {
    pointCollection_.push_back(pointCollection);
    Mat descMat=pointCollection->getDescriptors();
    if(descMat.empty())
    {
      vector<KeyPoint> keyPoints=pointCollection->getKeypoints();
      if(keyPoints.empty())
      {
        pointCollection->computeKeypoints();
      }
      pointCollection->computeDescriptors();
      descMat = pointCollection->getDescriptors();
    }

    vector<Mat> pointsDesc;
    pointsDesc.push_back( descMat );
    matcher_->add( pointsDesc );
  }

  void PointsMatcher::clear()
  {
    matcher_->clear();
    pointCollection_.clear();
  }

  void PointsMatcher::train()
  {
    matcher_->train();
  }

  bool PointsMatcher::isMaskSupported()
  {
    return matcher_->isMaskSupported();
  }

  void PointsMatcher::knnMatch(  cv::Ptr<PointsToTrack> queryPoints,std::vector<std::vector<cv::DMatch> >& matches, int knn,
    const std::vector<cv::Mat>& masks, bool compactResult)
  {
    vector<KeyPoint> keyPoints=queryPoints->getKeypoints();
    if(keyPoints.empty())
    {
      queryPoints->computeKeypoints();
      keyPoints=queryPoints->getKeypoints();
    }
    Mat descMat=queryPoints->getDescriptors();
    if(descMat.empty())
    {
      queryPoints->computeDescriptors();
      descMat=queryPoints->getDescriptors();
    }

    CV_Assert(!descMat.empty());

    matcher_->knnMatch( descMat, matches, knn, masks, compactResult );
  }

  void PointsMatcher::radiusMatch( cv::Ptr<PointsToTrack> queryPoints,std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
    const std::vector<cv::Mat>& masks, bool compactResult )
  {
    vector<KeyPoint> keyPoints=queryPoints->getKeypoints();
    if(keyPoints.empty())
    {
      queryPoints->computeKeypoints();
    }
    Mat descMat=queryPoints->getDescriptors();
    if(descMat.empty())
    {
      queryPoints->computeDescriptors();
      descMat=queryPoints->getDescriptors();
    }
    matcher_->radiusMatch( descMat, matches, maxDistance, masks, compactResult );
  }

  bool PointsMatcher::empty() const
  {
    return pointCollection_.empty() ||  matcher_->empty();
  }

  Ptr<PointsMatcher> PointsMatcher::clone( bool emptyTrainData ) const
  {
    Ptr<PointsMatcher> outPointMatcher=new PointsMatcher( matcher_->clone(emptyTrainData) );
    outPointMatcher->pointCollection_=pointCollection_;

    return outPointMatcher;
  }
}