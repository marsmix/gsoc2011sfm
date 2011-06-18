#include "PointsMatcher.h"

using cv::Mat;
using cv::Ptr;
using std::vector;
using cv::KeyPoint;
using cv::DMatch;

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
    pointCollection->computeKeypointsAndDesc(false);

    Mat descMat=pointCollection->getDescriptors();
    if(descMat.empty())
    {
      vector<KeyPoint> keyPoints=pointCollection->getKeypoints();
      CV_Assert( !keyPoints.empty() );

      pointCollection->computeDescriptors();
      descMat = pointCollection->getDescriptors();
    }

    vector<Mat> pointsDesc;
    pointsDesc.push_back( descMat );
    matcher_->add( pointsDesc );
    pointCollection_.push_back(pointCollection);

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

  void PointsMatcher::match( cv::Ptr<PointsToTrack> queryPoints,
    std::vector<cv::DMatch>& matches,
    const std::vector<cv::Mat>& masks )
  {
    queryPoints->computeKeypointsAndDesc(false);

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints();
    Mat descMat=queryPoints->getDescriptors();

    CV_Assert( !keyPoints.empty() );
    CV_Assert( !descMat.empty() );
    
    matcher_->match( descMat, matches, masks );
  }

  void PointsMatcher::knnMatch(  Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, int knn,
    const vector<Mat>& masks, bool compactResult)
  {
    queryPoints->computeKeypointsAndDesc(false);

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints();
    Mat descMat=queryPoints->getDescriptors();

    CV_Assert( !keyPoints.empty() );
    CV_Assert( !descMat.empty() );

    matcher_->knnMatch( descMat, matches, knn, masks, compactResult );
  }

  void PointsMatcher::radiusMatch( cv::Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, float maxDistance,
    const vector<Mat>& masks, bool compactResult )
  {
    queryPoints->computeKeypointsAndDesc(false);

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints();
    Mat descMat=queryPoints->getDescriptors();

    CV_Assert( !keyPoints.empty() );
    CV_Assert( !descMat.empty() );

    matcher_->radiusMatch( descMat, matches, maxDistance, masks, compactResult );
  }

  bool PointsMatcher::empty() const
  {
    return pointCollection_.empty() ||  matcher_->empty();
  }

  Ptr<PointsMatcher> PointsMatcher::clone( bool emptyTrainData ) const
  {
    Ptr<PointsMatcher> outPointMatcher=new PointsMatcher( matcher_->clone(emptyTrainData) );
    if( !emptyTrainData )
      outPointMatcher->pointCollection_=pointCollection_;

    return outPointMatcher;
  }

  void PointsMatcher::crossMatch( Ptr<PointsMatcher> otherMatcher,
    vector<DMatch>& matches,
    const std::vector<cv::Mat>& masks )
  {
    //TODO: for now this function only work with matcher having only one picture
    CV_Assert( this->pointCollection_.size()==1 );
    CV_Assert( otherMatcher->pointCollection_.size()==1 );

    if(matches.empty())
    {
      //as we don't have matches guess, compute them:
      match( otherMatcher->pointCollection_[0], matches, masks);
    }

    
    //now construct the vector of DMatch, but in the other way:
    vector<DMatch> matchesOtherWay;
    otherMatcher->match( pointCollection_[0], matchesOtherWay, masks );
    //now check for reciprocity:
    unsigned int nbPoints=matches.size();
    for(unsigned int i=0; i<nbPoints; i++)
    {
      DMatch d1=matches[i];
      DMatch d2=matchesOtherWay[d1.trainIdx];
      if(d2.trainIdx!=d1.queryIdx)
      {
        //remove the current match!
        nbPoints--;//because we have a match less!
        matches[i]=matches[nbPoints];
        matches.pop_back();
        i--;//because we have to test this one!
      }
    }
    matchesOtherWay.clear();
  }
}