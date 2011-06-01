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

  void PointsMatcher::knnMatch(  Ptr<PointsToTrack> queryPoints,vector<vector<DMatch> >& matches, int knn,
    const vector<Mat>& masks, bool compactResult)
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

  void PointsMatcher::radiusMatch( cv::Ptr<PointsToTrack> queryPoints,vector<vector<DMatch> >& matches, float maxDistance,
    const vector<Mat>& masks, bool compactResult )
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

  void PointsMatcher::crossCheck( Ptr<PointsToTrack> queryPoints,vector<vector<DMatch>>& matches, bool compactMatches )
  {
    //As always, ensure that queryPoints are computed. If not, compute them:
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

    //first create a new matcher using the same algorithm:
    Ptr<cv::DescriptorMatcher> matcherAlgo=matcher_->clone(true);
    //Add the queryPoints as data:
    vector<Mat> pointsDesc;
    pointsDesc.push_back( descMat );
    matcherAlgo->add(pointsDesc);

    //now construct the vector of DMatch, but in the other way:
    vector<vector<DMatch>> matchesOtherWay;
    unsigned int nbPointsCollection=pointCollection_.size();
    for(unsigned int i=0; i<nbPointsCollection; i++)
    {
      vector<DMatch> matchesTmp;
      matcherAlgo->match( pointCollection_[i]->getDescriptors(), matchesTmp );
      matchesOtherWay.push_back(matchesTmp);
    }

    //now check for reciprocity:
    unsigned int nbPoints=matches.size();
    for(unsigned int i=0; i<nbPoints; i++)
    {
      unsigned int nbMatches=matches[i].size();
      for(unsigned int j=0; j<nbMatches; j++)
      {
        DMatch d1=matches[i][j];
        DMatch d2=matchesOtherWay[d1.imgIdx][d1.trainIdx];
        if(d2.trainIdx!=d1.queryIdx)
        {
          //remove the current match!
          nbMatches--;
          matches[i][j]=matches[i][nbMatches];
          matches[i].pop_back();
          j--;//because we have to test this match again as the value as changed!
        }
      }
    }

    //now we can look for mutual matches, and keep only correct matches:
    if(compactMatches)
    {
      //if we have no match for a point, remove from vector:
      nbPoints=matches.size();
      for(unsigned int i=0; i<nbPoints; i++)
      {
        if(matches[i].empty())
        {
          nbPoints--;
          matches[i]=matches[nbPoints];
          matches.pop_back();
          i--;//because we have to test this match again as the value as changed!
        }
      }
    }
  }
}