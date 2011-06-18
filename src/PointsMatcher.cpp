#include "PointsMatcher.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace OpencvSfM{
  using cv::Mat;
  using cv::Ptr;
  using std::vector;
  using cv::KeyPoint;
  using cv::DMatch;
  using cv::Size;
  using cv::Rect;

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
  void PointsMatcher::drawMatches( const Mat& img1,
    const vector<cv::KeyPoint>& keypoints1,
    const vector<cv::KeyPoint>& keypoints2,
    const vector<cv::DMatch>& matches1to2, Mat& outImg,
    const cv::Scalar& matchColor, const cv::Scalar& singlePointColor,
    const std::vector<char>& matchesMask, int flags )
  {
    Size size( img1.cols, img1.rows );
    if( flags & cv::DrawMatchesFlags::DRAW_OVER_OUTIMG )
    {
      if( size.width > outImg.cols || size.height > outImg.rows )
        CV_Error( CV_StsBadSize, "outImg has size less than needed to draw img1" );
    }
    else
    {
      outImg.create( size, CV_MAKETYPE(img1.depth(), 3) );

      if( img1.type() == CV_8U )
        cv::cvtColor( img1, outImg, CV_GRAY2BGR );
      else
        img1.copyTo( outImg );
    }

    // draw keypoints
    if( !(flags & cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) )
    {
      drawKeypoints( outImg, keypoints1, outImg, singlePointColor,
        flags + cv::DrawMatchesFlags::DRAW_OVER_OUTIMG );
    }

    cv::RNG& rng = cv::theRNG();
    bool isRandMatchColor = matchColor == cv::Scalar::all(-1);
    vector<DMatch>::size_type total_size=matches1to2.size();
    // draw matches
    for( vector<DMatch>::size_type m = 0; m < total_size; m++ )
    {
      int i1 = matches1to2[m].queryIdx;
      int i2 = matches1to2[m].trainIdx;
      if( matchesMask.empty() || matchesMask[m] )
      {
        const KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];

        cv::Scalar color = isRandMatchColor ?
          cv::Scalar( rng(256), rng(256), rng(256) ) : matchColor;

        cv::Point center1( cvRound(kp1.pt.x), cvRound(kp1.pt.y) );
        cv::Point center2( cvRound(kp2.pt.x), cvRound(kp2.pt.y) );
        int radius = 3;
        cv::circle( outImg, center1, radius, color, 1, CV_AA );
        cv::circle( outImg, center2, radius, color, 1, CV_AA );
        cv::line( outImg, center1, center2, color, 1, CV_AA );

      }
    }

  }
}