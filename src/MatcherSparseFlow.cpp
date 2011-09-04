#include "MatcherSparseFlow.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>

#include "PointsMatcher.h"
#include "PointsToTrack.h"
#include "PointsToTrackWithImage.h"

namespace OpencvSfM{
  using cv::Mat;
  using cv::Ptr;
  using std::vector;
  using cv::KeyPoint;
  using cv::DMatch;
  using cv::Size;
  using cv::Rect;

  MatcherSparseFlow::MatcherSparseFlow( 
    const cv::Ptr<cv::DescriptorMatcher>& matcher, double dist_allowed )
    : PointsMatcher( matcher )
  {
    max_distance_ = dist_allowed;
  };
  
  cv::Ptr<PointsMatcher> MatcherSparseFlow::clone( bool emptyTrainData )
  {
    P_MUTEX( thread_concurr );
    MatcherSparseFlow* outPointMatcher =
      new MatcherSparseFlow( matcher_->clone( emptyTrainData ), max_distance_ );
    if( !emptyTrainData )
      outPointMatcher->pointCollection_ = pointCollection_;
    V_MUTEX( thread_concurr );

    return outPointMatcher;
  };
  
  void MatcherSparseFlow::match( cv::Ptr<PointsToTrack> queryPoints,
      std::vector<cv::DMatch>& matches,
      const std::vector<cv::Mat>& masks )
  {
    //first match using classical matcher:
    PointsMatcher::match( queryPoints, matches, masks );
    //Now try to improve these matches using optical flow:
    P_MUTEX( thread_concurr );

    const vector<KeyPoint>& keyPoints=pointCollection_[0]->getKeypoints( );
    const vector<KeyPoint>& keyPoints1=queryPoints->getKeypoints( );
    vector<cv::Point2f> keyPointsIn;
    vector<cv::Point2f> keyPointsOut;
    vector<uchar> status;
    vector<float> error;

    cv::Mat img1 = pointCollection_[0]->getImage();
    cv::Mat img2 = queryPoints->getImage();
    CV_Assert( !img1.empty() && !img2.empty() );

    for( size_t cpt = 0; cpt<keyPoints.size(); cpt++)
    {
      //is this keypoint in matches?
      int has_keypoint = -1;

      for( size_t cpt1 = 0; has_keypoint<0 && cpt1 < matches.size(); cpt1++ )
        has_keypoint = matches[ cpt1 ].trainIdx == cpt? cpt1 : -1;

      const KeyPoint &kp = keyPoints[cpt];
      keyPointsIn.push_back( cv::Point2f(kp.pt.x, kp.pt.y) );
      if( has_keypoint<0 )
      {//TODO: get the closest optical flow and set him the same displacement
        keyPointsOut.push_back( cv::Point2f(kp.pt.x, kp.pt.y) );
      }
      else
      {
        const KeyPoint &kp1 = keyPoints1[ matches[ has_keypoint ].queryIdx ];
        keyPointsOut.push_back( cv::Point2f(kp1.pt.x, kp1.pt.y) );
      }
    }

    cv::calcOpticalFlowPyrLK(img1, img2, keyPointsIn, keyPointsOut,
      status, error );

    //construct the DMatch vector (find the closest point in queryPoints)
    int idxPointsAdd = keyPoints1.size() - 1;
    vector<cv::KeyPoint> keypoints_to_add;
    vector< int > idx_to_add;
    for( size_t cpt = 0; cpt<keyPointsOut.size(); cpt++)
    {
      if( status[cpt]!=0 )
      {
        cv::Point2f& kp = keyPointsOut[ cpt ];
        if( kp.x>0 && kp.y>0 )
        {
          float dist_min = 1e10;
          int idx_min = 0;
        }
      }
    }
    queryPoints->free_descriptors( true );//descriptors need to be recomputed!
    queryPoints->addKeypoints( keypoints_to_add );
    for( size_t cpt = 0; cpt<keypoints_to_add.size(); cpt++)
    {
      cv::KeyPoint& kp = keypoints_to_add[ cpt ];
      float dist_min = 1e10;
      int idx_min = queryPoints->getClosestKeypoint( kp.pt );
      const cv::KeyPoint& kpClose = queryPoints->getKeypoint( idx_min );
      float dist = sqrt( (kpClose.pt.x - kp.pt.x)*(kpClose.pt.x - kp.pt.x)
        + (kpClose.pt.y - kp.pt.y) * (kpClose.pt.y - kp.pt.y) );
      if( dist_min<max_distance_ )
        matches.push_back( cv::DMatch(idx_min, idx_to_add[cpt], dist_min) );
    }
    V_MUTEX( thread_concurr );
  }
  
  void MatcherSparseFlow::knnMatch( cv::Ptr<PointsToTrack> queryPoints,
      std::vector<std::vector<cv::DMatch> >& matches, int k,
      const std::vector<cv::Mat>& masks, bool compactResult )
  {
    if( k!=1 )
      CV_Error( CV_StsError, "not yet implemented..." );

    vector<DMatch> matchesTmp;
    match( queryPoints, matchesTmp, masks );
    matches.push_back(matchesTmp);
  }
  
  void MatcherSparseFlow::radiusMatch( cv::Ptr<PointsToTrack> queryPoints,
    std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
      const std::vector<cv::Mat>& masks, bool compactResult )
  {
    double tmp = max_distance_;
    vector<DMatch> matchesTmp;
    match( queryPoints, matchesTmp, masks );
    matches.push_back(matchesTmp);
    max_distance_ = tmp;
  }

}