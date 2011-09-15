#include "MatcherSparseFlow.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
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
    //Ptr<PointsMatcher> simple_matcher = new PointsMatcher( this->matcher_->clone( true ) );
    //simple_matcher->add( queryPoints );
    //simple_matcher->match( pointCollection_[0], matches, masks );
    PointsMatcher::match( queryPoints,matches );

    P_MUTEX( thread_concurr );


    //////////////////////////////////////////////////////////////////////////
    //Using fundamental function from OpenCV, improve the matches

    //First compute points matches:
    int size_match=matches.size( );
    Mat srcP( 1,size_match,CV_32FC2 );
    Mat destP( 1,size_match,CV_32FC2 );
    vector<uchar> status;

    //vector<KeyPoint> points1 = point_matcher->;
    for( int i = 0; i < size_match; i ++ ){
      const KeyPoint &key1 = pointCollection_[0]->getKeypoint(
        matches[ i ].trainIdx );
      const KeyPoint &key2 = queryPoints->getKeypoint(
        matches[ i ].queryIdx );
      srcP.at<float[ 2 ]>( 0,i )[ 0 ] = key1.pt.x;
      srcP.at<float[ 2 ]>( 0,i )[ 1 ] = key1.pt.y;
      destP.at<float[ 2 ]>( 0,i )[ 0 ] = key2.pt.x;
      destP.at<float[ 2 ]>( 0,i )[ 1 ] = key2.pt.y;
      status.push_back( 1 );
    }
    
    Mat fundam = cv::findFundamentalMat( srcP, destP, status, cv::FM_RANSAC, 1 );

    //refine the mathing :
    for( int i = 0; i < size_match; ++i ){
      if( status[ i ] == 0 )
      {
        status[ i ] = status[ --size_match ];
        status.pop_back( );
        matches[ i-- ] = matches[ size_match ];
        matches.pop_back( );
      }
    }
    //now using optical flow, improve this:

    const vector<KeyPoint>& keyPoints=pointCollection_[0]->getKeypoints( );
    const vector<KeyPoint>& keyPoints1=queryPoints->getKeypoints( );
    vector<cv::Point2f> keyPointsIn;
    vector<cv::Point2f> keyPointsOut;
    vector<float> error;

    cv::Mat img1 = pointCollection_[0]->getImage();
    cv::Mat img2 = queryPoints->getImage();
    CV_Assert( !img1.empty() && !img2.empty() );
    vector< int > corresponding_point;

    int nbIn = 0, nbOut = 0;
    for( size_t cpt = 0; cpt<keyPoints.size(); cpt++)
    {
      //is this keypoint in matches?
      int has_keypoint = -1;

      for( size_t cpt1 = 0; has_keypoint<0 && cpt1 < matches.size(); cpt1++ )
        has_keypoint = matches[ cpt1 ].trainIdx == cpt? cpt1 : -1;

      const KeyPoint &kp = keyPoints[cpt];
      bool point_added = false;
      if( has_keypoint<0 )
      {//TODO: find the closest point and set him the same displacement

        size_t nb_points = keyPoints.size();
        int idx_min = -1;
        float dist_min = 1e10;
        for(size_t i = 0; i<nb_points ; ++i)
        {
          if(cpt!=i)
          {
            const cv::KeyPoint& kp_i = keyPoints[i];
            float dist = sqrt( (kp.pt.x - kp_i.pt.x)*(kp.pt.x - kp_i.pt.x)
              + (kp.pt.y - kp_i.pt.y) * (kp.pt.y - kp_i.pt.y) );
            if( dist<dist_min )
            {//is this point in matches?
              has_keypoint = -1;
              for( size_t cpt2 = 0; has_keypoint<0 && cpt2 < matches.size(); cpt2++ )
                has_keypoint = matches[ cpt2 ].trainIdx == i? cpt2 : -1;

              if( has_keypoint>0 )
              {
                dist_min = dist;
                idx_min = has_keypoint;
              }
            }
          }
        }

        if( idx_min>0 )
        {
          size_t pt_idx = matches[ idx_min ].trainIdx;
          size_t pt_idx1 = matches[ idx_min ].queryIdx;
          const KeyPoint &kp_src = keyPoints[ pt_idx ];
          const KeyPoint &kp_dest = keyPoints1[ pt_idx1 ];
          float dx = kp_dest.pt.x - kp_src.pt.x,
            dy = kp_dest.pt.y - kp_src.pt.y;
          //if( kp.pt.x + dx>0 && kp.pt.x + dx<img2.cols &&
          //  kp.pt.y + dy>0 && kp.pt.y + dy<img2.rows )
          {
            keyPointsOut.push_back( cv::Point2f(kp.pt.x + dx, kp.pt.y + dy) );
            point_added = true;
          }
        }
      }
      else
      {
        size_t pt_idx = matches[ has_keypoint ].queryIdx;
        const KeyPoint &kp1 = keyPoints1[ pt_idx ];
        keyPointsOut.push_back( cv::Point2f(kp1.pt.x, kp1.pt.y) );
        point_added = true;
      }
      if( point_added )
      {
        nbIn++;
        keyPointsIn.push_back( cv::Point2f(kp.pt.x, kp.pt.y) );
        corresponding_point.push_back( cpt );
      }
    }
    cv::calcOpticalFlowPyrLK(img1, img2, keyPointsIn, keyPointsOut,
      status, error, cv::Size(31,31),4, cv::TermCriteria(
      cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30, 0.01),
      0.75, cv::OPTFLOW_USE_INITIAL_FLOW );
    queryPoints->free_descriptors( true );//descriptors are not needed now!
    
    std::vector<cv::DMatch> final_matches;
    //construct the DMatch vector (find the closest point in queryPoints)
    int idxPointsAdd = keyPoints1.size() - 1;
    vector<cv::KeyPoint> keypoints_to_add;
    vector< int > idx_to_add;
    for( size_t cpt = 0; cpt<keyPointsOut.size(); cpt++)
    {
      if( cpt>=status.size() || status[cpt]!=0 )
      {
        cv::Point2f& kp = keyPointsOut[ cpt ];
        size_t idx = queryPoints->getClosestKeypoint( kp );
        const cv::KeyPoint& kpClose = queryPoints->getKeypoint( idx );
        float dist = sqrt( (kpClose.pt.x - kp.x)*(kpClose.pt.x - kp.x)
          + (kpClose.pt.y - kp.y) * (kpClose.pt.y - kp.y) );
        if( dist<max_distance_ )
          final_matches.push_back( cv::DMatch(idx, corresponding_point[cpt], dist) );
      }
    }

    matches = final_matches;
    /*
    queryPoints->addKeypoints( keypoints_to_add );
    for( size_t cpt = 0; cpt<keypoints_to_add.size(); cpt++)
    {
      cv::KeyPoint& kp = keypoints_to_add[ cpt ];
      int idx_min = queryPoints->getClosestKeypoint( kp.pt );
      const cv::KeyPoint& kpClose = queryPoints->getKeypoint( idx_min );
      float dist = sqrt( (kpClose.pt.x - kp.pt.x)*(kpClose.pt.x - kp.pt.x)
        + (kpClose.pt.y - kp.pt.y) * (kpClose.pt.y - kp.pt.y) );
      if( dist<max_distance_ )
        matches.push_back( cv::DMatch(idx_min, idx_to_add[cpt], dist_min) );
    }*/
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