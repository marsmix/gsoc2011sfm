
#include "macro.h" //SFM_EXPORTS

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>

#include "PointsMatcher.h"
#include "PointsToTrack.h"

namespace OpencvSfM{
  using cv::Mat;
  using cv::Ptr;
  using std::vector;
  using cv::KeyPoint;
  using cv::DMatch;
  using cv::Size;
  using cv::Rect;

  PointsMatcher::PointsMatcher( const cv::Ptr<cv::DescriptorMatcher>& matcher )
  {
    CV_DbgAssert( !matcher.empty( ) );
    matcher_ = matcher;
    INIT_MUTEX( thread_concurr );
  }

  PointsMatcher::PointsMatcher( )
  {
    matcher_ = NULL;
    INIT_MUTEX( thread_concurr );
  }

  PointsMatcher::PointsMatcher( const PointsMatcher& copy )
  {
    INIT_MUTEX( thread_concurr );
    pointCollection_=copy.pointCollection_;
    matcher_ = copy.matcher_->clone( true );
  }

  PointsMatcher::~PointsMatcher( void )
  {
    //matcher_ is freed thanks to cv::Ptr
    //idem for pointCollection_
  }
  const cv::KeyPoint &PointsMatcher::getKeypoint( int numKey ) const
  {
    CV_DbgAssert( pointCollection_.size( )>0 );
    return pointCollection_[ 0 ]->getKeypoint( numKey );
  }

  void PointsMatcher::add( Ptr<PointsToTrack> pointCollection )
  {
    P_MUTEX( thread_concurr );
    pointCollection_.push_back( pointCollection );
    V_MUTEX( thread_concurr );
  }

  void PointsMatcher::clear( )
  {
    P_MUTEX( thread_concurr );
    matcher_->clear( );
    for(size_t i = 0; i<pointCollection_.size(); ++i)
      pointCollection_[i]->free_descriptors();
    V_MUTEX( thread_concurr );
  }

  void PointsMatcher::train( )
  {
    P_MUTEX( thread_concurr );
    matcher_->clear( );

    vector<Mat> pointsDesc;
    Ptr<PointsToTrack> pointCollection;
    vector< Ptr< PointsToTrack > >::iterator it =
      pointCollection_.begin( ),
      it_end = pointCollection_.end( );
    while( it != it_end )
    {
      pointCollection = *it;
      pointCollection->computeKeypointsAndDesc( false );

      pointsDesc.push_back( pointCollection->getDescriptors( ) );

      pointCollection->free_descriptors();//save memory...

      it++;
    }


    matcher_->add( pointsDesc );
    matcher_->train( );
    V_MUTEX( thread_concurr );
  }

  bool PointsMatcher::isMaskSupported( )
  {
    return matcher_->isMaskSupported( );
  }

  void PointsMatcher::match( cv::Ptr<PointsToTrack> queryPoints,
    std::vector<cv::DMatch>& matches,
    const std::vector<cv::Mat>& masks )
  {
    train( );

    P_MUTEX( thread_concurr );
    queryPoints->computeKeypointsAndDesc( false );

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints( );
    Mat descMat=queryPoints->getDescriptors( );

    CV_DbgAssert( !keyPoints.empty( ) );
    CV_DbgAssert( !descMat.empty( ) );

    matcher_->match( descMat, matches, masks );
    V_MUTEX( thread_concurr );

    queryPoints->free_descriptors();
  }

  void PointsMatcher::knnMatch(  Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, int knn,
    const vector<Mat>& masks, bool compactResult )
  {
    train( );

    P_MUTEX( thread_concurr );
    queryPoints->computeKeypointsAndDesc( false );

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints( );
    Mat descMat=queryPoints->getDescriptors( );

    CV_DbgAssert( !keyPoints.empty( ) );
    CV_DbgAssert( !descMat.empty( ) );

    matcher_->knnMatch( descMat, matches, knn, masks, compactResult );
    V_MUTEX( thread_concurr );

    queryPoints->free_descriptors();
  }

  void PointsMatcher::radiusMatch( cv::Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, float maxDistance,
    const vector<Mat>& masks, bool compactResult )
  {
    train( );

    P_MUTEX( thread_concurr );
    queryPoints->computeKeypointsAndDesc( false );

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints( );
    Mat descMat=queryPoints->getDescriptors( );

    CV_DbgAssert( !keyPoints.empty( ) );
    CV_DbgAssert( !descMat.empty( ) );

    matcher_->radiusMatch( descMat, matches, maxDistance, masks, compactResult );
    V_MUTEX( thread_concurr );

    queryPoints->free_descriptors();
  }

  bool PointsMatcher::empty( ) const
  {
    return pointCollection_.empty( ) ||  matcher_->empty( );
  }

  Ptr<PointsMatcher> PointsMatcher::clone( bool emptyTrainData )
  {
    P_MUTEX( thread_concurr );
    Ptr<PointsMatcher> outPointMatcher=new PointsMatcher( matcher_->clone( emptyTrainData ) );
    if( !emptyTrainData )
      outPointMatcher->pointCollection_=pointCollection_;
    V_MUTEX( thread_concurr );

    return outPointMatcher;
  }

  void PointsMatcher::crossMatch( Ptr<PointsMatcher> otherMatcher,
    vector<DMatch>& matches,
    const std::vector<cv::Mat>& masks )
  {
    train( );

    //TODO: for now this function only work with matcher having only one picture
    CV_DbgAssert( this->pointCollection_.size( )==1 );
    CV_DbgAssert( otherMatcher->pointCollection_.size( )==1 );

    if( matches.empty( ) )
    {
      //as we don't have matches for img1 -> img2, compute them:
      this->match( otherMatcher->pointCollection_[ 0 ], matches, masks );
    }


    //now construct the vector of DMatch, but in the other way ( 2 -> 1 ):
    vector<DMatch> matchesOtherWay;
    P_MUTEX( thread_concurr );
    otherMatcher->match( pointCollection_[ 0 ], matchesOtherWay, masks );
    V_MUTEX( thread_concurr );
    //now check for reciprocity:
    size_t nbPoints = matches.size( ),
      nbPoints1 = matchesOtherWay.size( );
    for( size_t i=0; i<nbPoints; i++ )
    {
      DMatch d1=matches[ i ];
      bool found = false;
      for( size_t j=0; j<nbPoints1 && !found ; ++j )
      {
        DMatch d2=matchesOtherWay[ j ];
        found = (d1.trainIdx == d2.queryIdx) && (d1.queryIdx == d2.trainIdx);
      }
      if( !found )
      {
        //remove the current match!
        nbPoints--;//because we have a match less!
        matches[ i ]=matches[ nbPoints ];
        matches.pop_back( );
        i--;//because we have to test this one!
      }
    }
    matchesOtherWay.clear( );
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
      outImg.create( size, CV_MAKETYPE( img1.depth( ), 3 ) );

      if( img1.type( ) == CV_8U )
        cv::cvtColor( img1, outImg, CV_GRAY2BGR );
      else
        img1.copyTo( outImg );
    }

    // draw keypoints
    if( !( flags & cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS ) )
    {
      drawKeypoints( outImg, keypoints1, outImg, singlePointColor,
        flags + cv::DrawMatchesFlags::DRAW_OVER_OUTIMG );
    }

    cv::RNG& rng = cv::theRNG( );
    bool isRandMatchColor = matchColor == cv::Scalar::all( -1 );
    vector<DMatch>::size_type total_size=matches1to2.size( );
    // draw matches
    for( vector<DMatch>::size_type m = 0; m < total_size; m++ )
    {
      int i1 = matches1to2[ m ].trainIdx;
      int i2 = matches1to2[ m ].queryIdx;
      if( matchesMask.empty( ) || matchesMask[ m ] )
      {
        const KeyPoint &kp1 = keypoints1[ i1 ], &kp2 = keypoints2[ i2 ];

        cv::Scalar color = isRandMatchColor ?
          cv::Scalar( rng( 256 ), rng( 256 ), rng( 256 ) ) : matchColor;

        cv::Point center1( cvRound( kp1.pt.x ), cvRound( kp1.pt.y ) );
        cv::Point center2( cvRound( kp2.pt.x ), cvRound( kp2.pt.y ) );
        int radius = 1;
        cv::circle( outImg, center1, radius, color, 1, CV_AA );
        //cv::circle( outImg, center2, radius, color, 1, CV_AA );
        cv::line( outImg, center1, center2, color, 1, CV_AA );

      }
    }

  }


  void PointsMatcher::read( const cv::FileNode& node,
    PointsMatcher& points )
  {
    std::string myName=node.name( );
    if( myName != "PointsMatcher" )
      return;//this node is not for us...

    //This matcher need only point coordinates as the matcher is already loaded:

    if( node.empty( ) || !node.isSeq( ) )
      CV_Error( CV_StsError, "PointsMatcher FileNode is not correct!" );

    int numImg=0;
    cv::FileNodeIterator it = node.begin( ), it_end = node.end( );
    while( it != it_end )
    {
      Ptr<PointsToTrack> ptt_tmp = Ptr<PointsToTrack>( new PointsToTrack( numImg ));
      cv::FileNode node_tmp = ( *it )[ "PointsToTrack" ];
      PointsToTrack::read( node_tmp, *ptt_tmp );
      points.pointCollection_.push_back( ptt_tmp );
      it++;
      numImg++;
    }
  }

  void PointsMatcher::write ( cv::FileStorage& fs,
    const PointsMatcher& points )
  {
    vector<PointsToTrack>::size_type key_size = points.pointCollection_.size( );

    fs << "PointsMatcher" << "[";

    for ( vector<PointsToTrack>::size_type i=0; i < key_size; i++ )
    {
      PointsToTrack::write( fs, *points.pointCollection_[ i ] );
    }
    fs << "]";
  }

  
  PointsMatcherOpticalFlow::PointsMatcherOpticalFlow( std::string name_of_algo,
    double dist_allowed )
  {
    transform(name_of_algo.begin(), name_of_algo.end(),
      name_of_algo.begin(),tolower);
    id_algo = 0;
    max_distance = dist_allowed;

    if( name_of_algo == "opticalflowpyrlk" )
      id_algo = 1;
    if( name_of_algo == "opticalflowfarneback" )
      id_algo = 2;
    if( name_of_algo == "opticalflowbm" )
      id_algo = 3;
    if( name_of_algo == "opticalflowhs" )
      id_algo = 4;
    if( name_of_algo == "opticalflowlk" )
      id_algo = 5;
  }

  void PointsMatcherOpticalFlow::clear( )
  {
    //nothing to do...
  }

  void PointsMatcherOpticalFlow::train( )
  {
    //nothing to do...
  }

  bool PointsMatcherOpticalFlow::isMaskSupported( )
  {
    return false;
  }

  void PointsMatcherOpticalFlow::match( cv::Ptr<PointsToTrack> queryPoints,
    std::vector<cv::DMatch>& matches,
    const std::vector<cv::Mat>& masks )
  {
    P_MUTEX( thread_concurr );

    if(pointCollection_[0]->getKeypoints( ).empty())
      pointCollection_[0]->computeKeypoints();
    const vector<KeyPoint>& keyPoints=pointCollection_[0]->getKeypoints( );
    vector<cv::Point2f> keyPointsIn;
    vector<cv::Point2f> keyPointsOut;
    vector<uchar> status;
    vector<float> error;

    cv::Mat img1 = pointCollection_[0]->getImage();
    cv::Mat img2 = queryPoints->getImage();
    CV_Assert( !img1.empty() && !img2.empty()  );

    for( size_t cpt = 0; cpt<keyPoints.size(); cpt++)
    {
      const KeyPoint &kp = keyPoints[cpt];
      keyPointsIn.push_back( cv::Point2f(kp.pt.x, kp.pt.y) );
    }

    switch( id_algo )
    {
    case 1://1:  OpticalFlowPyrLK
      cv::calcOpticalFlowPyrLK(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;/*
    case 2://2:  OpticalFlowFarneback
      cv::OpticalFlowFarneback(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    case 3://3:  OpticalFlowBM
      cv::OpticalFlowBM(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    case 4://4:  OpticalFlowHS
      cv::OpticalFlowHS(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    case 5://5:  OpticalFlowLK
      cv::OpticalFlowLK(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    default:;*/
    }

    //construct the DMatch vector (find the closest point in queryPoints)
    if(queryPoints->getKeypoints( ).empty())
      queryPoints->computeKeypoints();
    const vector< KeyPoint >& points = queryPoints->getKeypoints();
    for( size_t cpt = 0; cpt<keyPointsOut.size(); cpt++)
    {
      if( status[cpt]!=0 )
      {
        float dist_min = 1e10;
        int idx_min = 0;
        for(size_t i = 0; i<points.size() ; ++i)
        {
          const cv::KeyPoint& kp = points[i];
          float dist = sqrt( (keyPointsOut[cpt].x - kp.pt.x)*(keyPointsOut[cpt].x - kp.pt.x)
            + (keyPointsOut[cpt].y - kp.pt.y)*(keyPointsOut[cpt].y - kp.pt.y) );
          if(dist<dist_min)
          {
            dist_min = dist;
            idx_min = i;
          }
        }
        if( dist_min<max_distance )
        matches.push_back( cv::DMatch(idx_min, cpt, dist_min) );//not really the correct dist...
      }
    }
    V_MUTEX( thread_concurr );
  }

  void PointsMatcherOpticalFlow::knnMatch(  Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, int knn,
    const vector<Mat>& masks, bool compactResult )
  {
    if( knn!=1 )
      CV_Error( CV_StsError, "not yet implemented..." );

    vector<DMatch> matchesTmp;
    match( queryPoints, matchesTmp, masks );
    matches.push_back(matchesTmp);
  }

  void PointsMatcherOpticalFlow::radiusMatch( cv::Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, float maxDistance,
    const vector<Mat>& masks, bool compactResult )
  {
    P_MUTEX( thread_concurr );

    if(pointCollection_[0]->getKeypoints( ).empty())
      pointCollection_[0]->computeKeypoints();
    const vector<KeyPoint>& keyPoints=pointCollection_[0]->getKeypoints( );
    vector<cv::Point2f> keyPointsIn;
    vector<cv::Point2f> keyPointsOut;
    vector<char> status;
    vector<double> error;

    cv::Mat img1 = pointCollection_[0]->getImage();
    cv::Mat img2 = queryPoints->getImage();
    CV_Assert( !img1.empty() && !img2.empty()  );

    for( size_t cpt = 0; cpt<keyPoints.size(); cpt++)
    {
      const KeyPoint &kp = keyPoints[cpt];
      keyPointsIn.push_back( cv::Point2f(kp.pt.x, kp.pt.y) );
    }

    switch( id_algo )
    {
    case 1://1:  OpticalFlowPyrLK
      cv::calcOpticalFlowPyrLK(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;/*
    case 2://2:  OpticalFlowFarneback
      cv::OpticalFlowFarneback(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    case 3://3:  OpticalFlowBM
      cv::OpticalFlowBM(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    case 4://4:  OpticalFlowHS
      cv::OpticalFlowHS(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    case 5://5:  OpticalFlowLK
      cv::OpticalFlowLK(img1, img2, keyPointsIn, keyPointsOut,
        status, error );
      break;
    default:;*/
    }

    //construct the DMatch vector (find the closest point in queryPoints)
    if(queryPoints->getKeypoints( ).empty())
      queryPoints->computeKeypoints();
    const std::vector< cv::KeyPoint >& points = queryPoints->getKeypoints();
    for( size_t cpt = 0; cpt<keyPointsOut.size(); cpt++)
    {
      if( status[cpt]!=0 )
      {
        vector<DMatch> matchesTmp;
        for(size_t i = 0; i<points.size() ; ++i)
        {
          const cv::KeyPoint& kp = points[i];
          float dist = sqrt( (keyPointsOut[cpt].x - kp.pt.x)*(keyPointsOut[cpt].x - kp.pt.x)
            + (keyPointsOut[cpt].y - kp.pt.y)*(keyPointsOut[cpt].y - kp.pt.y) );
          if( dist<maxDistance )
          {
            matchesTmp.push_back( cv::DMatch(i, cpt, dist) );//not really the correct dist...
          }
        }
        matches.push_back( matchesTmp );//not really the correct dist...
      }
      else
        matches.push_back( vector<DMatch>() );//empty match!
    }
    V_MUTEX( thread_concurr );
  }

  bool PointsMatcherOpticalFlow::empty( ) const
  {
    return pointCollection_.empty( );
  }

  Ptr<PointsMatcher> PointsMatcherOpticalFlow::clone( bool emptyTrainData )
  {
    P_MUTEX( thread_concurr );
    PointsMatcherOpticalFlow* outPointMatcher =
      new PointsMatcherOpticalFlow( "void" );
    outPointMatcher->id_algo = this->id_algo;
    if( !emptyTrainData )
      outPointMatcher->pointCollection_ = pointCollection_;
    V_MUTEX( thread_concurr );

    return outPointMatcher;
  }

}
