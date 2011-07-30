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
  {
    CV_DbgAssert( !matcher.empty( ) );
    matcher_ = matcher;
  }

  PointsMatcher::PointsMatcher( const PointsMatcher& copy )
  {
    pointCollection_=copy.pointCollection_;
    matcher_ = copy.matcher_->clone( true );
  }

  PointsMatcher::~PointsMatcher( void )
  {
    //TODO!!!!
  }

  void PointsMatcher::add( Ptr<PointsToTrack> pointCollection )
  {
    pointCollection_.push_back( pointCollection );

  }

  void PointsMatcher::clear( )
  {
    matcher_->clear( );
    pointCollection_.clear( );
  }

  void PointsMatcher::train( )
  {
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

      Mat descMat=pointCollection->getDescriptors( );

      pointsDesc.push_back( descMat );

      it++;
    }


    matcher_->add( pointsDesc );
    matcher_->train( );
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

    queryPoints->computeKeypointsAndDesc( false );

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints( );
    Mat descMat=queryPoints->getDescriptors( );

    CV_DbgAssert( !keyPoints.empty( ) );
    CV_DbgAssert( !descMat.empty( ) );

    matcher_->match( descMat, matches, masks );
  }

  void PointsMatcher::knnMatch(  Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, int knn,
    const vector<Mat>& masks, bool compactResult )
  {
    train( );

    queryPoints->computeKeypointsAndDesc( false );

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints( );
    Mat descMat=queryPoints->getDescriptors( );

    CV_DbgAssert( !keyPoints.empty( ) );
    CV_DbgAssert( !descMat.empty( ) );

    matcher_->knnMatch( descMat, matches, knn, masks, compactResult );
  }

  void PointsMatcher::radiusMatch( cv::Ptr<PointsToTrack> queryPoints,
    vector<vector<DMatch> >& matches, float maxDistance,
    const vector<Mat>& masks, bool compactResult )
  {
    train( );

    queryPoints->computeKeypointsAndDesc( false );

    vector<KeyPoint> keyPoints=queryPoints->getKeypoints( );
    Mat descMat=queryPoints->getDescriptors( );

    CV_DbgAssert( !keyPoints.empty( ) );
    CV_DbgAssert( !descMat.empty( ) );

    matcher_->radiusMatch( descMat, matches, maxDistance, masks, compactResult );
  }

  bool PointsMatcher::empty( ) const
  {
    return pointCollection_.empty( ) ||  matcher_->empty( );
  }

  Ptr<PointsMatcher> PointsMatcher::clone( bool emptyTrainData ) const
  {
    Ptr<PointsMatcher> outPointMatcher=new PointsMatcher( matcher_->clone( emptyTrainData ) );
    if( !emptyTrainData )
      outPointMatcher->pointCollection_=pointCollection_;

    return outPointMatcher;
  }

  void PointsMatcher::matchWithFundamental( Ptr<PointsMatcher> otherMatcher,
    cv::Mat fundamentalMat,
    cv::Mat img1,
    vector<DMatch>& matches,
    const std::vector<cv::Mat>& masks )
  {
    train( );

    //TODO: for now this function only work with matcher having only one picture
    CV_DbgAssert( this->pointCollection_.size( )==1 );
    CV_DbgAssert( otherMatcher->pointCollection_.size( )==1 );

    const vector<cv::KeyPoint>& src_points = pointCollection_[ 0 ]->getKeypoints( );
    const vector<cv::KeyPoint>& dest_points = otherMatcher->pointCollection_[ 0 ]->
      getKeypoints( );

    matches.clear( );
    vector< vector< DMatch > > matchesKNN;
    knnMatch( otherMatcher->pointCollection_[ 0 ], matchesKNN, 4, masks );


    //now construct the vector of DMatch, but in the other way ( 2 -> 1 ):
    vector< vector< DMatch > > matchesKNNOtherWay;
    otherMatcher->knnMatch( pointCollection_[ 0 ], matchesKNNOtherWay, 2, masks );
    //now check for reciprocity:
    unsigned int nbPoints=matchesKNN.size( );
    for( unsigned int i=0; i<nbPoints; ++i )
    {
      bool isFound=false;
      double distMin=1e15;
      unsigned int sizeVect=matchesKNN[ i ].size( ),
        minMatch = 0;

      const cv::KeyPoint p = dest_points[ i ];
      cv::Mat x( 3,1,CV_64F );
      x.at<double>( 0,0 ) = p.pt.x;
      x.at<double>( 1,0 ) = p.pt.y;
      x.at<double>( 2,0 ) = 1;
      // See page 287 equation ( 11.9 ) of HZ.
      cv::Mat F_x = fundamentalMat.t( ) * x;
      double l_a = F_x.at<double>( 0,0 );
      double l_b = F_x.at<double>( 1,0 );
      double l_c = F_x.at<double>( 2,0 );
      double nu = l_a*l_a + l_b*l_b;
      nu = nu ? 1./sqrt( nu ) : 1.;
      l_a *= nu; l_b *= nu; l_c *= nu;
      nu = 1.0 / sqrt( l_a * l_a + l_b * l_b );

      for( unsigned int j=0; j<sizeVect; ++j )
      {
        DMatch d1=matchesKNN[ i ][ j ];
        cv::KeyPoint p1 = src_points[ d1.trainIdx ];
        double dist = abs( l_a * p1.pt.x + l_b * p1.pt.y + l_c ) * nu;
        if( dist<distMin )
        {
          distMin = dist;
          minMatch = j;
        }
      }
      if( distMin < 1 )
      {
        DMatch d_best=matchesKNN[ i ][ minMatch ];
        sizeVect=matchesKNNOtherWay[ d_best.trainIdx ].size( );
        for( unsigned int j=0; j<sizeVect && !isFound; ++j )
        {
          DMatch d2=matchesKNNOtherWay[ d_best.trainIdx ][ j ];

          if( d2.trainIdx == d_best.queryIdx )
          {
            isFound = true;
            matches.push_back( d_best );
          }
        }
      }
    }
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
      match( otherMatcher->pointCollection_[ 0 ], matches, masks );
    }


    //now construct the vector of DMatch, but in the other way ( 2 -> 1 ):
    vector<DMatch> matchesOtherWay;
    otherMatcher->match( pointCollection_[ 0 ], matchesOtherWay, masks );
    //now check for reciprocity:
    unsigned int nbPoints=matches.size( );
    for( unsigned int i=0; i<nbPoints; i++ )
    {
      DMatch d1=matches[ i ];
      DMatch d2=matchesOtherWay[ d1.trainIdx ];
      if( d2.trainIdx!=d1.queryIdx )
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
      fs  << "{";
      PointsToTrack::write( fs, *points.pointCollection_[ i ] );
      fs  << "}";
    }
    fs << "]";
  }
}
