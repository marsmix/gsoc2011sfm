#include "PointsToTrack.h"
#include "TracksOfPoints.h"

using cv::Mat;
using cv::Scalar;
using cv::KeyPoint;
using cv::Point;
using std::vector;
using cv::line;
using cv::circle;

namespace OpencvSfM{
  int PointsToTrack::glob_number_images_ = 0;

  PointsToTrack::PointsToTrack( int corresponding_image,
    std::vector<KeyPoint> keypoints, Mat descriptors )
    :keypoints_( keypoints ),descriptors_( descriptors )
  {
    if ( corresponding_image>=0 )
      corresponding_image_ = corresponding_image;
    else
      corresponding_image_ = PointsToTrack::glob_number_images_;

    PointsToTrack::glob_number_images_++;
    nb_workers_ = 0;
    INIT_MUTEX(worker_exclusion);
  }

  void PointsToTrack::free_descriptors( bool force )
  {
    P_MUTEX(worker_exclusion);
    if(nb_workers_<=1 || force)
    {//if equal to 0, descriptors_ is already empty...
      nb_workers_ = 0;
      descriptors_.release( );
    }
    else
      nb_workers_--;
    V_MUTEX(worker_exclusion);
  }

  PointsToTrack::~PointsToTrack( void )
  {
    keypoints_.clear( );
    descriptors_.release( );
    PointsToTrack::glob_number_images_--;
  }

  int PointsToTrack::computeKeypointsAndDesc( bool forcing_recalculation )
  {
    P_MUTEX(worker_exclusion);
    nb_workers_++;
    if( !forcing_recalculation )
    {
      bool need_keypoints = keypoints_.empty( );
      bool need_descriptors = need_keypoints || descriptors_.empty( );
      if( need_keypoints )
        impl_computeKeypoints_();
      cv::KeyPointsFilter::runByKeypointSize( keypoints_, 3 );
      if( need_descriptors )
      {
        impl_computeDescriptors_();
      }
    }
    else
    {
      impl_computeKeypoints_();
      impl_computeDescriptors_();
    }

    V_MUTEX(worker_exclusion);
    return keypoints_.size( );
  }


  int PointsToTrack::computeKeypoints( )
  {
    P_MUTEX(worker_exclusion);
    impl_computeKeypoints_();
    cv::KeyPointsFilter::runByKeypointSize( keypoints_, 3 );
    V_MUTEX(worker_exclusion);
    return keypoints_.size( );
  }

  void PointsToTrack::computeDescriptors( )
  {
    P_MUTEX(worker_exclusion);
    nb_workers_++;
    impl_computeDescriptors_();
    V_MUTEX(worker_exclusion);
  }

  void PointsToTrack::addKeypoints( std::vector<cv::KeyPoint> keypoints,cv::Mat descriptors/*=cv::Mat( )*/,bool computeMissingDescriptor/*=false*/ )
  {
    P_MUTEX(worker_exclusion);
    //add the keypoints to the end of our points vector:
    this->keypoints_.insert( this->keypoints_.end( ),keypoints.begin( ),keypoints.end( ) );


    cv::KeyPointsFilter::runByKeypointSize( keypoints_, 3 );
    V_MUTEX(worker_exclusion);

    if( !computeMissingDescriptor )
    {
      if( !descriptors_.empty( ) )
      {
        Mat newDescriptors( this->keypoints_.size( ), this->descriptors_.cols,
          this->descriptors_.type( ) );
        newDescriptors(
          cv::Rect( 0, 0, this->descriptors_.cols,this->descriptors_.rows ) ) =
          this->descriptors_;

        newDescriptors( cv::Rect( 0, this->descriptors_.rows,
          this->descriptors_.cols,descriptors.rows ) ) = descriptors;

        this->descriptors_=newDescriptors;
      }
    }
    else
    {
      this->computeDescriptors( );
    }
  }
  void PointsToTrack::printPointsOnImage( const Mat &image, Mat& outImg, const Scalar& color/*=Scalar::all( -1 )*/, int flags/*=DrawMatchesFlags::DEFAULT*/ ) const
  {
    if( outImg.empty( ) )
      outImg=image.clone( );
    cv::drawKeypoints( image, keypoints_, outImg, color, flags );
  }
  void PointsToTrack::read( const cv::FileNode& node, PointsToTrack& points )
  {
    std::string myName=node.name( );
    if( myName != "PointsToTrack" )
      return;//this node is not for us...
    cv::FileNode node_keypoints = node[ "keypoints" ];
    if( node_keypoints.empty( ) )
      CV_Error( CV_StsError, "PointsToTrack FileNode is not correct!" );

    cv::FileNodeIterator it = node_keypoints.begin( ), it_end = node_keypoints.end( );
    while( it != it_end )
    {
      KeyPoint kpt;
      it >> kpt.pt.x >> kpt.pt.y >> kpt.size >> kpt.angle >> kpt.response >> kpt.octave >> kpt.class_id;
      points.keypoints_.push_back( kpt );
      //it++ is not needed as the >> operator increment automatically it!
    }

    cv::FileNode node_descriptors = node[ "descriptors" ];
    if( node_descriptors.empty( ) )
      CV_Error( CV_StsError, "PointsToTrack FileNode is not correct!" );
    node_descriptors >> points.descriptors_;

    CV_Assert( points.descriptors_.rows == points.keypoints_.size() );

    //as the loaded PointsToTrack can't recompute the descriptors_, set the nbworkers
    //to a high value (this will disable free_descriptors to release descriptor ;)
    points.nb_workers_ = 1e5;

    cv::FileNode node_colors = node[ "colors" ];
    if( !node_colors.empty( ) )
    {
      cv::FileNodeIterator it = node_colors.begin( ), it_end = node_colors.end( );
      while( it != it_end )
      {
        int color;
        it >> color;
        points.RGB_values_.push_back( color );
        //it++ is not needed as the >> operator increment automatically it!
      }
    }
  };
  void PointsToTrack::write( cv::FileStorage& fs, const PointsToTrack& keypoints )
  {
    PointsToTrack ppt = keypoints;
    //as we want to save them, compute keypoints and descriptor:
    ppt.computeKeypointsAndDesc();

    fs << "{" << "PointsToTrack" << "{";
    cv::write( fs, "keypoints", ppt.keypoints_ );

    fs << "descriptors" << ppt.descriptors_;
    fs << "colors" <<"[:";
    unsigned int size_max = ppt.RGB_values_.size( );
    for(unsigned int i=0; i<size_max; ++i )
    {
      fs << (int)(ppt.RGB_values_[i] & 0x00FFFFFF);
    }
    fs << "]" << "}" << "}";
  }

  void PointsToTrack::getKeyMatches( const std::vector<TrackOfPoints>& matches,
    int otherImage, std::vector<cv::Point2f>& pointsVals ) const
  {
    //for each points:
    vector<TrackOfPoints>::size_type key_size = matches.size( );
    vector<TrackOfPoints>::size_type i;

    for ( i=0; i < key_size; i++ )
    {
      const TrackOfPoints &track = matches[ i ];

      if( track.containImage( corresponding_image_ ) &&
        track.containImage( otherImage ) )
      {
        const KeyPoint &kp = keypoints_[ track.getPointIndex( corresponding_image_ ) ];
        pointsVals.push_back( cv::Point2f( kp.pt.x,kp.pt.y ));
      }
    }
  }

  size_t PointsToTrack::getClosestKeypoint( cv::Point2f point )
  {
    P_MUTEX(worker_exclusion);
    size_t nb_points = keypoints_.size(),
      idx_min = 0;
    float dist_min = 1e10;
    for(size_t i = 0; i<nb_points ; ++i)
    {
      const cv::KeyPoint& kp = keypoints_[i];
      float dist = sqrt( (point.x - kp.pt.x)*(point.x - kp.pt.x)
        + (point.y - kp.pt.y) * (point.y - kp.pt.y) );
      if( dist<dist_min )
      {
        dist_min = dist;
        idx_min = i;
      }
    }
    V_MUTEX(worker_exclusion);
    return idx_min;
  };
}
