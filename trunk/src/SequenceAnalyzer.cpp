#include "SequenceAnalyzer.h"

#include "config.h" //SEMAPHORE

#include INCLUDE_MUTEX
#include <boost/thread/thread.hpp>

#include <iostream>
#include <sstream>
using cv::Ptr;
using cv::Mat;
using cv::DMatch;
using cv::KeyPoint;
using std::vector;
using cv::Point3d;

namespace OpencvSfM{
  SequenceAnalyzer::SequenceAnalyzer( MotionProcessor input_sequence,
    cv::Ptr<cv::FeatureDetector> feature_detector,
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor,
    cv::Ptr<PointsMatcher> match_algorithm )
    :match_algorithm_( match_algorithm ),
    feature_detector_( feature_detector ),
    descriptor_extractor_( descriptor_extractor )
  {
    //only finite sequences can be used:
    CV_DbgAssert( input_sequence.isBidirectional( ) );
    //go back to the begining:
    input_sequence.setProperty( CV_CAP_PROP_POS_FRAMES,0 );

    //load entire sequence! Can be problematic but if a user want to have
    //more controls, he can use the other constructor...

    int nbFrame=0;
    Mat currentImage=input_sequence.getFrame( );
    while ( !currentImage.empty( ) )// && nbFrame<50 )
    {

      Ptr<PointsToTrack> ptrPoints_tmp( new PointsToTrackWithImage (
        nbFrame, currentImage, Mat( ), feature_detector, descriptor_extractor ));
      ptrPoints_tmp->computeKeypointsAndDesc( );

      points_to_track_.push_back( ptrPoints_tmp );
      images_.push_back( currentImage );
      nbFrame++;
      currentImage=input_sequence.getFrame( );
    }
  }

  SequenceAnalyzer::SequenceAnalyzer(
    vector< Ptr< PointsToTrack > > &points_to_track,
    Ptr<PointsMatcher> match_algorithm,
    const std::vector<cv::Mat> &images )
    :images_( images ),points_to_track_( points_to_track ),
    match_algorithm_( match_algorithm )
  {
  }

  using cv::DescriptorMatcher;
  using cv::FlannBasedMatcher;
  //by default, use flann based matcher
  SequenceAnalyzer::SequenceAnalyzer( std::vector<cv::Mat> &images, cv::FileNode file )
    :images_( images ),
    match_algorithm_( new PointsMatcher( Ptr<DescriptorMatcher>( new FlannBasedMatcher( ) )) )
  {
    read( file,*this );
  }

  SequenceAnalyzer::~SequenceAnalyzer( void )
  {
  }

  void SequenceAnalyzer::addNewImage( cv::Mat image, Ptr<PointsToTrack> points )
  {
    if( points.empty( ) )
    {
      CV_DbgAssert( !feature_detector_.empty( ) &&
        !descriptor_extractor_.empty( ) );
      int nbFrame = points_to_track_.size( );
      Ptr<PointsToTrack> ptrPoints_tmp( new PointsToTrackWithImage (
        nbFrame, image, Mat( ), feature_detector_, descriptor_extractor_ ));
      ptrPoints_tmp->computeKeypointsAndDesc( );

      points_to_track_.push_back( ptrPoints_tmp );
    }
    else
      points_to_track_.push_back( points );

    images_.push_back( image );
  }

  //Boost thread:
  struct MatchingThread{
    unsigned int i;
    vector< Ptr< PointsMatcher > >::iterator matches_it;
    Ptr<SequenceAnalyzer> seq_analyser;

    static vector< Ptr< PointsMatcher > >::iterator end_matches_it;
    static vector<Mat> masks;
    static unsigned int mininum_points_matches;
    //semaphore to synchronize threads:
    CREATE_STATIC_MUTEX( thread_concurr );
    CREATE_STATIC_MUTEX( add_to_track );

    //////////////////////////////////////////////////////////////////////////

    MatchingThread(Ptr<SequenceAnalyzer> seq_analyser,unsigned int i,
      vector< Ptr< PointsMatcher > >::iterator matches_it)
    {
      this->i = i;
      this->matches_it = matches_it;
      this->seq_analyser = seq_analyser;
      this->seq_analyser.addref();//avoid ptr deletion...
    }

    void operator()()
    {
      Ptr<PointsMatcher> point_matcher = ( *matches_it );
      if( point_matcher.empty() )
        return;
      vector< Ptr< PointsMatcher > >::iterator matches_it1 = matches_it+1;
      unsigned int j=i+1;
      while ( matches_it1 != end_matches_it )
      {
        Ptr<PointsMatcher> point_matcher1 = ( *matches_it1 );
        vector<DMatch> matches_i_j;

        point_matcher->crossMatch( point_matcher1, matches_i_j, masks );

//////////////////////////////////////////////////////////////////////////
//For now we use fundamental function from OpenCV but soon use libmv !

        //First compute points matches:
        int size_match=matches_i_j.size( );
        vector<cv::Point2f> srcP;
        vector<cv::Point2f> destP;
        vector<uchar> status;

        //vector<KeyPoint> points1 = point_matcher->;
        for( int cpt = 0; cpt < size_match; ++cpt ){
          const KeyPoint &key1 = point_matcher1->getKeypoint(
            matches_i_j[ cpt ].queryIdx );
          const KeyPoint &key2 = point_matcher->getKeypoint(
            matches_i_j[ cpt ].trainIdx );
          srcP.push_back( cv::Point2f( key1.pt.x,key1.pt.y ) );
          destP.push_back( cv::Point2f( key2.pt.x,key2.pt.y ) );
          status.push_back( 1 );
        }
        Mat fundam = cv::findFundamentalMat( srcP, destP, status, cv::FM_RANSAC );

        unsigned int nbErrors = 0, nb_iter=0;
        //refine the mathing :
        size_match = status.size( );
        for( int cpt = 0; cpt < size_match; ++cpt ){
          if( status[ cpt ] == 0 )
          {
            status[ cpt ] = status[ --size_match ];
            status.pop_back( );
            srcP[ cpt ] = srcP[ size_match ];
            srcP.pop_back( );
            destP[ cpt ] = destP[ size_match ];
            destP.pop_back( );
            matches_i_j[ cpt-- ] = matches_i_j[ size_match ];
            matches_i_j.pop_back( );
            ++nbErrors;
          }
        }

        while( nbErrors > 50 && nb_iter < 8 &&
          matches_i_j.size( ) > mininum_points_matches )
        {
          fundam = cv::findFundamentalMat( srcP, destP, status, cv::FM_RANSAC, 1.5 );
          
          //refine the mathing :
          nbErrors =0 ;
          size_match = status.size( );
          for( int cpt = 0; cpt < size_match; ++cpt ){
            if( status[ cpt ] == 0 )
            {
              status[ cpt ] = status[ --size_match ];
              status.pop_back( );
              srcP[ cpt ] = srcP[ size_match ];
              srcP.pop_back( );
              destP[ cpt ] = destP[ size_match ];
              destP.pop_back( );
              matches_i_j[ cpt-- ] = matches_i_j[ size_match ];
              matches_i_j.pop_back( );
              ++nbErrors;
            }
          }
          nb_iter++;
        };
        /*
        //refine the mathing :
        matches_i_j.clear( );
        point_matcher->matchWithFundamental( point_matcher1,fundam,images_[ i ],
          matches_i_j,masks );*/
        

//////////////////////////////////////////////////////////////////////////
        if( matches_i_j.size( ) > mininum_points_matches && nb_iter < 8 )
        {
          P_MUTEX( add_to_track );
          seq_analyser->addMatches( matches_i_j,i,j );
          std::clog<<"find "<<matches_i_j.size( )<<
            " matches between "<<i<<" "<<j<<std::endl;
          V_MUTEX( add_to_track );
        }else
        {
          std::clog<<"can't find matches between "<<i<<" "<<j<<std::endl;
        }
        j++;
        matches_it1++;
      }
      V_MUTEX( MatchingThread::thread_concurr );//wake up waiting thread
    }
  };
  vector< Ptr< PointsMatcher > >::iterator MatchingThread::end_matches_it;
  vector<Mat> MatchingThread::masks;
  unsigned int MatchingThread::mininum_points_matches=50;

  DECLARE_MUTEX( MatchingThread::thread_concurr );
  DECLARE_MUTEX( MatchingThread::add_to_track );

  //////////////////////////////////////////////////////////////////////////

  void SequenceAnalyzer::computeMatches( )
  {
    //First compute missing features descriptors:
    vector< Ptr< PointsToTrack > >::iterator it =
      points_to_track_.begin( );
    vector< Ptr< PointsToTrack > >::iterator end_iter =
      points_to_track_.end( );
    while ( it != end_iter )
    {
      Ptr<PointsToTrack> points_to_track_i=( *it );

      points_to_track_i->computeKeypointsAndDesc( false );

      it++;
    }

    //here, all keypoints and descriptors are computed.
    //Now create and train the matcher:
    it=points_to_track_.begin( );
    //We skip previous matcher already computed:
    it+=matches_.size( );
    while ( it != end_iter )
    {
      Ptr<PointsToTrack> points_to_track_i = ( *it );
      Ptr<PointsMatcher> point_matcher = match_algorithm_->clone( true );
      point_matcher->add( points_to_track_i );
      point_matcher->train( );
      matches_.push_back( point_matcher );

      it++;
    }

    //Now we are ready to match each picture with other:
    vector<Mat> masks;
    vector< Ptr< PointsMatcher > >::iterator matches_it = matches_.begin( );

    MatchingThread::end_matches_it = matches_.end( );
    MatchingThread::mininum_points_matches = mininum_points_matches;
    unsigned int nb_proc = boost::thread::hardware_concurrency();
    INIT_SEMAPHORE( MatchingThread::thread_concurr,nb_proc );
    INIT_MUTEX( MatchingThread::add_to_track );

    unsigned int i=0;

    while ( matches_it != MatchingThread::end_matches_it )
    {
      //can we start a new thread?
      P_MUTEX( MatchingThread::thread_concurr );
      //create local values for the thead:
      MatchingThread match_thread(this, i, matches_it);
      //start the thread:
      boost::thread myThread(match_thread);

      i++;
      matches_it++;
    }
    for(int wait_endThread = 0; wait_endThread<nb_proc-1 ; ++wait_endThread)
      P_MUTEX( MatchingThread::thread_concurr );//wait for last threads
  }

  void SequenceAnalyzer::keepOnlyCorrectMatches( )
  {
    unsigned int tracks_size = tracks_.size( );
    unsigned int index=0;

    while ( index < tracks_size )
    {
      if( tracks_[ index ].getNbTrack( ) <= mininum_image_matches )
      {
        //problem with this track, too small to be consistent
        tracks_size--;
        tracks_[ index ]=tracks_[ tracks_size ];
        tracks_.pop_back( );
        index--;
      }
      index++;
    }
  }

  void SequenceAnalyzer::addMatches( vector<DMatch> &newMatches,
    unsigned int img1, unsigned int img2 )
  {
    //add to tracks_ the new matches:

    vector<DMatch>::iterator match_it = newMatches.begin( );
    vector<DMatch>::iterator match_it_end = newMatches.end( );

    while ( match_it != match_it_end )
    {
      DMatch &point_matcher = ( *match_it );

      bool is_found=false;
      vector<TrackOfPoints>::iterator tracks_it = tracks_.begin( );
      while ( tracks_it != tracks_.end( ) && !is_found )
      {
        TrackOfPoints& track = ( *tracks_it );

        if( track.containPoint( img1,point_matcher.trainIdx ))
        {
          track.addMatch( img2,point_matcher.queryIdx );
          is_found=true;
        }
        else
          tracks_it++;
      }
      if( !is_found )
      {
        //it's a new point match, create a new track:
        TrackOfPoints newTrack;
        newTrack.addMatch( img1,point_matcher.trainIdx );
        newTrack.addMatch( img2,point_matcher.queryIdx );
        tracks_.push_back( newTrack );
      }

      match_it++;
    }
  }

  void SequenceAnalyzer::addTracks( vector<TrackOfPoints> &newTracks )
  {

    vector<TrackOfPoints>::iterator match_it = newTracks.begin( ),
      match_it_end = newTracks.end( );

    while ( match_it != match_it_end )
    {
      tracks_.push_back( *match_it );

      match_it++;
    }
  }

  void SequenceAnalyzer::showTracks( int timeBetweenImg )
  {
    if( points_to_track_.size( ) == 0 )
      return;//nothing to do...

    //First compute missing features descriptors:
    unsigned int it=0,it1=0;
    unsigned int end_iter = points_to_track_.size( ) - 1 ;
    if( images_.size( ) - 1 < end_iter )
      end_iter = images_.size( ) - 1;
    while ( it < end_iter )
    {
      it1=it+1;
      while ( it1 < end_iter )
      {
        vector<DMatch> matches_to_print;
        //add to matches_to_print only points of img it and it+1:

        vector<TrackOfPoints>::iterator match_it = tracks_.begin( );
        vector<TrackOfPoints>::iterator match_it_end = tracks_.end( );

        while ( match_it != match_it_end )
        {
          if( match_it->containImage( it ) &&
            match_it->containImage( it1 ) )
          {
            matches_to_print.push_back( match_it->toDMatch( it,it1 ));
          }
          match_it++;
        }

        Mat firstImg=images_[ it ];
        Mat outImg;

        PointsMatcher::drawMatches( firstImg, points_to_track_[ it ]->getKeypoints( ),
          points_to_track_[ it1 ]->getKeypoints( ),
          matches_to_print, outImg,
          cv::Scalar::all( -1 ), cv::Scalar::all( -1 ), vector<char>( ),
          cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      imshow( "showTracks",outImg );
      cv::waitKey( timeBetweenImg );

        it1++;
      }
      it++;
    }
    cvDestroyWindow( "showTracks" );
  }

  void SequenceAnalyzer::read( const cv::FileNode& node, SequenceAnalyzer& me )
  {
    std::string myName=node.name( );
    if( myName != "SequenceAnalyzer" )
      CV_Error( CV_StsError, "SequenceAnalyzer FileNode is not correct!" );

    if( node.empty( ) || !node.isMap( ) )
      CV_Error( CV_StsError, "SequenceAnalyzer FileNode is not correct!" );

    int nb_pictures = ( int ) node[ "nbPictures" ];
    //initialisation of all empty vectors
    for( int i=0; i<nb_pictures; i++ )
    {
      Ptr<PointsToTrack> ptt = Ptr<PointsToTrack>( new PointsToTrack( i ));
      me.points_to_track_.push_back( ptt );

      Ptr<PointsMatcher> p_m = Ptr<PointsMatcher>( new PointsMatcher(
        *me.match_algorithm_ ) );
      p_m->add( ptt );

      me.matches_.push_back( p_m );
    }

    cv::FileNode node_TrackPoints = node[ "TrackPoints" ];

    //tracks are stored in the following form:
    //list of track where a track is stored like this:
    // nbPoints idImage1 point1  idImage2 point2 ...
    if( node_TrackPoints.empty( ) || !node_TrackPoints.isSeq( ) )
      CV_Error( CV_StsError, "SequenceAnalyzer FileNode is not correct!" );
    cv::FileNodeIterator it = node_TrackPoints.begin( ),
      it_end = node_TrackPoints.end( );
    while( it != it_end )
    {
      cv::FileNode it_track = ( *it )[ 0 ];
      int nbPoints,track_consistance;
      it_track[ "nbPoints" ] >> nbPoints;
      it_track[ "track_consistance" ] >> track_consistance;
      bool has_3d_point = false;
      it_track[ "has_3d_position" ] >> has_3d_point;
      TrackOfPoints track;
      if( has_3d_point )
      {
        cv::Vec3d point;
        point[ 0 ] = it_track[ "point3D_triangulated" ][ 0 ];
        point[ 1 ] = it_track[ "point3D_triangulated" ][ 1 ];
        point[ 2 ] = it_track[ "point3D_triangulated" ][ 2 ];
        track.point3D = Ptr<cv::Vec3d>( new cv::Vec3d( point ) );
      }
      cv::FileNodeIterator itPoints = it_track[ "list_of_points" ].begin( ),
        itPoints_end = it_track[ "list_of_points" ].end( );
      while( itPoints != itPoints_end )
      {
        int idImage;
        cv::KeyPoint kpt;
        idImage = ( *itPoints )[ 0 ];
        itPoints++;
        kpt.pt.x = ( *itPoints )[ 0 ];
        kpt.pt.y = ( *itPoints )[ 1 ];
        kpt.size = ( *itPoints )[ 2 ];
        kpt.angle = ( *itPoints )[ 3 ];
        kpt.response = ( *itPoints )[ 4 ];
        kpt.octave = ( *itPoints )[ 5 ];
        kpt.class_id = ( *itPoints )[ 6 ];

        unsigned int point_index = me.points_to_track_[ idImage ]->
          addKeypoint( kpt );
        track.addMatch( idImage,point_index );

        itPoints++;
      }
      track.track_consistance = track_consistance;
      me.tracks_.push_back( track );
      it++;
    }
  }

  void SequenceAnalyzer::write( cv::FileStorage& fs, const SequenceAnalyzer& me )
  {
    vector<TrackOfPoints>::size_type key_size = me.tracks_.size( );
    int idImage=-1, idPoint=-1;
    
    fs << "SequenceAnalyzer" << "{";
    fs << "nbPictures" << ( int )me.points_to_track_.size( );
    fs << "TrackPoints" << "[";
    for ( vector<TrackOfPoints>::size_type i=0; i < key_size; i++ )
    {
      const TrackOfPoints &track = me.tracks_[ i ];
      unsigned int nbPoints = track.getNbTrack( );
      if( nbPoints > 0 )
      {
        fs << "{" << "nbPoints" << ( int )nbPoints;
        fs << "track_consistance" << track.track_consistance;
        fs << "has_3d_position" << ( !track.point3D.empty( ) );
        if( !track.point3D.empty( ) )
          fs << "point3D_triangulated" << *track.point3D;
        fs << "list_of_points" << "[:";
        for ( unsigned int j = 0; j < nbPoints ; j++ )
        {
          idImage = track.images_indexes_[ j ];
          idPoint = track.point_indexes_[ j ];
          if( idImage>=0 && idPoint>=0 )
          {
            fs << idImage;
            fs  << "[:";

            const cv::KeyPoint kpt = me.points_to_track_[ idImage ]->
              getKeypoints( )[ idPoint ];
            cv::write( fs, kpt.pt.x );
            cv::write( fs, kpt.pt.y );
            cv::write( fs, kpt.size );
            cv::write( fs, kpt.angle );
            cv::write( fs, kpt.response );
            cv::write( fs, kpt.octave );
            cv::write( fs, kpt.class_id );
            fs << "]" ;
          }
        }
        fs << "]" << "}" ;
      }
    }
    fs << "]" << "}";
  }

  void SequenceAnalyzer::constructImagesGraph( )
  {
    images_graph_.initStructure( points_to_track_.size( ) );
    
    //for each points:
    vector<TrackOfPoints>::size_type key_size = tracks_.size( );
    vector<TrackOfPoints>::size_type i;

    for ( i=0; i < key_size; i++ )
    {
      TrackOfPoints &track = tracks_[ i ];
      unsigned int nviews = track.images_indexes_.size( );
      
      for( unsigned int cpt=0;cpt<nviews;cpt++ )
      {
        unsigned int imgSrc = track.images_indexes_[ cpt ];
        for( unsigned int cpt1=imgSrc+1;cpt1<nviews;cpt1++ )
        {
          images_graph_.addLink( imgSrc, track.images_indexes_[ cpt1 ] );
        }
      }
    }
  }

  vector<cv::Vec3d> SequenceAnalyzer::get3DStructure( )
  {
    vector<cv::Vec3d> out_vector;
    vector<TrackOfPoints>::iterator itTrack=tracks_.begin( );
    while ( itTrack != tracks_.end( ) )
    {
      if( !itTrack->point3D.empty( ) )
        out_vector.push_back( ( cv::Vec3d )( *itTrack ) );
      itTrack++;
    }
    return out_vector;
  }
}
