#include "EuclideanEstimator.h"
#include "StructureEstimator.h"
#include <opencv2/core/eigen.hpp>

#include "Visualizer.h"
#include "PCL_mapping.h"
#include <pcl/point_types.h>
#include "libmv/multiview/five_point.h"
#include "libmv/multiview/affine.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/robust_fundamental.h"
#include "libmv/multiview/robust_euclidean_resection.h"
#include "libmv/multiview/bundle.h"

#include <pcl/io/vtk_io.h>
#include <sstream>

using std::vector;
using cv::Ptr;

namespace OpencvSfM{
  //the next two functions are only for intern usage, no external interface...

  // Approximation of reprojection error;
  // The one from libmv don't use Mat2X as input
  double SampsonDistance2( const libmv::Mat &F,
    const libmv::Mat2X &x1, const libmv::Mat2X &x2 ) {
    double error_total= 0.0;
    libmv::Vec2 x1_i,x2_i;
    unsigned int n_points = x1.rows( );
    for( unsigned int i = 0; i < n_points ; ++i )
    {
      x1_i = x1.col( i );
      x2_i = x2.col( i );

      error_total += libmv::SampsonDistance( F, x1_i, x2_i );
    }

    return error_total;
  }

  //Idea from Snavely : Modeling the World from Internet Photo Collections
  //See with libmv team if such a function is usefull:
  double robust5Points( const libmv::Mat2X &x1, const libmv::Mat2X &x2,
    const libmv::Mat3 &K1, const libmv::Mat3 &K2,
    libmv::Mat3 &E )
  {
    unsigned int nPoints = x1.cols( );
    CV_DbgAssert( nPoints == x2.cols( ) );
    CV_DbgAssert( nPoints >= 5 );//need 5 points!

    cv::RNG& rng = cv::theRNG( );
    vector<int> masks( nPoints );
    double max_error = 1e9;

    int num_iter=0, max_iter=nPoints-5;
    for( num_iter=0; num_iter<max_iter; ++num_iter )
    {
      masks.clear( );
      int nb_vals=0;
      for ( unsigned int cpt = 0; cpt < nPoints; cpt++ ) {
        masks.push_back( 0 );
      }
      while( nb_vals < 5 )
      {
        int valTmp = rng( nPoints );
        if( masks[ valTmp ] == 0 )
        {
          masks[ valTmp ] = 1;
          nb_vals++;
        }
      }
      //create mask:
      libmv::Mat2X x1_tmp,x2_tmp;
      x1_tmp.resize( 2,nb_vals );
      x2_tmp.resize( 2,nb_vals );
      nb_vals=0;
      unsigned int i;
      for( i = 0; i<nPoints; ++i )
      {
        if( masks[ i ] != 0 )
        {
          x1_tmp( 0,nb_vals ) = x1( 0,i );
          x1_tmp( 1,nb_vals ) = x1( 1,i );
          x2_tmp( 0,nb_vals ) = x2( 0,i );
          x2_tmp( 1,nb_vals ) = x2( 1,i );
          nb_vals++;
        }
      }
      libmv::vector<libmv::Mat3, Eigen::aligned_allocator<libmv::Mat3> > Es(10);
      libmv::FivePointsRelativePose( x1_tmp,x2_tmp,&Es );
      unsigned int num_hyp = Es.size( );
      for ( i = 0; i < num_hyp; i++ ) {

        libmv::Mat3 F;
        libmv::FundamentalFromEssential( Es[ i ], K1, K2, &F );
        double error = SampsonDistance2( F, x1, x2 );

        if ( max_error > error ) {
          max_error = error;
          E = Es[ i ];
        }
      }
    }
    return max_error;
  }


  EuclideanEstimator::EuclideanEstimator( SequenceAnalyzer &sequence,
    vector<PointOfView>& cameras )
    :sequence_( sequence ),cameras_( cameras )
  {
    vector<PointOfView>::iterator itPoV=cameras.begin( );
    while ( itPoV!=cameras.end( ) )
    {
      addNewPointOfView( *itPoV );
      itPoV++;
    }
  }

  EuclideanEstimator::~EuclideanEstimator( void )
  {
    //TODO!!!!
  }

  void EuclideanEstimator::bundleAdjustement( )
  {
    //wrap the lourakis SBA:
    
    int n = point_computed_.size( ),   // number of points
      ncon = 0,// number of points (starting from the 1st) whose parameters should not be modified.
      m = 0,   // number of images (or camera)
      mcon = 0,// number of images (starting from the 1st) whose parameters should not be modified.
      cnp = 12,// number of parameters for ONE camera; e.g. 6 for Euclidean cameras
      //as we prefer speed versus memory, we use matrix for rotation, so +6
      pnp = 3;// number of parameters for ONE point; e.g. 3 for Euclidean points
    if( n < 3 )
      return;//we don't perform bundle as we don't have enough points...
    int i = 0, j = 0,
      nb_cam = camera_computed_.size( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );

    // Indexes of points corresponding to the projections:
    libmv::vector<libmv::Vecu>  x_ids;

    //because some points are sometime not visible:
    vector<int> idx_cameras;
    int nz_count = 0;
    for ( i = 0; i < nb_cam; ++i )
    {//for each camera:
      if( camera_computed_[ i ] )
      {
        idx_cameras.push_back(i);
        m++;//increament of camera count
        x_ids.push_back( libmv::Vecu() );

        int nb_projection = 0;
        unsigned int nb_points = point_computed_.size();
        for ( j = 0; j < nb_points; ++j )
        {//for each 3D point:
          if( point_computed_[ j ].containImage( i ) )
            nb_projection++;
        }
        x_ids[ m-1 ].resize( nb_projection );
        nz_count += nb_projection;
      }
    }
    
    //2D points:
    char *vmask = new char[ n*m ];//visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise.
    double *p = new double[m*cnp + n*pnp];//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
                   // aj are the image j parameters, bi are the i-th point parameters
    //order : rotation (Quaternion: 4), translation (3)

    double *x = new double[ 2*nz_count ];// measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
                   // x_ij is the projection of the i-th point on the j-th image.
                   // NOTE: some of the x_ij might be missing, if point i is not visible in image j;
                   // see vmask[i, j], max. size n*m*mnp

    //update each variable:
    int idx_visible = 0;
    double *p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      //extrinsic parameters only (intra are know in euclidean reconstruction)
      libmv::Mat3& r = rotations_[ idx_cam ];
      //add camera parameters to p:
      p_local[0] = r(0,0);p_local[1] = r(0,1);p_local[2] = r(0,2);
      p_local[3] = r(1,0);p_local[4] = r(1,1);p_local[5] = r(1,2);
      p_local[6] = r(2,0);p_local[5] = r(2,1);p_local[8] = r(2,2);

      p_local[9] = translations_[ idx_cam ](0);
      p_local[10] = translations_[ idx_cam ](1);
      p_local[11] = translations_[ idx_cam ](2);

      p_local+=cnp;
    }

    //now add the projections and 3D points:
    std::vector<int> myPoints;
    myPoints.reserve(m);
    idx_visible = 0;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      //2D projected points
      int j_visible = 0;
      for ( j = 0; j < n; ++j )
      {//for each 3D point:
        vmask[ j+(i*n) ] = point_computed_[ j ].containImage( idx_cam );
        if( vmask[ j+(i*n) ] )
        {
          cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
            point_computed_[ j_visible ].getIndexPoint( i ) );
          x[ idx_visible++ ] = pt.pt.x;
          x[ idx_visible++ ] = pt.pt.y;
        }

      }
    }
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      cv::Vec3d cv3DPoint = point_computed_[ i ];
      p_local[0] = cv3DPoint[ 0 ];
      p_local[1] = cv3DPoint[ 1 ];
      p_local[2] = cv3DPoint[ 2 ];
      p_local[3] = 1;//homogeneous coordinate...
      p_local += 4;
    }

    delete [] vmask;//visibility mask
    delete [] p;//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    delete [] x;// measurement vector
  }

  bool EuclideanEstimator::cameraResection( unsigned int image )
  {

    cv::Ptr< PointsToTrack > points_to_track = sequence_.getPoints( )[ image ];
    //the 3D points to use are stored in point_computed_...
    //extract 3D points and projections:
    vector<cv::Vec2d> x_im;
    vector<cv::Vec3d> X_w;
    libmv::Mat2X x_image;
    libmv::Mat3X X_world;

    // Selects only the reconstructed tracks observed in the image
    //for each points:
    unsigned int key_size = point_computed_.size( ),
      i = 0;
    vector<TrackOfPoints> matches;

    for ( i=0; i < key_size; ++i )
    {
      TrackOfPoints &track = point_computed_[ i ];
      if( track.containImage( image ) )
      {
        int idx = track.getIndexPoint( image );
        cv::KeyPoint p = points_to_track->getKeypoint( idx );
        x_im.push_back( cv::Vec2d( p.pt.x, p.pt.y ) );
        X_w.push_back( ( cv::Vec3d )track );
      }
    }

    //convert point in normalized camera coordinates
    //and convert to libmv structure:
    key_size = x_im.size( );
    cv::Ptr<Camera> device = cameras_[ image ].getIntraParameters( );
    vector<cv::Vec2d> x_im_norm = device->pixelToNormImageCoordinates( x_im );
    x_image.resize( 2, key_size );
    X_world.resize( 3, key_size );
    for ( i=0; i < key_size; ++i )
    {
      libmv::Vec2 tmpVec( x_im[ i ][ 0 ], x_im[ i ][ 1 ] );
      libmv::Vec3 tmpVec3D( X_w[ i ][ 0 ], X_w[ i ][ 1 ], X_w[ i ][ 2 ] );

      x_image.col( i ) = tmpVec;
      X_world.col( i ) = tmpVec3D;
    }

    libmv::Mat3 R;
    libmv::Vec3 t;
    double rms_inliers_threshold = 1;// in pixels
    libmv::vector<int> inliers;
    double score = libmv::EuclideanResectionEPnPRobust( x_image, X_world,
      intra_params_[ image ],
      rms_inliers_threshold, &R, &t, &inliers, 1e-3 );

    if( score > 5 || inliers.size( ) < (int)key_size / 4 )
      return false;//bad resection :(


    rotations_[ image ] = rotations_[ index_origin ] * R;
    translations_[ image ] = rotations_[ index_origin ] * t +
      translations_[ index_origin ];

    //update camera's structure:
    cv::Mat newRotation,newTranslation;
    cv::eigen2cv( rotations_[ image ], newRotation );
    cv::eigen2cv( translations_[ image ], newTranslation );
    cameras_[ image ].setRotationMatrix( newRotation );
    cameras_[ image ].setTranslationVector( newTranslation );

    //this camera is now computed:
    camera_computed_[ image ] = true;

    //Triangulate the points:
    StructureEstimator se( sequence_, this->cameras_ );

    vector<int> images_to_compute;
    key_size = camera_computed_.size( );
    for ( i=0; i < key_size; ++i )
    {
      if( camera_computed_[ i ] )
        images_to_compute.push_back( i );
    }

    point_computed_ = se.computeStructure( images_to_compute );

    return true;
  }

  
  void EuclideanEstimator::initialReconstruction( vector<TrackOfPoints>& tracks,
    int image1, int image2 )
  {
    CV_Assert( camera_computed_[ image1 ] );
    index_origin = image1;

    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );
    libmv::Mat3 E;
    Ptr<PointsToTrack> point_img1 = points_to_track[ image1 ];
    Ptr<PointsToTrack> point_img2 = points_to_track[ image2 ];
    //first extract points matches:
    libmv::Mat2X x1,x2;
    //for each points:
    unsigned int key_size = tracks.size( );
    unsigned int i;
    vector<TrackOfPoints> matches;

    for ( i=0; i < key_size; ++i )
    {
      TrackOfPoints &track = tracks[ i ];
      if( track.containImage( image1 ) && track.containImage( image2 ) )
        matches.push_back( track );
    }
    x1.resize( 2,matches.size( ) );
    x2.resize( 2,matches.size( ) );

    key_size = matches.size( );
    vector<cv::Vec2d> pointImg1,pointImg2;
    for ( i=0; i < key_size; ++i )
    {
      TrackOfPoints &track = matches[ i ];
      cv::DMatch match = track.toDMatch( image1, image2 );

      pointImg1.push_back( cv::Vec2d( point_img1->getKeypoint( match.trainIdx ).pt.x,
        point_img1->getKeypoint( match.trainIdx ).pt.y ) );
      pointImg2.push_back( cv::Vec2d( point_img2->getKeypoint( match.queryIdx ).pt.x,
        point_img2->getKeypoint( match.queryIdx ).pt.y ) );
    }
    vector<cv::Vec2d> pointNorm1 = cameras_[ image1 ].getIntraParameters( )->
      pixelToNormImageCoordinates( pointImg1 );
    vector<cv::Vec2d> pointNorm2 = cameras_[ image2 ].getIntraParameters( )->
      pixelToNormImageCoordinates( pointImg2 );
    key_size = pointNorm1.size( );
    for ( i=0; i < key_size; ++i )
    {
      x1( 0,i ) = -pointNorm1[ i ][ 0 ];
      x1( 1,i ) = -pointNorm1[ i ][ 1 ];
      x2( 0,i ) = -pointNorm2[ i ][ 0 ];
      x2( 1,i ) = -pointNorm2[ i ][ 1 ];
    }
    
    double error = robust5Points( x1, x2,
      intra_params_[ image1 ], intra_params_[ image2 ], E );


    //std::cout<<"E: "<<E<<std::endl;
    //std::cout<<"max_error: "<<error<<std::endl;


    //From this essential matrix extract relative motion:
    libmv::Mat3 R;
    libmv::Vec3 t;
    libmv::Vec2 x1Col, x2Col;
    x1Col << x1( 0,15 ), x1( 1,15 );
    x2Col << x2( 0,15 ), x2( 1,15 );
    bool ok = libmv::MotionFromEssentialAndCorrespondence( E,
      intra_params_[ image1 ], x1Col,
      intra_params_[ image2 ], x2Col,
      &R, &t );

    rotations_[ image2 ] = rotations_[ image1 ] * R;
    translations_[ image2 ] = rotations_[ image1 ] * t + translations_[ image1 ];

    //update camera's structure:
    cv::Mat newRotation,newTranslation;
    cv::eigen2cv( rotations_[ image2 ], newRotation );
    cv::eigen2cv( translations_[ image2 ], newTranslation );
    cameras_[ image2 ].setRotationMatrix( newRotation );
    cameras_[ image2 ].setTranslationVector( newTranslation );

    //this camera is now computed:
    camera_computed_[ image2 ] = true;

    //Triangulate the points:
    StructureEstimator se( sequence_, this->cameras_ );
    vector<int> images_to_compute;
    images_to_compute.push_back( image1 );
    images_to_compute.push_back( image2 );
    point_computed_ = se.computeStructure( images_to_compute );
    bundleAdjustement();
  }

  void proj_euclidean(int j, int i, double *aj, double *bj, double *xij,
    void* adata)
  {
    //as we do an euclidean bundle adjustement, the adata contain K.
    libmv::Mat3* K = (libmv::Mat3*)adata;
    Eigen::Quaterniond rot(aj[0], aj[1], aj[2], aj[3]);
    libmv::Vec3 translat(aj[4], aj[5], aj[6]);
  }

  void EuclideanEstimator::computeReconstruction( )
  {
    vector<TrackOfPoints>& tracks = sequence_.getTracks( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );
    ImagesGraphConnection &images_graph = sequence_.getImgGraph( );
    double ransac_threshold = 0.4 * sequence_.getImage( 0 ).rows / 100.0;
    //now create the graph:

    int img1,img2;
    int nbMatches = images_graph.getHighestLink( img1,img2 );
    vector<ImageLink> bestMatches;
    images_graph.getOrderedLinks( bestMatches, 100, nbMatches );
    double min_inliners=1e7;
    int index_of_min=0;
    cv::Mat minFundamental;
    for( unsigned int cpt=0;cpt<bestMatches.size( );cpt++ )
    {
      //construct the homography and choose the worse matches:
      //( see Snavely "Modeling the World from Internet Photo Collections" )
      std::vector<cv::Point2f> pointsImg1, pointsImg2;
      vector<uchar> status;
      points_to_track[ bestMatches[ cpt ].imgSrc ]->getKeyMatches( tracks,
        bestMatches[ cpt ].imgDest, pointsImg1 );
      points_to_track[ bestMatches[ cpt ].imgDest ]->getKeyMatches( tracks,
        bestMatches[ cpt ].imgSrc, pointsImg2 );

      //compute the homography:
      cv::findHomography( pointsImg1,pointsImg2,status,CV_RANSAC,
        ransac_threshold );
      //count the inliner points:
      double inliners=0;
      for( unsigned int i=0;i<status.size( );++i )
      {
        if( status[ i ] != 0 )
          inliners++;
      }
      double percent_inliner = inliners/static_cast<double>( pointsImg1.size( ) );
      if( percent_inliner < min_inliners )
      {
        min_inliners = percent_inliner;
        index_of_min = cpt;
        minFundamental = cv::findFundamentalMat( pointsImg1, pointsImg2,
          status, cv::FM_LMEDS );
      }
    }
    //we will start the reconstruction using bestMatches[ index_of_min ]
    //to avoid degenerate cases such as coincident cameras
    img1 = bestMatches[ index_of_min ].imgSrc;
    img2 = bestMatches[ index_of_min ].imgDest;

    vector<int> images_computed;
    images_computed.push_back( img1 );
    images_computed.push_back( img2 );
    camera_computed_[ img1 ] = true;
    initialReconstruction( tracks, img1, img2 );

    //now we have updated the position of the camera which take img2
    //and 3D estimation from these 2 first cameras...
    //Find for other cameras position:
    vector<ImageLink> images_close;
    int nbIter = 0 ;
    //while( nbMatches>10 && nbIter<10 )
    {
      nbIter++;
      images_close.clear( );
      while ( images_close.size( ) == 0 )
      {
        images_graph.getImagesRelatedTo( img1,images_close,
          nbMatches * 0.8 - 10 );
        images_graph.getImagesRelatedTo( img2,images_close,
          nbMatches * 0.8 - 10 );
        nbMatches = nbMatches * 0.8 - 10;
      }

      //for each images links from images_close, comptute the camera position:
      for( unsigned int cpt=0;cpt<images_close.size( );cpt++ )
      {
        //We don't want to compute twice the same camera position:
        int new_id_image = -1;
        if( !camera_computed_ [ images_close[ cpt ].imgSrc ] )
          new_id_image = images_close[ cpt ].imgSrc;
        if( !camera_computed_ [ images_close[ cpt ].imgDest ] )
        {
          if ( new_id_image >= 0 )
          {
            //the two images are new!
            //skeep them for now...
            //TODO : make this work ; )
            new_id_image = -1;
          }
          else
            new_id_image = images_close[ cpt ].imgDest;
        }

        if( new_id_image >= 0 )
        {
          cameraResection( new_id_image );
        }
      }
      // Performs a bundle adjustment
      //bundleAdjustement( );
    }//*/
    vector<cv::Vec3d> tracks3D;
    vector<TrackOfPoints>::iterator itTrack=point_computed_.begin( );
    while ( itTrack != point_computed_.end( ) )
    {
      tracks3D.push_back( ( cv::Vec3d )( *itTrack ) );
      itTrack++;
    }

    //////////////////////////////////////////////////////////////////////////
    // Open 3D viewer and add point cloud

    Visualizer debugView ( "Debug viewer" );
    debugView.add3DPoints( tracks3D );

    for( unsigned int i = 0; i<cameras_.size( ) ; ++i )
      if( camera_computed_[i] )
      {
        std::stringstream cam_name;
        cam_name<<"Cam"<< ( i+1 );
        debugView.addCamera( cameras_[ i ],
          cam_name.str() );
      }



    debugView.runInteract( );
  }
}
