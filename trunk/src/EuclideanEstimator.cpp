

#include <pcl/point_types.h>
#include "libmv/multiview/five_point.h"
#include "libmv/multiview/affine.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/robust_fundamental.h"
#include "libmv/multiview/focal_from_fundamental.h"
#include "opencv2/core/eigen.hpp"

#include <Eigen/Eigenvalues>

#include <pcl/io/vtk_io.h>
#include <sstream>

#include "EuclideanEstimator.h"
#include "StructureEstimator.h"
#include "Camera.h"
#include "Visualizer.h"
#include "PCL_mapping.h"
#include "bundle_related.h"

using std::vector;
using cv::Ptr;

namespace OpencvSfM{
  //the next two functions are only for intern usage, no external interface...

  /** Approximation of reprojection error;
  * The one from libmv don't use Mat2X as input
  */
  double SampsonDistance2( const libmv::Mat &F,
    const libmv::Mat2X &x1, const libmv::Mat2X &x2 ) {
    double error_total= 0.0;
    unsigned int n_points = x1.cols( );
    for( unsigned int i = 0; i < n_points ; ++i )
      error_total += libmv::SymmetricEpipolarDistance( F, x1.col( i ), x2.col( i ) );

    return error_total/(double)n_points;
  }

  /**Idea from Snavely : Modeling the World from Internet Photo Collections
  * See with libmv team if such a function is usefull:
  */
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

    int num_iter=0, max_iter=MIN( 2500, nPoints*(nPoints-5) );
    for( num_iter=0; num_iter<max_iter; ++num_iter )
    {
      masks.assign( nPoints, 0 );
      int nb_vals=0;
      //choose 5 random points:
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
    index_origin = 0;
  }

  EuclideanEstimator::~EuclideanEstimator( void )
  {
    //TODO!!!!
  }

  void EuclideanEstimator::addNewPointOfView( const PointOfView& camera )
  {
    libmv::Mat3 intra_param;
    cv::Ptr<Camera> intra=camera.getIntraParameters( );
    //transpose because libmv needs intra params this way...
    cv::cv2eigen( intra->getIntraMatrix( ).t(), intra_param );
    intra_params_.push_back( intra_param );
    libmv::Mat3 rotation_mat;
    cv::cv2eigen( camera.getRotationMatrix( ), rotation_mat );
    rotations_.push_back( rotation_mat );
    libmv::Vec3 translation_vec;
    cv::cv2eigen( camera.getTranslationVector( ), translation_vec );
    translations_.push_back( translation_vec );
    camera_computed_.push_back( false );
  }

  void EuclideanEstimator::bundleAdjustement( )
  {
    //wrap the lourakis SBA:
    
    unsigned int n = point_computed_.size( ),   // number of points
      ncon = 0,// number of points (starting from the 1st) whose parameters should not be modified.
      m = 0,   // number of images (or camera)
      mcon = 1,// number of cameras (starting from the 1st) whose parameters should not be modified.
      cnp = 6,// number of parameters for ONE camera; e.g. 6 for Euclidean cameras
      //use only vector part of quaternion to enforce the unit lenght...
      pnp = 3,// number of parameters for ONE 3D point; e.g. 3 for Euclidean points
      mnp = 2;// number of parameters for ONE projected point; e.g. 2 for Euclidean points

    unsigned int i = 0, j = 0,
      nb_cam = camera_computed_.size( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );

    //because some points are sometime not visible:
    vector<int> idx_cameras;
    idx_cameras.push_back( index_origin );

    libmv::vector< libmv::Mat3 > intra_p;
    std::vector<bool> pointOK;
    int nbPoints = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      //test if at least 2 views see this point:
      int nbCam = 0;
      for(size_t k =0; k<nb_cam; ++k)
      {
        if(camera_computed_[ k ] && point_computed_[ j ].containImage( k ))
          nbCam++;
      }
      pointOK.push_back( nbCam>=2 );
      if(pointOK[j])
        nbPoints++;
    }

    int nz_count = 0;
    for ( i = 0; i < nb_cam; ++i )
    {//for each camera:
      if( camera_computed_[ i ] )
      {
        if( i!=index_origin )//index_origin is already added...
          idx_cameras.push_back(i);
        m++;//increament of camera count

        int nb_projection = 0;
        for ( j = 0; j < n; ++j )
        {//for each 3D point:
          if( pointOK[j])
          {
            if( point_computed_[ j ].containImage( i ) )
              nb_projection++;
          }
        }
        nz_count += nb_projection;
      }
    }
    n=nbPoints;
    
    //2D points:
    char *vmask = new char[ n*m ];//visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise.
    double *p = new double[m*cnp + n*pnp];//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
                   // aj are the image j parameters, bi are the i-th point parameters

    double *x = new double[ 2*nz_count ];// measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
                   // x_ij is the projection of the i-th point on the j-th image.
                   // NOTE: some of the x_ij might be missing, if point i is not visible in image j;
                   // see vmask[i, j], max. size n*m*mnp

    libmv::vector< Eigen::Quaterniond > init_rotation;
    libmv::vector< libmv::Vec3 > init_translat;
    //update each variable:
    int idx_visible = 0;
    double *p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      intra_p.push_back( intra_params_[idx_cam] );
      //extrinsic parameters only (intra are know in euclidean reconstruction)
      init_rotation.push_back( (Eigen::Quaterniond)rotations_[ idx_cam ] );
      init_translat.push_back( translations_[ idx_cam ] );
      //add camera parameters to p:
      //as this is rotation, the quaternion's length is unity. Only 3 values are needed.
      //4th value equal:
      //sqrt(1.0 - quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2]));

      p_local[0] = 0; p_local[1] = 0; p_local[2] = 0;

      p_local[3] = 0; p_local[4] = 0; p_local[5] = 0;

      p_local+=cnp;
    }

    //now add the projections and 3D points:
    idx_visible = 0;
    int j_real = 0;
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        for ( i=0; i < m; ++i )
        {//for each camera:
          int idx_cam = idx_cameras[i];
          vmask[ i+j_real*m ] = point_computed_[ j ].containImage( idx_cam );
          if( vmask[ i+j_real*m ] )
          {
            cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
              point_computed_[ j ].getPointIndex( idx_cam ) );
            x[ idx_visible++ ] = pt.pt.x;
            x[ idx_visible++ ] = pt.pt.y;
          }
        }
        j_real++;
      }
    }
    double* points3D_values = p_local;
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        cv::Ptr<cv::Vec3d> cv3DPoint = point_computed_[ j ].get3DPosition();
        if(cv3DPoint.empty())
        {
          *(p_local++) = 0;
          *(p_local++) = 0;
          *(p_local++) = 0;
        }
        else
        {
          *(p_local++) = (*cv3DPoint)[ 0 ];
          *(p_local++) = (*cv3DPoint)[ 1 ];
          *(p_local++) = (*cv3DPoint)[ 2 ];
        }
      }
    }

    bundle_datas data(intra_p,init_rotation,init_translat,
      cnp, pnp, mnp,ncon, mcon);
    data.points3D = points3D_values;

    //////////////////////////////////////////////////////////////////////////
//#define PRINT_DEBUG
#ifdef PRINT_DEBUG
    //Debug compare projected point vs estimated point:
    idx_visible = 0;
    double max_distance = 0;
    double max_depth = 0;
    j_real = 0;
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        for ( i=0; i < m; ++i )
        {//for each camera:
          //2D projected points
          if( vmask[ i+j_real*m ] )
          {
            cv::Vec3d& cv3DPoint = point_computed_[ j ];
            //cout<<"Vec3d : "<< cv3DPoint[ 0 ]<<", "<< cv3DPoint[ 1 ]<<", "<< cv3DPoint[ 2 ]<<endl;
            int idx_cam = idx_cameras[i];
            cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
              point_computed_[ j ].getPointIndex( idx_cam ) );
            cv::Vec2d proj = cameras_[ idx_cam ].project3DPointIntoImage(cv3DPoint);/*
            cout<<pt.pt.x<<","<<pt.pt.y<<" -> ";
            cout<<proj[0]<<","<<proj[1]<<endl;*/
            max_distance += (pt.pt.x - proj[0])*(pt.pt.x - proj[0]) +
              (pt.pt.y - proj[1])*(pt.pt.y - proj[1]);

            libmv::Vec3 X(cv3DPoint[0],cv3DPoint[1],cv3DPoint[2]);
            max_depth += abs( (rotations_[ idx_cam ]*X)(2) + translations_[ idx_cam ](2) );
          }
        }
        j_real++;
      }
      //system("pause");
    }
#endif
    ////////////////////////////////////////////////////////////////////////*/

    //TUNING PARAMETERS:
    int itmax = 10000;        //max iterations
    int verbose = 1;
    double opts[SBA_OPTSSZ] = {
      0.00001,		//Tau
      1e-20,		//E1
      1e-20,		//E2
      0,		//E3 average reprojection error
      0		//E4 relative reduction in the RMS reprojection error
    };

    double info[SBA_INFOSZ];
    
    //use sba library
    int iter = sba_motstr_levmar_x(n, ncon, m, mcon, vmask, p, cnp, pnp, x, NULL, mnp,
        img_projsRTS_x, img_projsRTS_jac_x, (void*)&data, itmax, 0, opts, info);

    std::cout<<"SBA ("<<nz_count<<") returned in "<<iter<<" iter, reason "<<info[6]
    <<", error "<<info[1]<<" [initial "<< info[0]<<"]\n";
    if(iter>1)
    {
    //set new values:
    m = idx_cameras.size();
    n = point_computed_.size( );
    idx_visible = 0;
    p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      //extrinsic parameters only (intra are know in euclidean reconstruction)

      Eigen::Quaterniond rot_init = data.rotations[i];
      double c1 = p_local[0];
      double c2 = p_local[1];
      double c3 = p_local[2];
      double coef=(1.0 - c1*c1 - c2*c2 - c3*c3 );
      if( coef>0 )
        coef = sqrt( coef );
      else//problem with this rotation...
      {
        coef = 0;
        Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
        quat_delta.normalize();
        c1=quat_delta.x(); c2=quat_delta.y(); c3=quat_delta.z();
        coef = quat_delta.w();
      }

      Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
      Eigen::Quaterniond rot_total = quat_delta * rot_init;
      //add camera parameters to p:
      rotations_[ idx_cam ] = rot_total.toRotationMatrix();

      translations_[ idx_cam ](0) += p_local[3];
      translations_[ idx_cam ](1) += p_local[4];
      translations_[ idx_cam ](2) += p_local[5];

      //update camera's structure:
      cv::Mat newRotation,newTranslation;
      cv::eigen2cv( rotations_[ idx_cam ], newRotation );
      cv::eigen2cv( translations_[ idx_cam ], newTranslation );
      cameras_[ idx_cam ].setRotationMatrix( newRotation );
      cameras_[ idx_cam ].setTranslationVector( newTranslation );

      p_local+=cnp;
    }
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        cv::Vec3d cv3DPoint;
        cv3DPoint[ 0 ] = *(p_local++);
        cv3DPoint[ 1 ] = *(p_local++);
        cv3DPoint[ 2 ] = *(p_local++);
        point_computed_[ j ].set3DPosition( cv3DPoint );
      }
    }

    //////////////////////////////////////////////////////////////////////////
#ifdef PRINT_DEBUG
    //Debug compare projected point vs estimated point:
    idx_visible = 0;
    //2D projected points
    double max_distance_1 = 0;
    double max_depth1 = 0;
    j_real = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      if( pointOK[j] )
      {
      for ( i=0; i < m; ++i )
      {//for each camera:
        if( vmask[ i+j_real*m ] )
        {
          cv::Vec3d& cv3DPoint = point_computed_[ j ];
          //cout<<"Vec3d : "<< cv3DPoint[ 0 ]<<", "<< cv3DPoint[ 1 ]<<", "<< cv3DPoint[ 2 ]<<endl;
          int idx_cam = idx_cameras[i];
          cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
            point_computed_[ j ].getPointIndex( idx_cam ) );
          cv::Vec2d proj = cameras_[ idx_cam ].project3DPointIntoImage(cv3DPoint);/*
          cout<<pt.pt.x<<","<<pt.pt.y<<" -> ";
          cout<<proj[0]<<","<<proj[1]<<endl;*/
          max_distance_1 += (pt.pt.x - proj[0])*(pt.pt.x - proj[0]) +
            (pt.pt.y - proj[1])*(pt.pt.y - proj[1]);
          libmv::Vec3 X(cv3DPoint[0],cv3DPoint[1],cv3DPoint[2]);
          max_depth1 += abs( (rotations_[ idx_cam ]*X)(2) + translations_[ idx_cam ](2) );
        }
      }
      j_real++;
    }
      //system("pause");
    }
    cout<< (max_distance)<<"  ; "<<(max_distance_1)<<endl;
    cout<< (max_depth)<<"  ; "<<(max_depth1)<<endl;
#endif
    ////////////////////////////////////////////////////////////////////////*/

    }

    delete [] vmask;//visibility mask
    delete [] p;//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    delete [] x;// measurement vector
  }

  bool EuclideanEstimator::cameraResection( unsigned int image, int max_reprojection )
  {
    //wrap the lourakis SBA:
    cout<<"resection"<<endl;
    unsigned int n = point_computed_.size( ),   // number of points
      m = 0,   // number of images (or camera)
      mcon = 0,// number of images (starting from the 1st) whose parameters should not be modified.
      cnp = 6,// number of parameters for ONE camera; e.g. 6 for Euclidean cameras
      //use only vector part of quaternion to enforce the unit lenght...
      mnp = 2;// number of parameters for ONE projected point; e.g. 2 for Euclidean points

    unsigned int i = 0, j = 0,
      nb_cam = camera_computed_.size( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );
    vector< TrackOfPoints > real_track;
    //keep only tracks having image:

    for(i=0; i<n; i++)
      if( point_computed_[i].containImage(image) )
        real_track.push_back(point_computed_[i]);
    n = real_track.size();

    //now for fun show the sequence on images:
    //sequence_.showTracks( image, real_track );

    //because some points are sometime not visible:
    vector<int> idx_cameras;
    libmv::vector< libmv::Mat3 > intra_p;
    int nz_count = 0;
    for ( i = 0; i < nb_cam; ++i )
    {//for each camera:
      if( camera_computed_[ i ] && i!=image )
      {
        idx_cameras.push_back(i);
        m++;//increament of camera count

        int nb_projection = 0;
        for ( j = 0; j < n; ++j )
        {//for each 3D point:
          if( real_track[ j ].containImage( i ) )
            nb_projection++;
        }
        nz_count += nb_projection;
      }
    }

    mcon = m;//other cameras are constant!

    idx_cameras.push_back(image);
    m++;//increament of camera count

    int nb_projection = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      if( real_track[ j ].containImage( image ) )
        nb_projection++;
    }
    nz_count += nb_projection;

    //2D points:
    char *vmask = new char[ n*m ];//visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise.
    double *p = new double[m*cnp + n*3];//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    // aj are the image j parameters, bi are the i-th point parameters

    double *x = new double[ 2*nz_count ];// measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
    // x_ij is the projection of the i-th point on the j-th image.
    // NOTE: some of the x_ij might be missing, if point i is not visible in image j;
    // see vmask[i, j], max. size n*m*mnp

    libmv::vector< Eigen::Quaterniond > init_rotation;
    libmv::vector< libmv::Vec3 > init_translat;
    //update each variable:
    double *p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      intra_p.push_back( intra_params_[idx_cam] );
      //extrinsic parameters only (intra are know in euclidean reconstruction)
      init_rotation.push_back( (Eigen::Quaterniond)rotations_[ idx_cam ] );
      init_translat.push_back( translations_[ idx_cam ] );
      //add camera parameters to p:
      //as this is rotation, the quaternion's length is unity. Only 3 values are needed.
      //4th value equal:
      //sqrt(1.0 - quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2]));

      p_local[0] = 0; p_local[1] = 0; p_local[2] = 0;


      p_local[3] = 0; p_local[4] = 0; p_local[5] = 0;

      p_local+=cnp;
    }

    //now add the projections and 3D points:
    unsigned int idx_visible = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      for ( i=0; i < m; ++i )
      {//for each camera:
        int idx_cam = idx_cameras[i];
        vmask[ i+j*m ] = real_track[ j ].containImage( idx_cam );
        if( vmask[ i+j*m ] )
        {
          cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
            real_track[ j ].getPointIndex( idx_cam ) );
          x[ idx_visible++ ] = pt.pt.x;
          x[ idx_visible++ ] = pt.pt.y;
        }
      }
    }
    double* points3D_values = p_local;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      cv::Vec3d cv3DPoint = real_track[ j ];
      *(p_local++) = cv3DPoint[ 0 ];
      *(p_local++) = cv3DPoint[ 1 ];
      *(p_local++) = cv3DPoint[ 2 ];
    }

    //TUNING PARAMETERS:
    int itmax = 1000;        //max iterations
    int verbose = 0;         //no debug
    double opts[SBA_OPTSSZ] = {
      0.1,		//Tau
      1e-12,		//E1
      1e-12,		//E2
      0,		//E3 average reprojection error
      0		//E4 relative reduction in the RMS reprojection error
    };

    double info[SBA_INFOSZ];
    bundle_datas data(intra_p,init_rotation, init_translat,
      cnp, 3, mnp, 0, mcon);
    data.points3D = points3D_values;
    //use sba library
    int iter = sba_mot_levmar_x(n, m, mcon, vmask, p, cnp, x, NULL, mnp,
      img_projsRT_x, NULL, (void*)&data, itmax, 0, opts, info);

    bool resection_ok = true;
    if( ( iter<=0 ) || (info[1]/nz_count)>max_reprojection )
    {
      resection_ok = false;
      std::cout<<"resection rejected ("<<nz_count<<") : "<<info[1]/nz_count<<std::endl;
    }
    else
    {
      std::cout<<"SBA ("<<nz_count<<") returned in "<<iter<<" iter, reason "<<info[6]
      <<", error "<<info[1]/nz_count<<" [initial "<< info[0]/nz_count<<"]\n";


      //set new values:
      m = idx_cameras.size();
      n = real_track.size( );
      idx_visible = 0;
      p_local = p + cnp*(m-1);

      //extrinsic parameters only (intra are know in euclidean reconstruction)
      Eigen::Quaterniond rot_init = data.rotations[ m-1 ];
      double c1 = p_local[0];
      double c2 = p_local[1];
      double c3 = p_local[2];
      double coef=(1.0 - c1*c1 - c2*c2 - c3*c3 );
      if( coef>0 )
        coef = sqrt( coef );
      else//problem with this rotation...
      {
        coef = 0;
        Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
        quat_delta.normalize();
        c1=quat_delta.x(); c2=quat_delta.y(); c3=quat_delta.z();
        coef = quat_delta.w();
      }

      Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
      Eigen::Quaterniond rot_total = quat_delta * rot_init;
      //add camera parameters to p:
      rotations_[ image ] = rot_total.toRotationMatrix();

      translations_[ image ](0) += p_local[3];
      translations_[ image ](1) += p_local[4];
      translations_[ image ](2) += p_local[5];

      //update camera's structure:
      cv::Mat newRotation,newTranslation;
      cv::eigen2cv( rotations_[ image ], newRotation );
      cv::eigen2cv( translations_[ image ], newTranslation );
      cameras_[ image ].setRotationMatrix( newRotation );
      cameras_[ image ].setTranslationVector( newTranslation );

      camera_computed_[ image ] = true;
    }

    delete [] vmask;//visibility mask
    delete [] p;//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    delete [] x;// measurement vector


    return resection_ok;
  }

  void EuclideanEstimator::initialReconstruction( int image1, int image2 )
  {
    vector<TrackOfPoints>& tracks = sequence_.getTracks( );
    camera_computed_[ image1 ] = true;
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
    std::cout<<"max_error: "<<error<<std::endl;


    //From this essential matrix extract relative motion:
    libmv::Mat3 R;
    libmv::Vec3 t;
    libmv::Vec2 x1Col, x2Col;
    x1Col << x1( 0,0 ), x1( 1,0 );
    x2Col << x2( 0,0 ), x2( 1,0 );
    bool ok = libmv::MotionFromEssentialAndCorrespondence( E,
      intra_params_[ image1 ], x1Col,
      intra_params_[ image2 ], x2Col,
      &R, &t );

    rotations_[ image2 ] = R * rotations_[ image1 ];
    translations_[ image2 ] = t + R * translations_[ image1 ];

    //update camera's structure:
    cv::Mat newRotation,newTranslation;
    cv::eigen2cv( rotations_[ image2 ], newRotation );
    cv::eigen2cv( translations_[ image2 ], newTranslation );
    cameras_[ image2 ].setRotationMatrix( newRotation );
    cameras_[ image2 ].setTranslationVector( newTranslation );

    //this camera is now computed:
    camera_computed_[ image2 ] = true;

    //Triangulate the points:
    StructureEstimator se( &sequence_, &this->cameras_ );
    vector<int> images_to_compute;
    images_to_compute.push_back( image1 );
    images_to_compute.push_back( image2 );
    point_computed_ = se.computeStructure( images_to_compute );
    //bundleAdjustement();
  }

  void EuclideanEstimator::addMoreMatches(int img1, int img2,
    std::string detect, std::string extractor )
  {
    std::string methodMatch = "FlannBased";
    if( extractor.find("ORB")!=std::string::npos ||
      extractor.find("BRIEF")!=std::string::npos)
      methodMatch = "BruteForce-Hamming";
    Ptr<PointsMatcher> point_matcher = 
      new PointsMatcher( cv::DescriptorMatcher::create( methodMatch ) );

    Ptr<PointsToTrack> pointCollection = Ptr<PointsToTrack>(
      new PointsToTrackWithImage( img1, sequence_.getImage(img1), detect, extractor ));
    Ptr<PointsToTrack> pointCollection1 = Ptr<PointsToTrack>(
      new PointsToTrackWithImage( img2, sequence_.getImage(img2), detect, extractor ));
    pointCollection->computeKeypointsAndDesc( true );
    point_matcher->add( pointCollection );
    point_matcher->train();

    pointCollection1->computeKeypointsAndDesc( true );
    Ptr<PointsMatcher> point_matcher1 =
      new PointsMatcher( cv::DescriptorMatcher::create( methodMatch ) );
    point_matcher1->add( pointCollection1 );
    point_matcher1->train( );

    vector< cv::DMatch > matches_i_j = 
      SequenceAnalyzer::simple_matching(point_matcher, point_matcher1 );
    //matches don't use the same indices... set correct one:
    vector<cv::KeyPoint>& kpImg1 =
      sequence_.getPointsToTrack()[img1]->getModifiableKeypoints();
    vector<cv::KeyPoint>& kpImg2 =
      sequence_.getPointsToTrack()[img2]->getModifiableKeypoints();
    int point_added = 0;
    size_t nbK1 = kpImg1.size(), nbK2 = kpImg2.size(), cpt1;
    for(size_t cpt=0; cpt<matches_i_j.size(); ++cpt)
    {
      const cv::KeyPoint &key1 = point_matcher->getKeypoint(
        matches_i_j[ cpt ].trainIdx );
      const cv::KeyPoint &key2 = point_matcher1->getKeypoint(
        matches_i_j[ cpt ].queryIdx );
      //now look for a close keypoint:
      //first in img1:
      bool found = false;
      for(cpt1 = 0; cpt1<nbK1 && !found; ++cpt1)
        found = (abs(kpImg1[cpt1].pt.x - key1.pt.x) +
        abs(kpImg1[cpt1].pt.y - key1.pt.y) ) < 2;
      if(found)
        matches_i_j[ cpt ].trainIdx = --cpt1;
      else
      {
        matches_i_j[ cpt ].trainIdx = nbK1++;
        kpImg1.push_back( key1 );
        point_added++;
      }

      found = false;
      for(cpt1 = 0; cpt1<nbK2 && !found; ++cpt1)
        found = (abs(kpImg2[cpt1].pt.x - key2.pt.x) +
        abs(kpImg2[cpt1].pt.y - key2.pt.y) ) < 2;
      if(found)
        matches_i_j[ cpt ].queryIdx = --cpt1;
      else
      {
        matches_i_j[ cpt ].queryIdx = nbK2++;
        kpImg2.push_back( key2 );
        point_added++;
      }
    }
    sequence_.addMatches(matches_i_j,img1,img2);

    SequenceAnalyzer::keepOnlyCorrectMatches( sequence_, 2, 0 );

    //Triangulate the points:
    StructureEstimator se( &sequence_, &this->cameras_ );

    vector<int> images_to_compute;
    unsigned int key_size = camera_computed_.size( );
    for (unsigned int i=0; i < key_size; ++i )
    {
      if( camera_computed_[ i ] )
        images_to_compute.push_back( i );
    }

    point_computed_ = se.computeStructure( images_to_compute, 2 );
    //se.removeOutliersTracks( 5, &point_computed_ );
    //SequenceAnalyzer::keepOnlyCorrectMatches(point_computed_,2,0);
  }

  void EuclideanEstimator::computeReconstruction( )
  {
    vector<TrackOfPoints>& tracks = sequence_.getTracks( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );
    ImagesGraphConnection &images_graph = sequence_.getImgGraph( );
    //double ransac_threshold = 0.4 * sequence_.getImage( 0 ).rows / 100.0;
    double ransac_threshold = 3.0;
    //now create the graph:

    int img1,img2;
    int nbMatches = images_graph.getHighestLink( img1,img2 );
    vector<ImageLink> bestMatches;
    images_graph.getOrderedLinks( bestMatches, nbMatches*0.8 );
    double min_inliners=1e7;
    int index_of_min=0;/*
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
          status, cv::FM_RANSAC );
      }
    }
    //we will start the reconstruction using bestMatches[ index_of_min ]
    //to avoid degenerate cases such as coincident cameras*/

    vector<int> images_computed;
    int iter = 0;
    bool not_correct;
    int nbMax = 0, bestId = 0;/*
    do
    {
      img1 = bestMatches[ index_of_min ].imgSrc;
      img2 = bestMatches[ index_of_min ].imgDest;
      cout<<img1<<", "<<img2<<endl;
      //sequence_.showTracksBetween(img1, img2);

      images_computed.clear();
      images_computed.push_back( img1 );
      images_computed.push_back( img2 );
      camera_computed_[ img1 ] = true;
      initialReconstruction( img1, img2 );
    
      //try to find more matches:
      std::vector< TrackOfPoints > point_before = point_computed_;
      cout<<point_computed_.size()<<endl;

      not_correct = point_computed_.size() < nbMatches/3 ||
        point_computed_.size() < point_before.size()-20;
      if( point_computed_.size()>nbMax )
      {
        nbMax = point_computed_.size();
        bestId = index_of_min;
      }
      point_computed_.clear();
      camera_computed_[ img1 ] = false;
      camera_computed_[ img2 ] = false;
      index_of_min++;
    } while ( index_of_min<bestMatches.size() );*/
    bestId = bestMatches.size() - 1;
    img1 = bestMatches[ bestId ].imgSrc;
    img2 = bestMatches[ bestId ].imgDest;
    cout<<img1<<", "<<img2<<endl;
    //sequence_.showTracksBetween(img1, img2);

    images_computed.clear();
    images_computed.push_back( img1 );
    images_computed.push_back( img2 );
    camera_computed_[ img1 ] = true;
    initialReconstruction( img1, img2 );

    //try to find more matches:
    std::vector< TrackOfPoints > point_before = point_computed_;
    cout<<"before"<<point_computed_.size()<<endl;
    addMoreMatches( img1, img2 );
    cout<<"after"<<point_computed_.size()<<endl;
    if( point_before.size() > point_computed_.size() )
      point_computed_ = point_before;


    //bundleAdjustement();

    //now we have updated the position of the camera which take img2
    //and 3D estimation from these 2 first cameras...
    //Find for other cameras position:
    vector<ImageLink> images_close;
    int nbIter = 0;
    while( nbMatches>10 && images_computed.size()<cameras_.size() && nbIter<20 )
    {
      nbIter++;
      images_close.clear( );
      while ( images_close.size( ) < 2 && nbMatches>0 )
      {
        for(size_t cpt = 0; cpt<images_computed.size(); ++cpt )
        {
          images_graph.getImagesRelatedTo( images_computed[ cpt ],
            images_close, nbMatches * 0.8 - 10 );
          //remove from images_close those into camera_computed_:
          for(size_t cpt1 = 0; cpt1<images_close.size(); ++cpt1 )
          {
            if( camera_computed_[ images_close[cpt1].imgDest ] &&
              camera_computed_[ images_close[cpt1].imgSrc ] )
            {
              images_close[ cpt1 ] = images_close[ images_close.size() - 1 ];
              images_close.pop_back();
              cpt1--;
            }
          }
        }
        nbMatches = nbMatches * 0.9;
        if(nbMatches<0)
          nbMatches=0;
      }

      //for each images, comptute the camera position:
      for( size_t cpt=0;cpt<images_close.size( );cpt++ )
      {
        //We don't want to compute twice the same camera position:
        int new_id_image = -1,
          old_id_image = -1;
        if( !camera_computed_[ images_close[ cpt ].imgSrc ] )
        {
          new_id_image = images_close[ cpt ].imgSrc;
          old_id_image = images_close[ cpt ].imgDest;
        }
        else
        {
          new_id_image = images_close[ cpt ].imgDest;
          old_id_image = images_close[ cpt ].imgSrc;
        }

        if( new_id_image >= 0 )
        {
          if( cameraResection( new_id_image, 50*(nbIter/4.0+1.0) ) )
          {
            images_computed.push_back( new_id_image );
            std::vector< TrackOfPoints > point_before = point_computed_;
            cout<<"before"<<point_computed_.size()<<endl;
            addMoreMatches( old_id_image, new_id_image );
            cout<<"after"<<point_computed_.size()<<endl;
            if( point_computed_.size() < point_before.size() )
              point_computed_ = point_before;
          }
        }
      }
      // Performs a bundle adjustment
      bundleAdjustement();
    }//*/
    /*
    for( unsigned int i = 0; i<cameras_.size( ) ; ++i )
      camera_computed_[i] = true;

    point_computed_ = sequence_.getTracks();

    bundleAdjustement();*/

  }

  void EuclideanEstimator::viewEstimation( bool coloredPoints )
  {
    vector<cv::Vec3d> tracks3D;
    vector< unsigned int > colors;
    vector<TrackOfPoints>::iterator itTrack=point_computed_.begin( );
    while ( itTrack != point_computed_.end( ) )
    {
      tracks3D.push_back( ( cv::Vec3d )( *itTrack ) );
      colors.push_back( itTrack->getColor() );
      itTrack++;
    }

    //////////////////////////////////////////////////////////////////////////
    // Open 3D viewer and add point cloud
    Visualizer debugView ( "Debug viewer" );
    if( coloredPoints )
      debugView.add3DPointsColored( tracks3D,colors, "Euclidean estimated" );
    else
      debugView.add3DPoints( tracks3D, "Euclidean estimated" );
    
    for( unsigned int i = 0; i<cameras_.size( ) ; ++i )
      if( camera_computed_[i] )
      {
        std::stringstream cam_name;
        cam_name<<"Cam"<< ( i+1 );
        debugView.addCamera( cameras_[ i ],
          cam_name.str() );
        cout<<cameras_[ i ].getTranslationVector()<<endl;
      }
      


      debugView.runInteract( );
  }
}
