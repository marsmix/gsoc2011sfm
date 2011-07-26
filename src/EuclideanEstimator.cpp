#include "EuclideanEstimator.h"
#include "StructureEstimator.h"
#include <opencv2/core/eigen.hpp>

#include "Visualizer.h"
#include "PCL_mapping.h"
#include <pcl/point_types.h>
#include "libmv/multiview/five_point.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/robust_euclidean_resection.h"

#include <pcl/io/vtk_io.h>

using std::vector;
using cv::Ptr;

namespace OpencvSfM{
  //the next two functions are only for intern usage, no external interface...
  
  //Idea from Snavely : Modeling the World from Internet Photo Collections
  //See with libmv team if such a function is usefull:
  double robust5Points(const libmv::Mat2X &x1, const libmv::Mat2X &x2,
    const libmv::Mat3 &K1, const libmv::Mat3 &K2,
    libmv::Mat3 &E)
  {
    unsigned int nPoints = x1.cols();
    CV_DbgAssert( nPoints == x2.cols() );
    CV_DbgAssert( nPoints >= 5 );//need 5 points!

    libmv::vector<libmv::Mat3> Es;
    cv::RNG& rng = cv::theRNG();
    vector<int> masks(nPoints);
    double max_error = 1e9;

    int num_iter=0, max_iter=nPoints-5;
    for(num_iter=0; num_iter<max_iter; ++num_iter)
    {
      masks.clear();
      int nb_vals=0;
      for (unsigned int cpt = 0; cpt < nPoints; cpt++) {
        masks.push_back(0);
      }
      while( nb_vals < 5 )
      {
        int valTmp = rng(nPoints);
        if( masks[valTmp] == 0 )
        {
          masks[valTmp] = 1;
          nb_vals++;
        }
      }
      //create mask:
      libmv::Mat2X x1_tmp,x2_tmp;
      x1_tmp.resize(2,nb_vals);
      x2_tmp.resize(2,nb_vals);
      nb_vals=0;
      unsigned int i;
      for( i = 0; i<nPoints; ++i)
      {
        if( masks[i] != 0 )
        {
          x1_tmp(0,nb_vals) = x1( 0,i );
          x1_tmp(1,nb_vals) = x1( 1,i );
          x2_tmp(0,nb_vals) = x2( 0,i );
          x2_tmp(1,nb_vals) = x2( 1,i );
          nb_vals++;
        }
      }
      libmv::FivePointsRelativePose(x1_tmp,x2_tmp,&Es);
      unsigned int num_hyp = Es.size();
      for (i = 0; i < num_hyp; i++) {
        
        libmv::Mat3 F;
        libmv::FundamentalFromEssential( Es[i], K1, K2, &F );

        double error = libmv::SampsonDistance( F, x1, x2);

        if (max_error > error ) {
            max_error = error;
            E = Es[i];
        }
      }
    }
    return max_error;
  }


  EuclideanEstimator::EuclideanEstimator(SequenceAnalyzer &sequence,
    vector<PointOfView>& cameras)
    :sequence_(sequence),cameras_(cameras)
  {
    vector<PointOfView>::iterator itPoV=cameras.begin();
    while ( itPoV!=cameras.end() )
    {
      addNewPointOfView(*itPoV);
      itPoV++;
    }
  }


  EuclideanEstimator::~EuclideanEstimator(void)
  {
    //TODO!!!!
  }

  void EuclideanEstimator::cameraResection( unsigned int image,
    const Ptr< PointsToTrack > points_to_track )
  {
    //the 3D points to use are stored in point_computed_...
    //extract 3D points and projections:
    vector<cv::Vec2d> x_im;
    vector<cv::Vec3d> X_w;
    libmv::Mat2X x_image;
    libmv::Mat3X X_world;

    // Selects only the reconstructed tracks observed in the image
    //for each points:
    unsigned int key_size = point_computed_.size(),
      i = 0;
    vector<TrackOfPoints> matches;

    for (i=0; i < key_size; ++i)
    {
      TrackOfPoints &track = point_computed_[i];
      if( track.containImage( image ) )
      {
        int idx = track.getIndexPoint( image );
        cv::KeyPoint p = points_to_track->getKeypoint( idx );
        x_im.push_back( cv::Vec2d( p.pt.x, p.pt.y ) );
        X_w.push_back( (cv::Vec3d)track );
      }
    }

    //convert point in normalized camera coordinates
    //and convert to libmv structure:
    key_size = x_im.size();
    cv::Ptr<Camera> device = cameras_[image].getIntraParameters();
    vector<cv::Vec2d> x_im_norm = device->pixelToNormImageCoordinates( x_im );
    x_image.resize(key_size, 2);
    X_world.resize(key_size, 3);
    for (i=0; i < key_size; ++i)
    {
      libmv::Vec2 tmpVec( x_im[i][0], x_im[i][1] );
      libmv::Vec3 tmpVec3D( X_w[i][0], X_w[i][1], X_w[i][2] );

      x_image.col(i) = tmpVec;
      X_world.col(i) = tmpVec3D;
    }

    libmv::Mat3 R;
    libmv::Vec3 t;
    double rms_inliers_threshold = 1;// in pixels
    libmv::vector<int> inliers;
    libmv::EuclideanResectionEPnPRobust(x_image, X_world,
      intra_params_[image],
      rms_inliers_threshold, &R, &t, &inliers, 1e-3);
  }

  void EuclideanEstimator::initialReconstruction(vector<TrackOfPoints>& tracks,
    const vector< Ptr< PointsToTrack > > &points_to_track,
    int image1, int image2)
  {
    CV_Assert( camera_computed_[image1] );

    libmv::Mat3 E;
    Ptr<PointsToTrack> point_img1 = points_to_track[image1];
    Ptr<PointsToTrack> point_img2 = points_to_track[image2];
    //first extract points matches:
    libmv::Mat2X x1,x2;
    //for each points:
    unsigned int key_size = tracks.size();
    unsigned int i;
    vector<TrackOfPoints> matches;
    
    for (i=0; i < key_size; ++i)
    {
      TrackOfPoints &track = tracks[i];
      if( track.containImage(image1) && track.containImage(image2) )
        matches.push_back(track);
    }
    x1.resize(2,matches.size());
    x2.resize(2,matches.size());

    key_size = matches.size();
    vector<cv::Vec2d> pointImg1,pointImg2;
    for (i=0; i < key_size; ++i)
    {
      TrackOfPoints &track = matches[i];
      cv::DMatch match = track.toDMatch(image1, image2);
      
      pointImg1.push_back( cv::Vec2d(point_img1->getKeypoint(match.trainIdx).pt.x,
        point_img1->getKeypoint(match.trainIdx).pt.y) );
      pointImg2.push_back( cv::Vec2d( point_img2->getKeypoint(match.queryIdx).pt.x,
        point_img2->getKeypoint(match.queryIdx).pt.y) );
    }
    vector<cv::Vec2d> pointNorm1 = cameras_[image1].getIntraParameters()->
      pixelToNormImageCoordinates(pointImg1);
    vector<cv::Vec2d> pointNorm2 = cameras_[image2].getIntraParameters()->
      pixelToNormImageCoordinates(pointImg2);
    key_size = pointNorm1.size();
    for (i=0; i < key_size; ++i)
    {
      x1(0,i) = -pointNorm1[i][0];
      x1(1,i) = -pointNorm1[i][1];
      x2(0,i) = -pointNorm2[i][0];
      x2(1,i) = -pointNorm2[i][1];
    }
    
    double error = robust5Points( x1, x2,
      intra_params_[image1], intra_params_[image2], E );

    //std::cout<<"E: "<<E<<std::endl;
    //std::cout<<"max_error: "<<error<<std::endl;
    

    //From this essential matrix extract relative motion:
    libmv::Mat3 R;
    libmv::Vec3 t;
    libmv::Vec2 x1Col, x2Col;
    x1Col << x1(0,15), x1(1,15);
    x2Col << x2(0,15), x2(1,15);
    bool ok = libmv::MotionFromEssentialAndCorrespondence( E,
      intra_params_[image1], x1Col,
      intra_params_[image2], x2Col,
      &R, &t);

    rotations_[image2] = rotations_[image1] * R;
    translations_[image2] = rotations_[image1] * t + translations_[image1];

    //update camera's structure:
    cv::Mat newRotation,newTranslation;
    cv::eigen2cv( rotations_[image2], newRotation );
    cv::eigen2cv( translations_[image2], newTranslation );
    cameras_[image2].setRotationMatrix( newRotation );
    cameras_[image2].setTranslationVector( newTranslation );

    //this camera is now computed:
    camera_computed_[image2] = true;

    //Triangulate the points:
    StructureEstimator se(sequence_, this->cameras_);
    vector<int> images_to_compute;
    images_to_compute.push_back(image1);
    images_to_compute.push_back(image2);
    point_computed_ = se.computeStructure( images_to_compute );
  }


  void EuclideanEstimator::computeReconstruction()
  {
    vector<TrackOfPoints>& tracks = sequence_.getTracks();
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints();
    ImagesGraphConnection &images_graph = sequence_.getImgGraph();
    double ransac_threshold = 0.4 * sequence_.getImage(0).rows / 100.0;
    //now create the graph:
    
    int img1,img2;
    int nbMatches = images_graph.getHighestLink(img1,img2);
    vector<ImageLink> bestMatches;
    images_graph.getOrderedLinks(bestMatches, 100, nbMatches);
    double min_inliners=1e7;
    int index_of_min=0;
    cv::Mat minFundamental;
    for(unsigned int cpt=0;cpt<bestMatches.size();cpt++)
    {
      //construct the homography and choose the worse matches:
      //(see Snavely "Modeling the World from Internet Photo Collections")
      std::vector<cv::Point2f> pointsImg1, pointsImg2;
      vector<uchar> status;
      points_to_track[bestMatches[cpt].imgSrc]->getKeyMatches(tracks,
        bestMatches[cpt].imgDest, pointsImg1);
      points_to_track[bestMatches[cpt].imgDest]->getKeyMatches(tracks,
        bestMatches[cpt].imgSrc, pointsImg2);

      //compute the homography:
      cv::findHomography(pointsImg1,pointsImg2,status,CV_RANSAC,
        ransac_threshold );
      //count the inliner points:
      double inliners=0;
      for(unsigned int i=0;i<status.size();++i)
      {
        if( status[i] != 0 )
          inliners++;
      }
      double percent_inliner = inliners/static_cast<double>(pointsImg1.size());
      if( percent_inliner < min_inliners )
      {
        min_inliners = percent_inliner;
        index_of_min = cpt;
        minFundamental = cv::findFundamentalMat(pointsImg1, pointsImg2,
          status, cv::FM_LMEDS);
      }
    }
    //we will start the reconstruction using bestMatches[index_of_min]
    //to avoid degenerate cases such as coincident cameras
    img1 = bestMatches[index_of_min].imgSrc;
    img2 = bestMatches[index_of_min].imgDest;

    vector<int> images_computed;
    images_computed.push_back(img1);
    images_computed.push_back(img2);
    initialReconstruction(tracks, points_to_track, img1, img2);



    //now we have updated the position of the camera which take img2
    //and 3D estimation from these 2 first cameras...
    //Find for other cameras position:
    /*
    vector<ImageLink> images_close;

    while( nbMatches>10 )
    {

      images_close.clear();
      while ( images_close.size() == 0 )
      {
        images_graph.getImagesRelatedTo(img1,images_close,
          nbMatches * 0.8 - 10);
        images_graph.getImagesRelatedTo(img2,images_close,
          nbMatches * 0.8 - 10);
        nbMatches = nbMatches * 0.8 - 10;
      }

      //for each images links from images_close, comptute the camera position:
      for(unsigned int cpt=0;cpt<images_close.size();cpt++)
      {
        //We don't want to compute twice the same camera position:
        int new_id_image = -1;
        if( find( images_computed.begin(), images_computed.end(),
          images_close[cpt].imgSrc ) != images_computed.end() )
          new_id_image = images_close[cpt].imgSrc;
        if( find( images_computed.begin(), images_computed.end(),
          images_close[cpt].imgDest ) != images_computed.end() )
        {
          if ( new_id_image >= 0 )
          {
            //the two images are new!
            //skeep them for now...
            //TODO : make this work ;)
            new_id_image = -1;
          }
          else
            new_id_image = images_close[cpt].imgDest;
        }

        if( new_id_image >= 0 )
        {
          images_computed.push_back( new_id_image );
          if( new_id_image == images_close[cpt].imgSrc )
            initialReconstruction(tracks, points_to_track,
            images_close[cpt].imgDest, new_id_image);
          else
            initialReconstruction(tracks, points_to_track,
            images_close[cpt].imgSrc, new_id_image);
        }
      }
      }*/
    vector<cv::Vec3d> tracks3D;
    vector<TrackOfPoints>::iterator itTrack=point_computed_.begin();
    while ( itTrack != point_computed_.end() )
    {
      tracks3D.push_back( (cv::Vec3d)(*itTrack) );
      itTrack++;
    }

    //////////////////////////////////////////////////////////////////////////
    // Open 3D viewer and add point cloud
    
    Visualizer debugView ( "Debug viewer" );
    debugView.add3DPoints(tracks3D);
    debugView.addCamera( cameras_[img2], "Cam1" );
    debugView.addCamera( cameras_[img1], "Cam2" );


    debugView.runInteract();
  }
}
