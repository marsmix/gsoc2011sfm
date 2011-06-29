#include "ProjectiveEstimator.h"

using std::vector;
using cv::Ptr;

namespace OpencvSfM{
  //only for intern usage, no external interface...
  //Idea from Snavely : Modeling the World from Internet Photo Collections
  //See with libmv team if such a function is usefull:
  void robust5Points(const libmv::Mat2X &x1, const libmv::Mat2X &x2,
    const libmv::Mat3 &K1, const libmv::Mat3 &K2,
    libmv::Mat3 &E)
  {
    unsigned int nPoints = x1.rows();
    CV_Assert( nPoints == x2.rows() );

    libmv::vector<libmv::Mat3> Es;
    cv::RNG& rng = cv::theRNG();
    vector<int> masks;
    double max_error = 1e9;

    int num_iter=0, max_iter=10;
    for(num_iter=0; num_iter<max_iter; ++num_iter)
    {
      masks.clear();
      int nb_vals=0;
      for (unsigned int cpt = 0; cpt < nPoints; cpt++) {
        int valTmp = rng(2);
        if( valTmp>0 )
          nb_vals++;
        masks.push_back(valTmp);
      }
      while( nb_vals<5 )
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

        double error = libmv::SampsonDistance2( F, x1, x2);

        if (max_error > error ) {
            error = max_error;
            E = Es[i];
        }
      }
    }
  }


  ProjectiveEstimator::ProjectiveEstimator(SequenceAnalyzer &sequence,
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


  ProjectiveEstimator::~ProjectiveEstimator(void)
  {
    //TODO!!!!
  }

  void ProjectiveEstimator::updateTwoViewMotion(vector<TrackPoints>& tracks,
    vector<Ptr<PointsToTrack>> &points_to_track,
    int image1, int image2)
  {
    libmv::Mat3 E;
    Ptr<PointsToTrack> point_img1 = points_to_track[image1];
    Ptr<PointsToTrack> point_img2 = points_to_track[image2];
    //first extract points matches:
    libmv::Mat2X x1,x2;
    //for each points:
    unsigned int key_size = tracks.size();
    unsigned int i;
    vector<TrackPoints> matches;
    
    for (i=0; i < key_size; ++i)
    {
      TrackPoints &track = tracks[i];
      if( track.containImage(image1) && track.containImage(image2) )
        matches.push_back(track);
    }
    x1.resize(2,matches.size());
    x2.resize(2,matches.size());

    key_size = matches.size();
    for (i=0; i < key_size; ++i)
    {
      TrackPoints &track = matches[i];
      cv::DMatch match = track.toDMatch(image1, image2);

      x1(0,i) = point_img1->getKeypoint(match.trainIdx).pt.x;
      x1(1,i) = point_img1->getKeypoint(match.trainIdx).pt.y;
      x2(0,i) = point_img2->getKeypoint(match.queryIdx).pt.x;
      x2(1,i) = point_img2->getKeypoint(match.queryIdx).pt.y;
    }
    robust5Points( x1, x2, intra_params_[image1], intra_params_[image2], E );
    //From this essential matrix extract relative motion:
    libmv::Mat3 R;
    libmv::Vec3 t;
    bool ok = libmv::MotionFromEssentialAndCorrespondence( E,
      intra_params_[image1], x1,
      intra_params_[image2], x2,
      &R, &t);

    //As R and t are relative to first cam, set them to the real position:
    rotations_[image2] = R * rotations_[image1].transpose().inverse();
    translations_[image2] = t + R * translations_[image1];
  }

  void ProjectiveEstimator::comptueReconstruction()
  {
    vector<TrackPoints>& tracks = sequence_.getTracks();
    vector<Ptr<PointsToTrack>> &points_to_track = sequence_.getPoints();
    ImagesGraphConnection &images_graph = sequence_.getImgGraph();
    //now create the graph:
    
    int img1,img2;
    int nbMatches = images_graph.getHighestLink(img1,img2);
    vector<ImageLink> bestMatches;
    images_graph.getOrderedLinks(bestMatches, static_cast<int>(.8*nbMatches),
      nbMatches);
    int max_inliners=0,index_of_max=0;
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
      cv::findHomography(pointsImg1,pointsImg2,status,CV_RANSAC,50);
      //count the inliner points:
      int inliners=0;
      for(unsigned int i=0;i<status.size();++i)
      {
        if( status[i] != 0 )
          inliners++;
      }
      if(inliners>max_inliners)
      {
        max_inliners = inliners;
        index_of_max = cpt;
      }
    }

    //we will start the reconstruction using bestMatches[index_of_max]
    img1 = bestMatches[index_of_max].imgSrc;
    img2 = bestMatches[index_of_max].imgDest;
    updateTwoViewMotion(tracks, points_to_track, img1, img2);
  }
}