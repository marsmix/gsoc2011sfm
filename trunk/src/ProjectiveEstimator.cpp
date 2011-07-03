#include "ProjectiveEstimator.h"
#include "StructureEstimator.h"

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
    unsigned int nPoints = x1.cols();
    CV_Assert( nPoints == x2.cols() );
    CV_Assert( nPoints >= 5 );//need 5 points!

    libmv::vector<libmv::Mat3> Es;
    cv::RNG& rng = cv::theRNG();
    vector<int> masks(nPoints);
    double max_error = 1e9;

    int num_iter=0, max_iter=10;
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

        double error = libmv::SampsonDistance2( F, x1, x2);

        if (max_error > error ) {
            max_error = error;
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
      
      /*
      x1(0,i) = point_img1->getKeypoint(match.trainIdx).pt.x;
      x1(1,i) = point_img1->getKeypoint(match.trainIdx).pt.y;
      x2(0,i) = point_img2->getKeypoint(match.queryIdx).pt.x;
      x2(1,i) = point_img2->getKeypoint(match.queryIdx).pt.y;
      */

      vector<cv::Vec2d> pointImg1;
      pointImg1.push_back( cv::Vec2d(point_img1->getKeypoint(match.trainIdx).pt.x,
        point_img1->getKeypoint(match.trainIdx).pt.y) );
      vector<cv::Vec2d> pointImg2;
      pointImg2.push_back( cv::Vec2d( point_img2->getKeypoint(match.queryIdx).pt.x,
        point_img2->getKeypoint(match.queryIdx).pt.y) );
      vector<cv::Vec2d> pointNorm1 = cameras_[image1].getIntraParameters()->
        pixelToNormImageCoordinates(pointImg1);
      vector<cv::Vec2d> pointNorm2 = cameras_[image2].getIntraParameters()->
        pixelToNormImageCoordinates(pointImg2);
      x1(0,i) = pointNorm1[0][0];
      x1(1,i) = pointNorm1[0][1];
      x2(0,i) = pointNorm2[0][0];
      x2(1,i) = pointNorm2[0][1];
    }
    robust5Points( x1, x2, intra_params_[image1], intra_params_[image2], E );

    //From this essential matrix extract relative motion:
    libmv::Mat3 R;
    libmv::Vec3 t;
    bool ok = libmv::MotionFromEssentialAndCorrespondence( E,
      intra_params_[image1], x1.col(1),
      intra_params_[image2], x2.col(1),
      &R, &t);

    //As R and t are relative to first cam, set them to the real position:
    rotations_[image2] = R * rotations_[image1].transpose().inverse();
    translations_[image2] = t + R * translations_[image1];

    //update camera's structure:
    cv::Mat newRotation,newTranslation;
    libmv::convertEigenToCvMat( rotations_[image2], CV_64F, newRotation );
    libmv::convertEigenToCvMat( translations_[image2], CV_64F, newTranslation );
    cameras_[image2].setRotationMatrix( newRotation );
    cameras_[image2].setTranslationVector( newTranslation );
  }

  //////////////////////////////////////////////////////////////////////////
  template<typename TMat>
  inline double FrobeniusNorm(const TMat &A) {
    return sqrt(A.array().abs2().sum());
  }
  template<typename TMat>
  inline double FrobeniusDistance(const TMat &A, const TMat &B) {
    return FrobeniusNorm(A - B);
  }
  template<typename TVec>
  inline double DistanceL2(const TVec &x, const TVec &y) {
    return (x - y).norm();
  }

  // Normalize a vector with the L2 norm, and return the norm before it was
  // normalized.
  template<typename TVec>
  inline double NormalizeL2(TVec *x) {
    double norm = x->norm();
    *x /= norm;
    return norm;
  }
  //////////////////////////////////////////////////////////////////////////

  void ProjectiveEstimator::comptueReconstruction(vector<PointOfView>& camReal)
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

    
    //////////////////////////////////////////////////////////////////////////
    libmv::Mat3 rotation_mat,rot_real;
    libmv::convertCvMatToEigen(camReal[img1].getRotationMatrix(),rotation_mat);
    libmv::convertCvMatToEigen(camReal[img2].getRotationMatrix(),rot_real);
    libmv::Vec3 translation_vec,trans_real;
    libmv::convertCvMatToEigen(camReal[img1].getTranslationVector(),translation_vec);
    libmv::convertCvMatToEigen(camReal[img2].getTranslationVector(),trans_real);

    translation_vec = translations_[img2] + rotations_[img1] * translation_vec;
    NormalizeL2(&translation_vec);
    NormalizeL2(&trans_real);

    std::cout<<"translation computed:"<<std::endl<<translation_vec<<std::endl;
    std::cout<<"translation real:"<<std::endl<<trans_real<<std::endl<<std::endl;
    rotation_mat = rotations_[img1] * rotation_mat.transpose().inverse();
    std::cout<<"rotation computed:"<<std::endl<<rotation_mat<<std::endl;
    std::cout<<"rotation real:"<<std::endl<<rot_real<<std::endl<<std::endl;

    double dist = FrobeniusDistance(rotation_mat, rot_real);
    double dist1 = DistanceL2(translation_vec, trans_real);
    //////////////////////////////////////////////////////////////////////////
    

    camReal[img1] = cameras_[img1];
    camReal[img2] = cameras_[img2];

    StructureEstimator se(sequence_, cameras_);
    vector<TrackPoints> points3DTrack;
    se.computeTwoView(img1, img2, points3DTrack);
    vector<cv::Vec3d> points3D;
    for(unsigned int cpt=0;cpt<points3DTrack.size();++cpt)
      points3D.push_back(points3DTrack[cpt]);

    //now for each point of view, we draw the picture and these points projected:
    vector<PointOfView>::iterator itPoV = camReal.begin();
    int index_image=0;
    while ( itPoV!=camReal.end() )
    {
      cv::Mat imgTmp=sequence_.getImage(index_image);//get the current image
      if(imgTmp.empty())
        break;//end of sequence: quit!
      index_image++;

      //create the vector of 3D points viewed by this camera:
      vector<cv::KeyPoint> points2DOrigine;
      vector<cv::Vec2d> pixelProjected=itPoV->project3DPointsIntoImage(points3D);
      //convert Vec2d into KeyPoint:
      vector<cv::KeyPoint> points2D;
      for(unsigned int j=0;j<pixelProjected.size();j++)
        points2D.push_back( cv::KeyPoint( (float)pixelProjected[j][0],
        (float)pixelProjected[j][1], 10.0 ) );

      cv::Mat imgTmp1,imgTmp2;
      cv::drawKeypoints(imgTmp,points2DOrigine,imgTmp1,cv::Scalar(255,255,255));
      cv::drawKeypoints(imgTmp,points2D,imgTmp2,cv::Scalar(255,255,255));
      cv::imshow("Points origine...",imgTmp1);
      cv::imshow("Points projected...",imgTmp2);
      cv::waitKey(0);
      itPoV++;
    }
  }
}