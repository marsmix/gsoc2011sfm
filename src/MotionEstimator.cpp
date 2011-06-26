#include "MotionEstimator.h"

#include <iostream>
#include <sstream>
using cv::Ptr;
using cv::Mat;
using cv::DMatch;
using cv::KeyPoint;
using std::vector;
using cv::Point3d;

namespace OpencvSfM{


  bool TrackPoints::addMatch(const int image_src, const int point_idx1)
  {
    if( track_consistance<0 )
      return false;

    //If a track contains more than one keypoint in the same image but
    //a different keypoint, it is deemed inconsistent.
    vector<unsigned int>::iterator indexImg = images_indexes_.begin();
    vector<unsigned int>::iterator end_iter = images_indexes_.end();
    unsigned int index=0;
    while(indexImg != end_iter)
    {
      if ( *indexImg == image_src )
      {
        if( point_indexes_[index] == point_idx1 )
        {
          if( track_consistance>=0 )
            track_consistance++;
        }
        else
          track_consistance=-1;

        return track_consistance>=0;
      }
      index++;
      indexImg++;
    }

    images_indexes_.push_back(image_src);
    point_indexes_.push_back(point_idx1);
    return track_consistance>=0;
  }

  bool TrackPoints::containImage(const int image_wanted)
  {
    if(std::find(images_indexes_.begin(),images_indexes_.end(),image_wanted) ==
      images_indexes_.end())
      return false;
    return true;
  }

  bool TrackPoints::containPoint(const int image_src, const int point_idx1)
  {
    //we don't use find here because we want the number instead of iterator...
    vector<unsigned int>::iterator indexImg = images_indexes_.begin();
    vector<unsigned int>::iterator end_iter = images_indexes_.end();
    unsigned int index=0;
    while(indexImg != end_iter)
    {
      if ( *indexImg == image_src )
      {
        if( point_indexes_[index] == point_idx1 )
          return true;
      }
      index++;
      indexImg++;
    }
    return false;
  }

  DMatch TrackPoints::toDMatch(const int img1,const int img2) const
  {
    DMatch outMatch;
    char nbFound=0;
    //we don't use find here because we want the number instead of iterator...
    vector<unsigned int>::const_iterator indexImg = images_indexes_.begin();
    vector<unsigned int>::const_iterator end_iter = images_indexes_.end();
    unsigned int index=0;
    while(indexImg != end_iter)
    {
      if ( *indexImg == img1 )
      {
        nbFound++;
        outMatch.queryIdx = point_indexes_[index];
        if(nbFound==2)
          return outMatch;
      }
      if ( *indexImg == img2 )
      {
        nbFound++;
        outMatch.trainIdx = point_indexes_[index];
        if(nbFound==2)
          return outMatch;
      }
      index++;
      indexImg++;
    }
    return outMatch;
  };

  void TrackPoints::getMatch(const unsigned int index,
    int &idImage, int &idPoint) const
  {
    char nbFound=0;
    if( index < images_indexes_.size() )
    {
      idImage = images_indexes_[index];
      idPoint = point_indexes_[index];
    }
  };

  int TrackPoints::getIndexPoint(const unsigned int image) const
  {
    vector<unsigned int>::const_iterator result =
      std::find(images_indexes_.begin(),images_indexes_.end(),image);
    if( result == images_indexes_.end())
      return -1;
    return point_indexes_[ result - images_indexes_.begin() ];
  };
  
  double TrackPoints::errorEstimate(std::vector<PointOfView>& cameras,
    const std::vector<cv::Ptr<PointsToTrack>> &points_to_track,
    cv::Vec3d& points3D) const
  {
    double distance=0.0;
    unsigned int nviews = images_indexes_.size();
    for (unsigned int cpt = 0; cpt < nviews; cpt++) {
      int num_camera=images_indexes_[cpt];
      int num_point=point_indexes_[cpt];
      cv::Ptr<PointsToTrack> points2D = points_to_track[ num_camera ];

      const KeyPoint& p=points2D->getKeypoint(num_point);
      cv::Vec2d projP = cameras[num_camera].project3DPointIntoImage(points3D);

      //compute back-projection
      distance += ((p.pt.x-projP[0])*(p.pt.x-projP[0]) + 
        (p.pt.y-projP[1])*(p.pt.y-projP[1]) );
    }
    return distance/static_cast<double>(nviews);
  }
  double TrackPoints::triangulateLinear(vector<PointOfView>& cameras,
    const std::vector<cv::Ptr<PointsToTrack>> &points_to_track,
    cv::Vec3d& points3D, const vector<int> &masks)
  {
    unsigned int nviews = 0;
    bool hasMask=false;
    unsigned int i;
    if( masks.size()==images_indexes_.size() )
    {
      for (i=0; i<images_indexes_.size(); ++i)
      {
        if(masks[i]!=0)
          nviews++;
      }
      hasMask=true;
    }else
    {
      nviews = images_indexes_.size();
    }

    Mat design = Mat::zeros(3*nviews, 4 + nviews,CV_64FC1);
    unsigned  int real_position=0;
    i = 0;
    for (real_position = 0; real_position < images_indexes_.size();
      ++real_position ) {
      if( !hasMask || ( hasMask && masks[real_position]!=0))
      {
        int num_camera=images_indexes_[ real_position ];
        int num_point=point_indexes_[ real_position ];
        cv::Ptr<PointsToTrack> points2D = points_to_track[ num_camera ];
        const KeyPoint& p=points2D->getKeypoint(num_point);

        design(cv::Range(3*i,3*i+3), cv::Range(0,4)) = 
          -cameras[num_camera].getProjectionMatrix();
        design.at<double>(3*i + 0, 4 + i) = p.pt.x;
        design.at<double>(3*i + 1, 4 + i) = p.pt.y;
        design.at<double>(3*i + 2, 4 + i) = 1.0;
        i++;
      }
    }
    Mat X_and_alphas;
    cv::SVD::solveZ(design, X_and_alphas);

    double scal_factor=X_and_alphas.at<double>(3,0);
    points3D[0]=X_and_alphas.at<double>(0,0)/scal_factor;
    points3D[1]=X_and_alphas.at<double>(1,0)/scal_factor;
    points3D[2]=X_and_alphas.at<double>(2,0)/scal_factor;

    //update the point 3D:
    if(point3D.empty())
      point3D = Ptr<cv::Vec3d>( new cv::Vec3d(points3D) );
    else
      *point3D = points3D;

    return errorEstimate(cameras, points_to_track, points3D);
  }
  
  double TrackPoints::triangulateRobust(std::vector<PointOfView>& cameras,
    const std::vector<cv::Ptr<PointsToTrack>> &points_to_track, cv::Vec3d& points3D,
    double reproj_error)
  {
    cv::RNG& rng = cv::theRNG();
    unsigned int nviews = images_indexes_.size();
    double distance=0, best_distance=1e20;
    vector<int> masks;
    cv::Vec3d bestPoints3D;

    if( !point3D.empty() )
    {
      double error = errorEstimate(cameras, points_to_track, *point3D);
      if( error<reproj_error )
      {
        points3D = *point3D;
        return error;
      }
    }

    int num_iter=0, max_iter=10;
    for(num_iter=0; num_iter<max_iter; ++num_iter)
    {
      masks.clear();
      int nb_vals=0;
      for (unsigned int cpt = 0; cpt < nviews; cpt++) {
        int valTmp = rng(2);
        if( valTmp>0 )
          nb_vals++;
        masks.push_back(valTmp);
      }
      while( nb_vals<2 )
      {
        int valTmp = rng(nviews);
        if( masks[valTmp] == 0 )
        {
          masks[valTmp] = 1;
          nb_vals++;
        }
      }
      //create mask:
      triangulateLinear( cameras, points_to_track, points3D, masks);

      distance = errorEstimate(cameras, points_to_track, points3D);
      if( distance < best_distance )
      {
        //new best model...
        bestPoints3D = points3D;
        best_distance = distance;
        if( best_distance<reproj_error )
          num_iter=nviews;//quit the loop!
      }
    }
    points3D = bestPoints3D;
    //update the point 3D:
    if(point3D.empty())
      point3D = Ptr<cv::Vec3d>( new cv::Vec3d(points3D) );
    else
      *point3D = points3D;
    return distance;
  }
  MotionEstimator::MotionEstimator(vector<Ptr<PointsToTrack>> &points_to_track,
    Ptr<PointsMatcher> match_algorithm)
    :points_to_track_(points_to_track),match_algorithm_(match_algorithm)
  {
  }


  MotionEstimator::~MotionEstimator(void)
  {
  }

  void MotionEstimator::addNewImagesOfPoints(Ptr<PointsToTrack> points)
  {
    points_to_track_.push_back(points);
  }

  void MotionEstimator::computeMatches()
  {
    //First compute missing features descriptors:
    vector<Ptr<PointsToTrack>>::iterator it=points_to_track_.begin();
    vector<Ptr<PointsToTrack>>::iterator end_iter = points_to_track_.end();
    while (it != end_iter)
    {
      Ptr<PointsToTrack> points_to_track_i=(*it);

      points_to_track_i->computeKeypointsAndDesc(false);

      it++;
    }

    //here, all keypoints and descriptors are computed.
    //Now create and train matcher:
    it=points_to_track_.begin();
    //We skip previous matcher already computed:
    it+=matches_.size();
    while (it != end_iter)
    {
      Ptr<PointsToTrack> points_to_track_i = (*it);
      Ptr<PointsMatcher> point_matcher = match_algorithm_->clone(true);
      point_matcher->add( points_to_track_i );
      point_matcher->train();
      matches_.push_back( point_matcher );

      it++;
    }

    //Now we are ready to match each picture with other:
    vector<Mat> masks;
    vector<Ptr<PointsMatcher>>::iterator matches_it=matches_.begin();
    vector<Ptr<PointsMatcher>>::iterator end_matches_it=matches_.end();
    unsigned int i=0,j=0;
    while (matches_it != end_matches_it)
    {
      Ptr<PointsMatcher> point_matcher = (*matches_it);

      //the folowing vector is computed only the first time

      vector<Ptr<PointsMatcher>>::iterator matches_it1 = matches_it+1;
      j=i+1;
      while (matches_it1 != end_matches_it)
      {
        Ptr<PointsMatcher> point_matcher1 = (*matches_it1);
        vector<DMatch> matches_i_j;

        point_matcher->crossMatch(point_matcher1,matches_i_j,masks);
//////////////////////////////////////////////////////////////////////////
//For now we use fundamental function from OpenCV but soon use libmv !


        //First compute points matches:
        int size_match=matches_i_j.size();
        Mat srcP(1,size_match,CV_32FC2);
        Mat destP(1,size_match,CV_32FC2);
        vector<uchar> status;

        //vector<KeyPoint> points1 = point_matcher->;
        for( int cpt = 0; cpt < size_match; ++cpt ){
          const KeyPoint &key1 = point_matcher1->getKeypoint(
            matches_i_j[cpt].queryIdx);
          const KeyPoint &key2 = point_matcher->getKeypoint(
            matches_i_j[cpt].trainIdx);
          srcP.at<float[2]>(0,cpt)[0] = key1.pt.x;
          srcP.at<float[2]>(0,cpt)[1] = key1.pt.y;
          destP.at<float[2]>(0,cpt)[0] = key2.pt.x;
          destP.at<float[2]>(0,cpt)[1] = key2.pt.y;
          status.push_back(1);
        }

        Mat fundam = cv::findFundamentalMat(srcP, destP, status, cv::FM_RANSAC);

        //refine the mathing :
        for( int cpt = 0; cpt < size_match; ++cpt ){
          if( status[cpt] == 0 )
          {
            status[cpt] = status[--size_match];
            status.pop_back();
            matches_i_j[cpt--] = matches_i_j[size_match];
            matches_i_j.pop_back();
          }
        }


//////////////////////////////////////////////////////////////////////////
        if(matches_i_j.size()>mininum_points_matches)
          addMatches(matches_i_j,i,j);
        
        std::clog<<"find matches between "<<i<<" "<<j<<std::endl;
        j++;
        matches_it1++;
      }
      i++;
      matches_it++;
    }
  }

  void MotionEstimator::keepOnlyCorrectMatches()
  {
    unsigned int tracks_size = tracks_.size();
    unsigned int index=0;

    while ( index < tracks_size )
    {
      if( tracks_[index].getNbTrack() <= mininum_image_matches )
      {
        //problem with this track, too small to be consistent
        tracks_size--;
        tracks_[index]=tracks_[tracks_size];
        tracks_.pop_back();
        index--;
      }
      index++;
    }
  }

  void MotionEstimator::addMatches(vector<DMatch> &newMatches,
    unsigned int img1, unsigned int img2)
  {
    //add to tracks_ the new matches:

    vector<DMatch>::iterator match_it = newMatches.begin();
    vector<DMatch>::iterator match_it_end = newMatches.end();

    while ( match_it != match_it_end )
    {
      DMatch &point_matcher = (*match_it);

      bool is_found=false;
      vector<TrackPoints>::iterator tracks_it = tracks_.begin();
      while ( tracks_it != tracks_.end() && !is_found )
      {
        TrackPoints& track = (*tracks_it);

        if(track.containPoint(img1,point_matcher.trainIdx))
        {
          track.addMatch(img2,point_matcher.queryIdx);
          is_found=true;
        }
        else
          tracks_it++;
      }
      if( !is_found )
      {
        //it's a new point match, create a new track:
        TrackPoints newTrack;
        newTrack.addMatch(img1,point_matcher.trainIdx);
        newTrack.addMatch(img2,point_matcher.queryIdx);
        tracks_.push_back(newTrack);
      }

      match_it++;
    }
  }
  void MotionEstimator::addTracks(vector<TrackPoints> &newTracks)
  {

    vector<TrackPoints>::iterator match_it = newTracks.begin();
    vector<TrackPoints>::iterator match_it_end = newTracks.end();

    while ( match_it != match_it_end )
    {
      tracks_.push_back(*match_it);

      match_it++;
    }
  }

  void MotionEstimator::showTracks(vector<Mat>& images,
    int timeBetweenImg)
  {

    //First compute missing features descriptors:
    unsigned int it=0;
    unsigned int end_iter = points_to_track_.size() - 1 ;
    if(images.size() - 1 < end_iter)
      end_iter = images.size() - 1;
    while (it < end_iter)
    {
      vector<DMatch> matches_to_print;
      //add to matches_to_print only points of img it and it+1:

      vector<TrackPoints>::iterator match_it = tracks_.begin();
      vector<TrackPoints>::iterator match_it_end = tracks_.end();

      while ( match_it != match_it_end )
      {
        if( match_it->containImage(it) &&
          match_it->containImage(it+1) )
        {
          matches_to_print.push_back(match_it->toDMatch(it,it+1));
        }
        match_it++;
      }

      Mat firstImg=images[ it ];
      Mat outImg;

      PointsMatcher::drawMatches(firstImg, points_to_track_[it]->getKeypoints(),
        points_to_track_[ it + 1 ]->getKeypoints(),
        matches_to_print, outImg,
        cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      imshow("showTracks",outImg);
      cv::waitKey(timeBetweenImg);

      it++;
    }
  }

  void MotionEstimator::read( const cv::FileNode& node, MotionEstimator& me )
  {
    std::string myName=node.name();
    if( myName != "MotionEstimator")
      return;//this node is not for us...

    if( node.empty() || !node.isMap() )
      CV_Error( CV_StsError, "MotionEstimator FileNode is not correct!" );

    int nb_pictures = (int) node["nbPictures"];
    //initialisation of various vector (empty)
    for(int i=0; i<nb_pictures; i++)
    {
      Ptr<PointsToTrack> ptt = Ptr<PointsToTrack>(new PointsToTrack());
      me.points_to_track_.push_back(ptt);

      Ptr<PointsMatcher> p_m = Ptr<PointsMatcher>(new PointsMatcher(
        *me.match_algorithm_ ));
      p_m->add(ptt);

      me.matches_.push_back(p_m);
    }

    cv::FileNode& node_TrackPoints = node["TrackPoints"];

    //tracks are stored in the following form:
    //list of track where a track is stored like this:
    // nbPoints idImage1 point1  idImage2 point2 ...
    if( node_TrackPoints.empty() || !node_TrackPoints.isSeq() )
      CV_Error( CV_StsError, "MotionEstimator FileNode is not correct!" );
    cv::FileNodeIterator it = node_TrackPoints.begin(),
      it_end = node_TrackPoints.end();
    while( it != it_end )
    {
      cv::FileNode it_track = (*it)[0];
      int nbPoints,track_consistance;
      it_track["nbPoints"] >> nbPoints;
      it_track["track_consistance"] >> track_consistance;
      TrackPoints track;
      cv::FileNodeIterator itPoints = it_track["track"].begin(),
        itPoints_end = it_track["track"].end();
      while( itPoints != itPoints_end )
      {
        int idImage;
        cv::KeyPoint kpt;
        idImage = (*itPoints)[0];
        itPoints++;
        kpt.pt.x = (*itPoints)[0];
        kpt.pt.y = (*itPoints)[1];
        kpt.size = (*itPoints)[2];
        kpt.angle = (*itPoints)[3];
        kpt.response = (*itPoints)[4];
        kpt.octave = (*itPoints)[5];
        kpt.class_id = (*itPoints)[6];

        unsigned int point_index = me.points_to_track_[idImage]->
          addKeypoint( kpt );
        track.addMatch(idImage,point_index);

        itPoints++;
      }
      track.track_consistance = track_consistance;
      me.tracks_.push_back(track);
      it++;
    }
  }

  void MotionEstimator::write( cv::FileStorage& fs, const MotionEstimator& me )
  {
    vector<TrackPoints>::size_type key_size = me.tracks_.size();
    int idImage=-1, idPoint=-1;

    fs << "MotionEstimator" << "{";
    fs << "nbPictures" << (int)me.points_to_track_.size();
    fs << "TrackPoints" << "[";
    for (vector<TrackPoints>::size_type i=0; i < key_size; i++)
    {
      const TrackPoints &track = me.tracks_[i];
      unsigned int nbPoints = track.getNbTrack();
      if( nbPoints > 0)
      {
        fs << "{" << "nbPoints" << (int)nbPoints;
        fs << "track_consistance" << track.track_consistance;
        fs << "track" << "[:";
        for (unsigned int j = 0; j < nbPoints ; j++)
        {
          idImage = track.images_indexes_[j];
          idPoint = track.point_indexes_[j];
          if( idImage>=0 && idPoint>=0 )
          {
            fs << idImage;
            fs  << "[:";

            const cv::KeyPoint kpt = me.points_to_track_[idImage]->
              getKeypoints()[idPoint];
            cv::write(fs, kpt.pt.x);
            cv::write(fs, kpt.pt.y);
            cv::write(fs, kpt.size);
            cv::write(fs, kpt.angle);
            cv::write(fs, kpt.response);
            cv::write(fs, kpt.octave);
            cv::write(fs, kpt.class_id);
            fs << "]" ;
          }
        }
        fs << "]" << "}" ;
      }
    }
    fs << "]" << "}";
  }

  void MotionEstimator::triangulateNView(vector<PointOfView>& cameras,
    vector<cv::Vec3d>& points3D)
  {
    //for each points:
    vector<TrackPoints>::size_type key_size = tracks_.size();
    vector<PointOfView>::size_type num_camera = cameras.size();
    int idImage=-1, idPoint=-1;
    vector<TrackPoints>::size_type i;
    /*
    libmv::vector<libmv::Mat34> Ps(key_size);
    for (i=0; i < num_camera; i++)
    {
      Mat projTmp = cameras[i].getProjectionMatrix();
      Eigen::Map<libmv::Mat43> eigen_tmp((double*)projTmp.data);
      Ps[i] = eigen_tmp.transpose();
    }*/

    for (i=0; i < key_size; i++)
    {
      TrackPoints &track = tracks_[i];
      unsigned int nviews = track.images_indexes_.size();
      
      unsigned int maxImg=0;
      for(unsigned int cpt=0;cpt<nviews;cpt++)
        if(maxImg<track.images_indexes_[cpt])
          maxImg=track.images_indexes_[cpt];
      CV_Assert(nviews < cameras.size());

      cv::Vec3d point_final;
      //double distance=track.triangulateRobust( cameras,points_to_track_, point_final );
      double distance=track.triangulateLinear( cameras,points_to_track_, point_final );

      //this is used to take only correct 3D points:
      if(distance<10)
        points3D.push_back(point_final);
      
    }
  }
}