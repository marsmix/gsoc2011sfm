#include "MotionEstimator.h"

#include <iostream>
using cv::Ptr;
using cv::Mat;
using cv::DMatch;
using cv::KeyPoint;
using std::vector;

namespace OpencvSfM{


  bool TrackPoints::addMatch(int image_src, int point_idx1)
  {
    //If a track contains more than one keypoint in the same image,
    //it is deemed inconsistent.
    if(containImage(image_src))
      track_is_inconsistent = true;

    images_indexes_.push_back(image_src);
    point_indexes_.push_back(point_idx1);
    return !track_is_inconsistent;
  }

  bool TrackPoints::containImage(int image_wanted)
  {
    if(std::find(images_indexes_.begin(),images_indexes_.end(),image_wanted) ==
      images_indexes_.end())
      return false;
    return true;
  }

  bool TrackPoints::containPoint(int image_src, int point_idx1)
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

  DMatch TrackPoints::toDMatch(int img1,int img2)
  {
    DMatch outMatch;
    char nbFound=0;
    //we don't use find here because we want the number instead of iterator...
    vector<unsigned int>::iterator indexImg = images_indexes_.begin();
    vector<unsigned int>::iterator end_iter = images_indexes_.end();
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


  MotionEstimator::MotionEstimator(vector<Ptr<PointsToTrack>> &points_to_track,
    Ptr<PointsMatcher> match_algorithm)
    :points_to_track_(points_to_track),match_algorithm_(match_algorithm)
  {
  }


  MotionEstimator::~MotionEstimator(void)
  {
  }

  void MotionEstimator::addNewPointsToTrack(Ptr<PointsToTrack> points)
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
      if( tracks_[index].getNbTrack() <= mininum_image_matches)
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
      Mat secondImg=images[ it + 1 ];
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
}