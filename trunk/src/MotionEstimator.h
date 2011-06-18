#ifndef _GSOC_SFM_EXTERN_POS_ESTIM_H
#define _GSOC_SFM_EXTERN_POS_ESTIM_H 1

#include "PointsToTrack.h"
#include "MotionProcessor.h"
#include "PointsMatcher.h"

namespace OpencvSfM{

  /**
  * \brief This class store the track of keypoints.
  * A track is a connected set of matching keypoints across multiple images
  * 
  */
  class TrackPoints
  {
  protected:
    std::vector<unsigned int> images_indexes_;
    std::vector<unsigned int> point_indexes_;
    bool track_is_inconsistent;
  public:
    TrackPoints():track_is_inconsistent(false){};
    /**
    * This function add matches to track
    * @param image_src index of source matches image
    * @param point_idx index of point in source image
    * @return true if this match is correct, false if inconsistent with
    * Snavely's rules.
    */
    inline bool addMatch(int image_src, int point_idx);
    
    /**
    * This function is used to know if the track contains the image
    * @param image_wanted index of query image
    * @return true if this track contain points from the query image
    */
    inline bool containImage(int image_wanted);
    /**
    * This function is used to know if the track contains the query point
    * @param image_src index of query image
    * @param point_idx1 index of point in query image
    * @return true if this track contain the point from the query image
    */
    inline bool containPoint(int image_src, int point_idx1);
    /**
    * This function is used to get the numbers of image for this track
    * @return 0 if inconsistent, >= 2 else
    */
    inline unsigned int getNbTrack()
    {return track_is_inconsistent?0:images_indexes_.size();};
    /**
    * use this function to create a DMatch value from this track
    * @param img1 train match image
    * @param img2 query match image
    */
    inline cv::DMatch toDMatch(int img1,int img2);
  };

  /**
  * \brief This class tries to match points in the entire sequence.
  * It follow ideas proposed by Noah Snavely:
  * Modeling the World from Internet Photo Collections
  * 
  * This class process an input video to first extracts the
  * features, then matches them and keeps them only when
  * there is more than 3 pictures containing the point.
  */
  class MotionEstimator
  {
  protected:
    static const int mininum_points_matches = 20;
    static const int mininum_image_matches = 3;
    /**
    * One list of points for each picture
    */
    std::vector<cv::Ptr<PointsToTrack>> points_to_track_;
    /**
    * The matcher algorithm we should use to find matches.
    */
    cv::Ptr<PointsMatcher> match_algorithm_;
    /**
    * A matcher for each picture. Its role is to find quickly matches between
    * i^th picture and other images.
    */
    std::vector<cv::Ptr<PointsMatcher>> matches_;
    /**
    * List of each tracks found. A track is a connected set of matching
    * keypoints across multiple images
    */
    std::vector<TrackPoints> tracks_;
  public:
    /**
    * Constructor taking a vector of points to track and a PointsMatcher
    * algorithm to find matches.
    * @param mp points_to_track list of points to track with (or not) features
    * @param match_algorithm algorithm to match points of each images
    */
    MotionEstimator(std::vector<cv::Ptr<PointsToTrack>> &points_to_track,
      cv::Ptr<PointsMatcher> match_algorithm );

    ~MotionEstimator(void);
    /**
    * This method add new points to track. When adding, the matches are not
    * computed, use computeMatches to compute them!
    * @param points extracted points with features vectors.
    */
    void addNewPointsToTrack(cv::Ptr<PointsToTrack> points);
    
    /**
    * This method compute the matches between each points of each images.
    * It first compute missing features descriptor, then train each matcher.
    * Finally compute tracks of keypoints (a track is a connected set of 
    * matching keypoints across multiple images)
    */
    void computeMatches();
    /**
    * This method keep only tracks with more than mininum_image_matches
    */
    void keepOnlyCorrectMatches();
    /**
    * This method can be used to get the tracks
    */
    inline std::vector<TrackPoints> &getTracks(){return tracks_;};
    /**
    * Use this function to print the sequence of matches
    * @param images list of images where print points
    * @param timeBetweenImg see cv::waitKey for the value
    */
    void showTracks(std::vector<cv::Mat>& images,int timeBetweenImg=25);
  protected:
    /**
    * This function add matches to tracks
    * @param newMatches new matches to add
    * @param img1 index of source matches image
    * @param img2 index of destination matches image
    * @return 
    */
    inline void addMatches(std::vector<cv::DMatch> &newMatches,
      unsigned int img1, unsigned int img2);
  };

}

#endif