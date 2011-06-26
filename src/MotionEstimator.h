#ifndef _GSOC_SFM_EXTERN_POS_ESTIM_H
#define _GSOC_SFM_EXTERN_POS_ESTIM_H 1

#include "PointsToTrack.h"
#include "MotionProcessor.h"
#include "PointsMatcher.h"
#include "PointOfView.h"
#include "opencv2/calib3d/calib3d.hpp"

namespace OpencvSfM{

  class MotionEstimator;
  /**
  * \brief This class store the track of keypoints.
  * A track is a connected set of matching keypoints across multiple images
  * 
  * This class can be used as a Vec3d because it's the projection of a 3D points
  * Of course, use triangulate method before to create this 3D point!
  * 
  * Discussion: Store index of points or 2D position?
  */
  class TrackPoints
  {
    friend class MotionEstimator;

  protected:
    cv::Ptr<cv::Vec3d> point3D;
    std::vector<unsigned int> images_indexes_;
    std::vector<unsigned int> point_indexes_;
    /**
    * if <0 the track is inconsistent
    * if >0 represent the degree of consistence (higher is better)
    */
    int track_consistance;
  public:
    /**
    * cast operator to use this object as a 3D point!
    */
    operator cv::Ptr<cv::Vec3d>() {
      return point3D;
    }
    TrackPoints():track_consistance(0){};
    /**
    * This function add matches to track
    * @param image_src index of source matches image
    * @param point_idx index of point in source image
    * @return true if this match is correct, false if inconsistent with
    * Snavely's rules.
    */
    inline bool addMatch(const int image_src, const int point_idx);
    
    /**
    * This function is used to know if the track contains the image
    * @param image_wanted index of query image
    * @return true if this track contain points from the query image
    */
    inline bool containImage(const int image_wanted);
    /**
    * This function is used to know if the track contains the query point
    * @param image_src index of query image
    * @param point_idx1 index of point in query image
    * @return true if this track contain the point from the query image
    */
    inline bool containPoint(const int image_src, const int point_idx1);
    /**
    * This function is used to get the numbers of image for this track
    * @return 0 if inconsistent, >= 2 else
    */
    inline unsigned int getNbTrack() const
    {return track_consistance<0?0:images_indexes_.size();};
    /**
    * use this function to create a DMatch value from this track
    * @param img1 train match image
    * @param img2 query match image
    * @return DMatch value
    */
    inline cv::DMatch toDMatch(const int img1,const int img2) const;
    /**
    * use this function to get the n^th match value from this track
    * @param index which match
    * @param idImage out value of the image index
    * @param idPoint out value of the point index
    */
    inline void getMatch(const unsigned int index,
      int &idImage, int &idPoint) const;
    /**
    * use this function to get the index point of the wanted image
    * @param image index of wanted image
    * @return index of point
    */
    inline int getIndexPoint(const unsigned int image) const;
    inline double triangulateLinear(std::vector<PointOfView>& cameras,
      const std::vector<cv::Ptr<PointsToTrack>> &points_to_track,
      cv::Vec3d& points3D,
      const std::vector<int> &masks = std::vector<int>());
    inline double triangulateRobust(std::vector<PointOfView>& cameras,
      const std::vector<cv::Ptr<PointsToTrack>> &points_to_track,
      cv::Vec3d& points3D,
      double reproj_error = 4);
  protected:
    inline double errorEstimate(std::vector<PointOfView>& cameras,
      const std::vector<cv::Ptr<PointsToTrack>> &points_to_track,
      cv::Vec3d& points3D) const;
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
    void addNewImagesOfPoints(cv::Ptr<PointsToTrack> points);
    
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

    /**
    * Project previously 2D points matches using cameras parameters
    * @param cameras Input of cameras matrix with extrinsic parameters
    * @param points3D output of points in 3D
    */
    void triangulateNView(std::vector<PointOfView>& cameras,
      std::vector<cv::Vec3d>& points3D);

    static void read( const cv::FileNode& node, MotionEstimator& points );

    static void write( cv::FileStorage& fs, const MotionEstimator& points );
    
    inline int getNumViews() const
    {
      unsigned int maxImg=0;
      std::vector<TrackPoints>::size_type key_size = tracks_.size(),
        i=0;
      for (i=0; i < key_size; i++)
      {
        const TrackPoints &track = tracks_[i];
        int nviews = track.images_indexes_.size();
        for(int j = 0;j<nviews;++j)
          if(maxImg<track.images_indexes_[j])
            maxImg=track.images_indexes_[j];
      }
      return maxImg;
    }

    /**
    * This function add matches to tracks
    * @param newMatches new matches to add
    * @param img1 index of source matches image
    * @param img2 index of destination matches image
    */
    inline void addMatches(std::vector<cv::DMatch> &newMatches,
      unsigned int img1, unsigned int img2);
    /**
    * This function add new Tracks
    * @param newTracks new Tracks to add
    */
    void addTracks(std::vector<TrackPoints> &newTracks);
  };

}

#endif