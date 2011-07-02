#ifndef _GSOC_SFM_POINTS_3D_ESTIMATION_H
#define _GSOC_SFM_POINTS_3D_ESTIMATION_H 1

#include "SequenceAnalyzer.h"
#include "PointOfView.h"
#include "libmv_mapping.h"
#include <iostream>

namespace OpencvSfM{

  /** \brief This class perform a projective estimation
  * of the motion. Given points matches and cameras with intra parameters,
  * it tries to find the best cameras positions and 3D points. Does not 
  * perform a bundle ajustement!
  * 
  * As this class use a lot of libmv functions, the parameters are
  * using libmv structures...
  */
  class ProjectiveEstimator
  {
  protected:
    int index_origin;///<index of camera set as origin...
    libmv::vector<libmv::Mat3> intra_params_;
    libmv::vector<libmv::Mat3> rotations_;
    libmv::vector<libmv::Vec3> translations_;
    std::vector<PointOfView>& cameras_;
    SequenceAnalyzer &sequence_;
  public:
    ProjectiveEstimator(SequenceAnalyzer &sequence,
      std::vector<PointOfView>& cameras);
    virtual ~ProjectiveEstimator(void);

    /**
    * Add a new camera to the estimator
    * @param camera new point of view to add for reconstruction
    */
    inline void addNewPointOfView(const PointOfView& camera)
    {
      libmv::Mat3 intra_param;
      cv::Ptr<Camera> intra=camera.getIntraParameters();
      libmv::convertCvMatToEigen(intra->getIntraMatrix(),intra_param);
      intra_params_.push_back(intra_param);
      libmv::Mat3 rotation_mat;
      libmv::convertCvMatToEigen(camera.getRotationMatrix(),rotation_mat);
      rotations_.push_back(rotation_mat);
      libmv::Vec3 translation_vec;
      libmv::convertCvMatToEigen(camera.getTranslationVector(),translation_vec);
      translations_.push_back(translation_vec);
    }
    
    /**
    * improve cameras positions.
    */
    void comptueReconstruction();

  protected:
    void updateTwoViewMotion(std::vector<TrackPoints>& tracks,
      std::vector<cv::Ptr<PointsToTrack>> &points_to_track,
      int image1, int image2);
  };

}

#endif