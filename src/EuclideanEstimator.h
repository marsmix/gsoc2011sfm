#ifndef _GSOC_SFM_POINTS_3D_EUCLIDEAN_ESTIMATION_H
#define _GSOC_SFM_POINTS_3D_EUCLIDEAN_ESTIMATION_H 1

#include "macro.h" //SFM_EXPORTS
#include "SequenceAnalyzer.h"
#include "PointOfView.h"
#include "opencv2/core/eigen.hpp"
#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"
#include <iostream>

namespace OpencvSfM{

  /** \brief This class perform a projective estimation
  * of the motion. Given points matches and cameras with intra parameters,
  * it tries to find the best cameras positions and 3D points. Does not
  * perform a bundle ajustement!
  *
  * As this class use a lot of libmv functions, the data members are
  * using libmv structures...
  */
  class SFM_EXPORTS EuclideanEstimator
  {
  protected:
    int index_origin;///<index of camera set as origin...
    libmv::vector<libmv::Mat3> intra_params_;
    libmv::vector<libmv::Mat3> rotations_;
    libmv::vector<libmv::Vec3> translations_;
    std::vector<PointOfView>& cameras_;
    std::vector<bool> camera_computed_;
    std::vector< TrackOfPoints > point_computed_;
    SequenceAnalyzer &sequence_;
  public:
    EuclideanEstimator( SequenceAnalyzer &sequence,
      std::vector<PointOfView>& cameras );
    virtual ~EuclideanEstimator( void );

    /**
    * Add a new camera to the estimator
    * @param camera new point of view to add for reconstruction
    */
    inline void addNewPointOfView( const PointOfView& camera )
    {
      libmv::Mat3 intra_param;
      cv::Ptr<Camera> intra=camera.getIntraParameters( );
      cv::cv2eigen( intra->getIntraMatrix( ).t( ), intra_param );
      intra_params_.push_back( intra_param );
      libmv::Mat3 rotation_mat;
      cv::cv2eigen( camera.getRotationMatrix( ), rotation_mat );
      rotations_.push_back( rotation_mat );
      libmv::Vec3 translation_vec;
      cv::cv2eigen( camera.getTranslationVector( ), translation_vec );
      translations_.push_back( translation_vec );
      camera_computed_.push_back( false );
    }

    /**
    * comptue cameras and structure if intra parameters are known.
    */
    void computeReconstruction( );

  protected:
    void initialReconstruction( std::vector<TrackOfPoints>& tracks,
      int image1, int image2 );

    bool cameraResection( unsigned int image );

    void bundleAdjustement( );
  };

}

#endif
