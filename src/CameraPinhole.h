#ifndef _GSOC_SFM_CAMERA_PINHOLE_H
#define _GSOC_SFM_CAMERA_PINHOLE_H 1

//A lot of methods are inspired of LIBMV project (http://code.google.com/p/libmv/)

#include "opencv2/calib3d/calib3d.hpp"
#include "Camera.h"

namespace OpencvSfM{
  enum paramsOfIntraParameters{
    FOCAL_PARAM=1,         ///<Mask used to know if the focal parameters are estimated or not
    SKEW_PARAM=2,          ///<Mask used to know if the skew parameter is estimated or not
    PRINCIPAL_POINT_PARAM=4///<Mask used to know if the principal point estimated or not
  };

  /*! \brief This class represent the physical device which take the pictures. 
  *      It is not related to a 3D position which is the role of the PointOfView class.
  *      The role of the class is to store only intra parameters (without radial distortion)
  *
  *
  * So this class is devoted to the conversion between 3D points (using camera coordinate) and 2D points (using image coordinate) using the methods convertFromImageTo3Dray or convertFrom3DToImage
  */
  class CameraPinhole:public Camera{
  protected:
    cv::Mat intra_params_;///<store intra parameters(3*3 matrix). This matrix contains focal informations, principal point coordinates and skew of axis
    cv::Mat inv_intra_params_;///<This is the inverse transformation of intra_params_. Used to speed up calculus...

    /**
    * This attribut is used to know what we should estimate...
    * Example: if equal to 0, nothing should be estimated...
    * If equal to 3, focal and skew should be estimated ( FOCAL_PARAM + SKEW_PARAM)
    */
    unsigned char estimation_needed_;
  public:
    /**
    * Constructor with (or not) intra parameters.
    * @param intra_params matrix of intra parameters (3*3)
    * @param wantedEstimation values which need an estimation
    */
    CameraPinhole(cv::Mat intra_params=cv::Mat::eye(3, 3, CV_64F),unsigned char wantedEstimation=FOCAL_PARAM|SKEW_PARAM|PRINCIPAL_POINT_PARAM);
    /**
    * Constructor where initial camera matrix is computed from the 3D-2D point correspondences.
    * Currently, the function only supports planar calibration patterns, i.e. patterns where each object point has z-coordinate =0.
    * @param objectPoints The vector of vectors of the object points. See http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#cv-calibratecamera
    * @param imagePoints The vector of vectors of the corresponding image points. See http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#cv-calibratecamera
    * @param imageSize The image size in pixels; used to initialize the principal point
    * @param aspectRatio If it is zero or negative, both \f$f_x\f$  and \f$f_y\f$  are estimated independently. Otherwise \f$f_x=f_y * aspectRatio\f$ 
    * @param wantedEstimation values which need an estimation
    */
    CameraPinhole(const std::vector<std::vector<cv::Point3f> >& objectPoints, const std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Size imageSize, double aspectRatio=1.,unsigned char wantedEstimation=FOCAL_PARAM|SKEW_PARAM|PRINCIPAL_POINT_PARAM);
    ~CameraPinhole();

    /**
    * this method can be used to update the intra parameters.
    * @param newParams matrix of new parameters (3*3)
    * @param intraValues values which are useful in matrix
    */
    void updateIntrinsicMatrix(cv::Mat newParams,unsigned char intraValues=FOCAL_PARAM|SKEW_PARAM|PRINCIPAL_POINT_PARAM);

    /**
    * This method can transform points from image to 3D rays
    */
    virtual std::vector<cv::Vec4d> convertFromImageTo3Dray(std::vector<cv::Vec3d> points);
    
    /**
    * This method can convert 2D points from pixel image coordinates to 2D points in normalized image coordinates
    * @param points 2D points in pixel image homogeneous coordinates.
    * @return 2D points in normalized image homogeneous coordinates.
    */
    virtual std::vector<cv::Vec2d> pixelToNormImageCoordinates(std::vector<cv::Vec2d> points);
    /**
    * This method can convert 2D points from normalized image coordinates to 2D points in pixel image coordinates
    * @param points 2D points in normalized image homogeneous coordinates.
    * @return 2D points in pixel image coordinates.
    */
    virtual std::vector<cv::Vec2d> normImageToPixelCoordinates(std::vector<cv::Vec2d> points);
    /**
    * This method can create a projection matrix using intra parameters and given rotation and translation
    * @param rotation rotation matrix
    * @param translation translation vector
    * @return Projection matrix (4*3)
    */
    virtual cv::Mat computeProjectionMatrix(const cv::Mat &rotation,const cv::Vec3d &translation);
  };

};

#endif