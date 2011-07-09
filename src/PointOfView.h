#ifndef _GSOC_SFM_FIELD_OF_VIEW_H
#define _GSOC_SFM_FIELD_OF_VIEW_H 1

#include "macro.h" //SFM_EXPORTS
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Camera.h"
#include "TracksOfPoints.h"
#include <vector>
#include <string>


namespace OpencvSfM{

  class SFM_EXPORTS Camera;///<We will need this class, but Camera need our class too...
  class SFM_EXPORTS TrackPoints;

  enum paramsToEstimate
  {
    ROTATION_OK=1,    ///<Mask used to know if the rotation parameters should be estimated or not
    TRANSLATION_OK=2 ///<Mask used to know if the translation parameters should be estimated or not
  };

  /*! \brief This class represent the 3D position of the device which take the pictures. 
  *      The role of the class is to store everything related to the filed of view: picture, 3D position, points, matches and 3D points
  *
  * We use the so-called pinhole camera model. That is, a scene view is formed by projecting 3D points into the image plane using a perspective transformation.
  * Usual notation says that a point [u,v] from an image is related to the point [X,Y,Z] using the following notation :
  * \f[
  * s  \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} =  \begin{bmatrix}f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
  * \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_1  \\ r_{21} & r_{22} & r_{23} & t_2  \\ r_{31} & r_{32} & r_{33} & t_3 \end{bmatrix}
  * \begin{bmatrix} X \\ Y \\ Z \\ 1  \end{bmatrix}
  * \f]
  *
  * This leads to the following relation between local coordinates and global ones:
  * \f[
  * \begin{array}{l} \vspace{10pt}
  * \begin{bmatrix} x \\ y \\ z \end{bmatrix} = R  \begin{bmatrix} X \\ Y \\ Z \end{bmatrix} + t \\
  * x' = x/z \\ y' = y/z \vspace{10pt} 
  * \end{array}
  * \f]
  * 
  */
  class SFM_EXPORTS PointOfView
  {
  protected:
    cv::Mat rotation_;///<Rotation matrix R (data is stored into projection_matrix_)
    cv::Mat translation_;///<Translation vector t (Matrix instead of vector because data is stored into projection_matrix_)
    cv::Mat projection_matrix_;///<redundancy but speed improvement
    cv::Ptr<Camera> device_;///<intra parameters and distortion coefs

    unsigned char config_;///<This attribut is used to know what we should estimate... If equal to 0, nothing should be estimated...

  public:
    /**
    * To create a point of view, we need two things : a camera, and a point (with orientation).
    * Here we give an address of a Camera, and the file name of the picture.
    * If we have more informations, we can use the last parameters...
    * @param device address of existing Camera. This camera can be calibrated or not...
    * @param rotation Matrix of the known rotation (optional)...
    * @param translation Vector of the known translation (optional)...
    */
    PointOfView(cv::Ptr<Camera> device,cv::Mat rotation=cv::Mat::eye(3, 3, CV_64F),cv::Vec3d translation=cv::Vec3d(0.0,0.0,0.0));
    /**
    * Destructor of PointOfView, release all vectors... TODO: define how we should release the vectors...
    */
    virtual ~PointOfView(void);

    /**
    * use this function to get acces to the camera parameters
    * @return camera matrix
    */
    cv::Ptr<Camera> getIntraParameters() const{return device_;};
    
    /**
    * This method can convert 3D points from world coordinates to 2D points in pixel image coordinates
    * @param points 3D points in world coordinates.
    * @return 2D points in pixel image coordinates.
    */
    virtual std::vector<cv::Vec2d> project3DPointsIntoImage(std::vector<TrackPoints> points) const;
    /**
    * This method can convert 3D points from world coordinates to 2D points in pixel image coordinates
    * @param points 3D points in world coordinates.
    * @return 2D points in pixel image coordinates.
    */
    virtual std::vector<cv::Vec2d> project3DPointsIntoImage(std::vector<cv::Vec3d> points) const;
    /**
    * This method can convert 3D point from world coordinates to 2D point in pixel image coordinates
    * @param point 3D point in world coordinates.
    * @return 2D point in pixel image coordinates.
    */
    virtual cv::Vec2d project3DPointIntoImage(cv::Vec3d point) const;
    /**
    * This method test is 3D point is in front of Camera (can be view with the camera)
    * @param point 3D point in world coordinates (homogeneous, that is 4 values).
    * @return true if point can be seen with this point of view
    */
    virtual bool pointInFrontOfCamera(cv::Vec4d point) const;
    /**
    * This method return the intra parameters of the camera
    * @return Matrix K of intra parameters
    */
    virtual cv::Mat getProjectionMatrix() const;

    virtual cv::Mat getRotationMatrix() const
    {
      return rotation_;
    };
    virtual void setRotationMatrix(cv::Mat newRot)
    {
      for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
          rotation_.at<double>(i,j) = newRot.at<double>(i,j);
    };

    virtual cv::Mat getTranslationVector() const
    {
      return translation_;
    };

    virtual void setTranslationVector(cv::Mat newVect)
    {
      for(int i=0; i<3; ++i)
        translation_.at<double>(i,0) = newVect.at<double>(i,0);
    };
  };

}

#endif