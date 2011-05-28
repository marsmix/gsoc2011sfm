#ifndef _GSOC_SFM_FIELD_OF_VIEW_H
#define _GSOC_SFM_FIELD_OF_VIEW_H 1

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Camera.h"
#include "PointsToTrack.h"
#include "PointsMatcher.h"
#include "ExternPosEstim.h"
#include "Points3DEstim.h"
#include <vector>
#include <string>


namespace OpencvSfM{

  class Camera;///<We will need this class, but Camera need our class too...

#define MASK_ROTATION_OK 0x01  ///<Mask used to know if the rotation parameters should be estimated or not
#define MASK_TRANSLATION_OK 0x02  ///<Mask used to know if the translation parameters should be estimated or not
#define MASK_POINTS_OK 0x04  ///<Mask used to know if we have to estimate the points or not
#define MASK_MATCHES_OK 0x08  ///<Mask used to know if we have to estimate the matches or not

#define MASK_FIELD_NOT_LOADED 0x80  ///<Mask used to know if the object is correct or not (i.e. if the FoV is not initialised)

  /*! \brief This class represent the 3D position of the device which take the pictures. 
  *      The role of the class is to store everything related to the filed of view: picture, 3D position, points, matches and 3D points
  *
  * We use the so-called pinhole camera model. That is, a scene view is formed by projecting 3D points into the image plane using a perspective transformation.
  * Usual notation says that a point [u,v] from an image is related to the point [X,Y,Z] using the following notation :
  * /f[
  * s  \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} =  \begin{bmatrix}f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
  * \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_1  \\ r_{21} & r_{22} & r_{23} & t_2  \\ r_{31} & r_{32} & r_{33} & t_3 \end{bmatrix}
  * \begin{bmatrix} X \\ Y \\ Z \\ 1  \end{bmatrix}
  * /f]
  *
  * This leads to the following relation between local coordinates and global ones:
  * /f[
  * \begin{array}{l} \vspace{10pt}
  * \begin{bmatrix} x \\ y \\ z \end{bmatrix} = R  \begin{bmatrix} X \\ Y \\ Z \end{bmatrix} + t \\
  * x' = x/z \\ y' = y/z \vspace{10pt} 
  * \end{array}
  * /f]
  * 
  */
  class FieldOfView
  {
  protected:
    cv::Mat image_;///<The picture we get from this position... Could be RGB, B&W...
    cv::Mat rotation_;///<Rotation matrix R
    cv::Vec<double, 3> translation_;///<Translation vector t
    cv::Mat projection_matrix_;///<redundancy but speed improvement
    Camera *device_;///<intra parameters and distortion coefs
    PointsToTrack* pointsInfo_;///<Points which are found in the picture
    PointsMatcher* matches_;///<This object will store the differents matches estimations between this FoV and other related views... ! This object will be shared between different FoV !
    ExternPosEstim* extern_position_;//An object which can be used to compute a bundle adjustement, using the differents matching of points... ! This object will be shared between different FoV !
    Points3DEstim* points3D_;//for each point in pointsInfo, estimation of the 3D position... ! This object will be shared between different FoV !

    unsigned char config_;///<This attribut is used to know what we should estimate... If equal to 0, everything should be estimated...

  public:
    /**
    * If we use this contructor, the object is not correct! Indeed, we don't have the needed informations like the camera wich take this FoV.
    * Use isRealFoV to test the validity of objects if needed.
    */
    FieldOfView();
    /**
    * To create a field of view, we need two things : a camera, and an picture.
    * Here we give an address of a Camera, and the file name of the picture.
    * If we have more informations, we can use the last parameters...
    * @param device address of existing Camera. This camera can be calibrated or not...
    * @param imgFileName file name of the image taken from the device...
    * @param rotation Matrix of the known rotation (optional)...
    * @param translation Vector of the known translation (optional)...
    */
    FieldOfView(Camera *device,std::string imgFileName,cv::Mat rotation=cv::Mat::eye(3, 3, CV_64F),cv::Vec<double, 3> translation=cv::Vec<double, 3>(0.0,0.0,0.0));
    /**
    * To create a field of view, we need two things : a camera, and an picture.
    * Here we give an address of a Camera, and the picture (matrix).
    * If we have more informations, we can use the last parameters...
    * @param device address of existing Camera. This camera can be calibrated or not...
    * @param image Matrix of the image previously loaded...
    * @param rotation Matrix of the known rotation (optional)...
    * @param translation Vector of the known translation (optional)...
    */
    FieldOfView(Camera *device,cv::Mat image,cv::Mat rotation=cv::Mat::eye(3, 3, CV_64F),cv::Vec<double, 3> translation=cv::Vec<double, 3>(0.0,0.0,0.0));
    /**
    * To create a field of view, we need two things : a camera, and an picture.
    * Somethinmes, we don't have the picture but only points: we can create the field of view using this constructor...
    * If we have more informations, we can use the last parameters...
    * @param device address of existing Camera. This camera can be calibrated or not...
    * @param points points to track and optionally the associated features...
    * @param matches object where matches between our points and other FoV's points (optional)
    * @param rotation Matrix of the known rotation (optional)...
    * @param translation Vector of the known translation (optional)...
    */
    FieldOfView(Camera *device,PointsToTrack *points,PointsMatcher *matches=NULL,cv::Mat rotation=cv::Mat::eye(3, 3, CV_64F),cv::Vec<double, 3> translation=cv::Vec<double, 3>(0.0,0.0,0.0));
    /**
    * This constructor will not copy values... This allow smart pointer behavior
    */
    FieldOfView(FieldOfView& otherFieldOfView);
    /**
    * Destructor of FieldOfView, release all vectors... TODO: define how we should release the vectors...
    */
    virtual ~FieldOfView(void);
    
    /**
    * Use isRealFoV to test the validity of objects if needed.
    * @return true is the object is correctly loaded, false if you should not use it !
    */
    bool isRealFoV(){return (config_&MASK_FIELD_NOT_LOADED)==0;};
  };

}

#endif