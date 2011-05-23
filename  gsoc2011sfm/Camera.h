#ifndef _GSOC_SFM_CAMERA_H
#define _GSOC_SFM_CAMERA_H 1

#include "opencv2/core/core.hpp"
#include "FieldOfView.h"
#include <vector>
#include <string>
#include <sstream>

using cv::Mat;
using cv::Vec;
using cv::VideoCapture;
using std::vector;
using std::string;
using std::ostringstream;

namespace OpencvSfM{
	class FieldOfView;///<We will need this class, but FieldOfView need our class too...

#define MASK_FOCAL_OK			0x01	///<Mask used to know if the focal parameters should be estimated or not
#define MASK_SKEW_OK			0x02	///<Mask used to know if the focal parameters should be estimated or not
#define MASK_PRINCIPAL_POINT_OK 0x04	///<Mask used to know if the focal parameters should be estimated or not
#define MASK_RADIAL_OK			0x08	///<Mask used to know if the radial parameters should be estimated or not
#define MASK_TANGEANT_OK		0x10	///<Mask used to know if the tangential parameters should be estimated or not

#define MASK_IS_FILE			0x01	///<Mask used to know if the input is a single file
#define MASK_IS_LIST_FILES		0x02	///<Mask used to know if the input is a list of files
#define MASK_IS_WEBCAM			0x04	///<Mask used to know if the input is a webcam
#define MASK_IS_AVI				0x08	///<Mask used to know if the input is a video file

	/*! \brief This class represent the physical device which take the pictures. 
	*			It is not related to a 3D position which is the role of the class FieldOfView.
	*			The role of the class is to store only device related informations like intra parameters, radial and tangential distotion.
	*
	* This class can be used to store device related informations like intra parameters, radial and tangential distotion.
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
	* Additionnal radial and tangeancial distortion are modelized like this:
	* /f[
	* \begin{array}{l}
	* \vspace{10pt} x'' = x'  \dfrac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6} + 2 p_1 x' y' + p_2(r^2 + 2 x'^2)  \\
	* \vspace{10pt} y'' = y'  \dfrac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6} + p_1 (r^2 + 2 y'^2) + 2 p_2 x' y'  \\
	* \text{where} \quad r^2 = x'^2 + y'^2  \\ u = f_x*x'' + c_x \\ v = f_y*y'' + c_y \end{array}
	* /f]
	* radial_dist_ can be used to store /f$k_1/f$ to /f$k_6/f$
	* tangential_dist_ can be used to store /f$p_1/f$ and /f$p_2/f$
	*/
	class Camera
	{
	protected:
		Mat intra_params_;///<store intra parameters(3*3 matrix). This matrix contains focal informations, principal point coordinates and skew of axis
		Mat inv_intra_params_;///<This is the inverse transformation of intra_params_. Used to speed up calculus...
		Vec<double, 6> radial_dist_;///<used to store radial dist parameters (/f$k_1/f$ to /f$k_6/f$)
		Vec<double, 2> tangential_dist_;///<used to store tangential dist parameters (/f$p_1/f$ and /f$p_2/f$)

		unsigned char config_;///<This attribut is used to know what we should estimate... If equal to 0, everything should be estimated...
		unsigned char type_of_input_;///<This attribut is used to know which type is the input (webcam, video file, list of file or just one image)
		VideoCapture capture_;///<When the camera is attached to an avi file or webcam, this will be usefull to get frame...
		/**
		* When the camera is attached to a list of file, sourceName_ will be used to store name of the prefix.
		* For example, if the files are img1.jpg, img2.jpg, ... img125.jpg, sourceName_ will be equal to img
		*/
		string sourceName_;
		/**
		* When the camera is attached to a list of file, suffix_ will be used to store name of the suffix.
		* For example, if the files are img1.jpg, img2.jpg, ... img125.jpg, suffix_ will be equal to .jpg
		*/
		string suffix_;
		/**
		* When the camera is attached to a list of file, numFrame_ will be used to know how many frames we have take.
		*/
		int numFrame_;

		vector<FieldOfView> pointsOfView_;///<vector of the differents positions of the camera. This will store images too.
	public:
		Camera(Mat intra_params=Mat::eye(3, 3, CV_64F),Vec<double, 6> radial_dist=Vec<double, 6>(0.0,0.0,0.0,0.0,0.0,0.0),Vec<double, 2> tangential_dist=Vec<double, 2>(0.0,0.0));
		virtual ~Camera(void);
		
		/**
		* You can attach this camera to a webcam
		* use this method to set it as the input source!
		* @param idWebCam id of the webcam
		* @return true if input source opened without problems
		*/
		bool setInputSource(int idWebCam);
		/**
		* You can attach this camera to a video file or a picture.
		* use this method to set it as the input source!
		* @param nameOfFile name of the media file (picture or avi movie)
		* @return true if input source opened without problems
		*/
		bool setInputSource(string nameOfFile);
		/**
		* You can attach this camera to a list of file.
		* use this method to set the input source!
		* For example, if the files are img1.jpg, img2.jpg, ... img125.jpg, prefix will be equal to "img", suffix to ".jpg" and startNumber equal to 1
		* @param prefix the part of the files names which stay the same (img)
		* @param suffix the type of the files (.jpg for instance)
		* @param startNumber the first number to use...
		* @return true if input source opened without problems
		*/
		bool setInputSource(string prefix,string suffix,int startNumber=0);
		/**
		* use this method if you want to get a field of view from this camera.
		* Be carreful : this function can return a fake FieldOfView! Indeed, if we are at the end of the video, we can't get an other frame...
		* @param numFrame if you want to get a previously loaded frame, you can use this param. If below 0, get a new frame...
		* @return a FoV. If the video is finished, the FoV returned is not usable! Test the validity with the bool FieldOfView::isRealFoV() method.
		*/
		FieldOfView getFrame(int numFrame=-1);
	};

}

#endif