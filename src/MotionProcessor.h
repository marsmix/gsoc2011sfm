#ifndef _GSOC_SFM_MOTION_PROCESSOR_H
#define _GSOC_SFM_MOTION_PROCESSOR_H 1

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "dirent.h"
#include <string>
#include <sstream>
#include <vector>


namespace OpencvSfM{
  enum TypeOfMotionProcessor
  {
    IS_SINGLE_FILE,///<Mask used to know if the input is a single file
    IS_LIST_FILES,///<Mask used to know if the input is a list of files
    IS_WEBCAM,///<Mask used to know if the input is a webcam
    IS_DIRECTORY,///<Mask used to know if the input is an entire directory
    IS_VIDEO///<Mask used to know if the input is a video file
  };

  /*! \brief This class try to create a commun interface for files loading.
  *   Indeed, if you want to use webcam, avi file of list of files, you will have to do some annoying processing, like iterate the different files of the directory.
  *   With MotionProcessor, you can now use a folder of image the same way you use a webcam or a video file.
  *
  * The class is still in development as the way to open folder is not really clear...
  * The easy way would be to use "dirent.h" header, but the easy thing is not always the best thing...
  */
class MotionProcessor
{
protected:
  TypeOfMotionProcessor type_of_input_;///<This attribut is used to know which type is the input (webcam, video file, list of file or just one image)
  cv::VideoCapture capture_;///<When the camera is attached to an avi file or webcam, this will be usefull to get frame...
  std::vector<std::string> nameOfFiles_;///<If the motion processor use directory as input, we store here the names of files.
  /**
  * When the camera is attached to a list of file, sourceName_ will be used to store name of the prefix.
  * For example, if the files are img1.jpg, img2.jpg, ... img125.jpg, sourceName_ will be equal to img
  */
  std::string sourceName_;
  /**
  * When the camera is attached to a list of file, suffix_ will be used to store name of the suffix.
  * For example, if the files are img1.jpg, img2.jpg, ... img125.jpg, suffix_ will be equal to .jpg
  */
  std::string suffix_;

  unsigned int numFrame_;///<When the camera is attached to a list of file, numFrame_ will be used to know how many frames we have take.
  int wantedWidth_;///<if below 0, represent the wanted width of Mat returned by getFrame();
  int wantedHeight_;///<if below 0, represent the wanted height of Mat returned by getFrame();
  bool convertToRGB_;///<Boolean flags indicating whether images should be converted to RGB
public:
  MotionProcessor(void);
  ~MotionProcessor(void);

  /**
  * You can attach this camera to a webcam
  * use this method to set it as the input source!
  * @param idWebCam id of the webcam
  * @return true if input source opened without problems
  */
  bool setInputSource(int idWebCam);
  /**
  * You can attach this camera to a video file or a single picture.
  * use this method to set it as the input source!
  * @param nameOfFile name of the media file (picture or avi movie)
  * @param type of input (can be either IS_DIRECTORY, IS_VIDEO or IS_SINGLE_FILE)
  * @return true if input source opened without problems
  */
  bool setInputSource(std::string nameOfFile,TypeOfMotionProcessor inputType=IS_SINGLE_FILE);
  /**
  * You can attach this camera to a list of file.
  * use this method to set the input source!
  * For example, if the files are img1.jpg, img2.jpg, ... img125.jpg, prefix will be equal to "img", suffix to ".jpg" and startNumber equal to 1
  * @param prefix the part of the files names which stay the same (img)
  * @param suffix the type of the files (.jpg for instance)
  * @param startNumber the first number to use...
  * @return true if input source opened without problems
  */
  bool setInputSource(std::string prefix,std::string suffix,int startNumber=0);
  /**
  * use this method if you want to get a field of view from this camera.
  * Be carreful : this function can return a fake FieldOfView! Indeed, if we are at the end of the video, we can't get an other frame...
  * @param numFrame if you want to get a previously loaded frame, you can use this param. If below 0, get a new frame...
  * @return a FoV. If the video is finished, the FoV returned is not usable! Test the validity with the bool FieldOfView::isRealFoV() method.
  */
  cv::Mat getFrame();
  /**
  * use this method to change the properties of pictures retrived by this MotionProcessor.
  * the properties are the same than VideoCapture (see http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html#cv-videocapture-get)
  * @param idProp Property identifier
  * @param value new value of the property
  */
  bool setProperty(int idProp,double value);
  /**
  * use this method to get actual properties of pictures retrived by this MotionProcessor.
  * the properties are the same than VideoCapture (see http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html#cv-videocapture-get)
  * @param idProp Property identifier
  * @return the value of the property
  */
  double getProperty(int idProp);
};
}

#endif