#ifndef _GSOC_SFM_MOTION_PROCESSOR_H
#define _GSOC_SFM_MOTION_PROCESSOR_H 1

#include "macro.h" //SFM_EXPORTS

#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <string>


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
  class SFM_EXPORTS MotionProcessor
  {
  protected:
    TypeOfMotionProcessor type_of_input_;///<This attribut is used to know which type is the input ( webcam, video file, list of file or just one image )
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
    /**
    * When the camera is attached to a list of file, pos_in_loading_process_
    * will be used to store actual number of image ( not always the same than numFrame_ ).
    */
    unsigned int pos_in_loading_process_;

    unsigned int numFrame_;///<When the camera is attached to a list of file, numFrame_ will be used to know how many frames we have take.
    int wantedWidth_;///<if below 0, represent the wanted width of Mat returned by getFrame( );
    int wantedHeight_;///<if below 0, represent the wanted height of Mat returned by getFrame( );
    /**
    * if >0 the loaded image is forced to be a 3-channel color image
    * if =0 the loaded image is forced to be grayscale
    * if <0 the loaded image will be loaded as-is
    */
    uchar convertToRGB_;
  public:
    MotionProcessor( void );
    ~MotionProcessor( void );

    /**
    * Use this function to know if this flux is bidirectional ( i.e.
    * frames can be iterate randomly )
    * Can be used to know if the sequence is finite
    * @return true is you can access to frames randomly, false else
    */
    inline bool isBidirectional( )
    {
      return type_of_input_ != IS_WEBCAM;
    }
    /**
    * You can attach this motion handler to a webcam
    * use this method to set it as the input source!
    * @param idWebCam id of the webcam
    * @return true if input source opened without problems
    */
    bool setInputSource( int idWebCam );
    /**
    * You can attach this motion handler to a list of picture
    * use this method to set it as the input source!
    * @param list_images list of pictures' names
    * @return true if input source opened without problems
    */
    bool setInputSource( std::vector<std::string> list_images );
    /**
    * You can attach this motion handler to a video file or a single picture.
    * use this method to set it as the input source!
    * @param nameOfFile name of the media file ( picture or avi movie )
    * @param type of input ( can be either IS_DIRECTORY, IS_VIDEO or IS_SINGLE_FILE )
    * @return true if input source opened without problems
    */
    bool setInputSource( std::string nameOfFile,TypeOfMotionProcessor inputType=IS_SINGLE_FILE );
    /**
    * You can attach this motion handler to a list of file.
    * use this method to set the input source!
    * For example, if the files are img1.jpg, img2.jpg, ... img125.jpg, prefix will be equal to "img", suffix to ".jpg" and startNumber equal to 1
    * @param prefix the part of the files names which stay the same ( img )
    * @param suffix the type of the files ( .jpg for instance )
    * @param startNumber the first number to use...
    * @return true if input source opened without problems
    */
    bool setInputSource( std::string prefix,std::string suffix,int startNumber=0 );
    /**
    * use this method if you want to get a picture from this motion handler
    * @return The current frame. If the video is finished, the Mat returned is not usable! Test if the matrix is empty before using it!
    */
    cv::Mat getFrame( );
    /**
    * use this method to change the properties of pictures retrived by this MotionProcessor.
    * the properties are the same than VideoCapture ( see http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html#cv-videocapture-get )
    * @param idProp Property identifier
    * @param value new value of the property
    */
    bool setProperty( int idProp,double value );
    /**
    * use this method to get actual properties of pictures retrived by this MotionProcessor.
    * the properties are the same than VideoCapture ( see http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html#cv-videocapture-get )
    * @param idProp Property identifier
    * @return the value of the property
    */
    double getProperty( int idProp );
  };
}

#endif