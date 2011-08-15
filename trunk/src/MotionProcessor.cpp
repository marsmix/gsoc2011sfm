
#include "macro.h" //SFM_EXPORTS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef HAVE_BOOST
#include <boost/filesystem.hpp>   // includes all needed Boost.Filesystem declarations
#endif
#include <sstream>

#include "MotionProcessor.h"

using cv::Mat;
using cv::Size;
using cv::Vec;
using cv::Ptr;
using cv::VideoCapture;
using std::string;
using cv::imread;
using std::ostringstream;
using std::vector;
#ifdef HAVE_BOOST
using boost::filesystem::path;
#endif

namespace cv{
  CVAPI( int ) cvHaveImageReader( const char* filename );
}

namespace OpencvSfM{

  MotionProcessor::MotionProcessor( void )
  {
    numFrame_=0;
    pos_in_loading_process_ = 0;
    wantedWidth_=-1;
    wantedHeight_=-1;
    convertToRGB_=-1;
  }

  MotionProcessor::~MotionProcessor( void )
  {
    switch( type_of_input_ )
    {
    case IS_WEBCAM:
    case IS_VIDEO:
      capture_.release( );
      break;
    case IS_DIRECTORY:
    case IS_LIST_FILES:
    case IS_SINGLE_FILE:
      nameOfFiles_.clear( );
      break;
    }
  }

  bool MotionProcessor::setInputSource( int idWebCam )
  {
    if( this->capture_.open( idWebCam ))
    {
      this->type_of_input_=IS_WEBCAM;
      //if we have some properties, set them to the webcam:
      this->capture_.set( CV_CAP_PROP_CONVERT_RGB,this->convertToRGB_>0 );
      if( wantedHeight_>0 )
        this->capture_.set( CV_CAP_PROP_FRAME_HEIGHT,this->wantedHeight_ );
      if( wantedWidth_>0 )
        this->capture_.set( CV_CAP_PROP_FRAME_WIDTH,this->wantedWidth_ );
      return true;
    };
    return false;
  };

  bool MotionProcessor::setInputSource( vector<string> list_images )
  {
    this->type_of_input_=IS_LIST_FILES;
    nameOfFiles_ = list_images;
    suffix_ = "Not a dynamic list";
    return true;
  };

  bool MotionProcessor::setInputSource( string nameOfFile,TypeOfMotionProcessor inputType/*=IS_VIDEO*/ )
  {//IS_DIRECTORY, IS_VIDEO or IS_SINGLE_FILE
    this->sourceName_=nameOfFile;
    this->type_of_input_=inputType;

    if( inputType == IS_VIDEO )
      return this->capture_.open( nameOfFile );

    if( inputType == IS_DIRECTORY )
    {
#ifdef HAVE_BOOST
      path dirTmp( nameOfFile.c_str( ) );
      if ( !boost::filesystem::exists( dirTmp ) || !boost::filesystem::is_directory( dirTmp ) ) {
        return false;
      }

      boost::filesystem::directory_iterator iter= boost::filesystem::directory_iterator( dirTmp );
      while( iter != boost::filesystem::directory_iterator( ) )
      {
        string name_of_file = iter->path( ).string( );
        if( cv::cvHaveImageReader( (const char* )name_of_file.c_str( ) ) )
          nameOfFiles_.push_back( name_of_file );
        iter++;
      }
      //if we don't have loaded files, return an error!
      if( nameOfFiles_.empty( ) )
        return false;

      // sort, since directory iteration
      // is not ordered on some file systems
      std::sort( nameOfFiles_.begin( ), nameOfFiles_.end( ) );
#else
      return false;
#endif
    }
    return true;
  };

  bool MotionProcessor::setInputSource( string prefix,string suffix,int startNumber/*=0*/ )
  {
    this->sourceName_ = prefix;
    this->suffix_ = suffix;
    pos_in_loading_process_ = startNumber;
    this->type_of_input_ = IS_LIST_FILES;
    return true;//always true because the error will occur when user will try to get the frame, not now...
  };

  Mat MotionProcessor::getFrame( )
  {
    Mat imgTmp;

    //Is the current cursor in the middle of the video?
    if( type_of_input_==IS_LIST_FILES && numFrame_<nameOfFiles_.size( ) )
    {
      //Someone as changed the position of cursor...
      //Reload the wanted file:
      imgTmp=imread( nameOfFiles_[ numFrame_ ],convertToRGB_ );
      this->numFrame_++;//and move to the next frame
    }
    else
    {
      switch( type_of_input_ )
      {
      case IS_WEBCAM:
      case IS_VIDEO:
        {
          //Get the current image from file:
          bool aNewFrameIsAvailable=capture_.grab( );
          if( aNewFrameIsAvailable )
          {
            capture_.retrieve( imgTmp );
          }
        }
        break;
      case IS_LIST_FILES:
        {
          ostringstream oss;
          oss<<this->sourceName_<<this->numFrame_<<this->suffix_;

          imgTmp=imread( oss.str( ).c_str( ),convertToRGB_ );
          this->numFrame_++;

          unsigned int stop=this->numFrame_+10;
          while( imgTmp.empty( )&&stop>this->numFrame_ ){
            oss.str( "" );
            oss<<this->sourceName_<<this->numFrame_<<this->suffix_;
            imgTmp=imread( oss.str( ).c_str( ),convertToRGB_ );
            this->numFrame_++;
          }
          if( ! imgTmp.empty( ) )
            nameOfFiles_.push_back( oss.str( ) );//add the new file
        }
        break;
      case IS_DIRECTORY:
        {
          if( numFrame_<nameOfFiles_.size( ) )
          {
            imgTmp=imread( nameOfFiles_[ numFrame_ ],convertToRGB_ );
            this->numFrame_++;//and move to the next frame
          }
        }
        break;
      case IS_SINGLE_FILE:
        {
          imgTmp=imread( this->sourceName_.c_str( ),convertToRGB_ );
        }
        break;
      }
    }

    if( !imgTmp.empty( ) )
    {
      //now we ensure the file as the good properties:
      //First the colors:
      if( (( (convertToRGB_>0) && imgTmp.channels( )<=1 ) ||
        ( (convertToRGB_==0) && imgTmp.channels( )!=1 )) )
      {
        Mat correctImg;
        if( convertToRGB_==0 )
          cvtColor( imgTmp,correctImg,CV_RGB2GRAY );
        else
          cvtColor( imgTmp,correctImg,CV_GRAY2RGB );
        imgTmp=correctImg;
      }
      //then the width and height:
      if( (( wantedHeight_>0 )&&( wantedHeight_!=imgTmp.rows ))||
        ( (wantedWidth_>0 )&&( wantedWidth_!=imgTmp.cols )) )
      {
        Mat correctImg;
        int widthTmp=wantedWidth_;
        if( wantedWidth_<=0 )
          widthTmp=imgTmp.cols;
        int heightTmp=wantedHeight_;
        if( wantedHeight_<=0 )
          heightTmp=imgTmp.cols;
        cv::resize( imgTmp,correctImg,Size( widthTmp,heightTmp ));
        imgTmp=correctImg;
      }
    }

    return imgTmp;
  }

  bool MotionProcessor::setProperty( int _idProp,double _value )
  {
    bool videoCaptureLikeThisProperty=true;
    if( type_of_input_==IS_WEBCAM||type_of_input_==IS_VIDEO )
      videoCaptureLikeThisProperty = capture_.set( _idProp,_value );
    if( ! videoCaptureLikeThisProperty )
      return false;//as the VideoCapture object don't want this property, don't do anything else.

    //First store the property using attributs:
    switch ( _idProp )
    {
    case CV_CAP_PROP_CONVERT_RGB://Boolean flags indicating whether images should be converted to RGB
      convertToRGB_ = ( int )_value;
      break;
    case CV_CAP_PROP_FRAME_HEIGHT://Height of the frames in the video stream
      wantedHeight_= ( int )_value;
      break;
    case CV_CAP_PROP_FRAME_WIDTH://Width of the frames in the video stream
      wantedWidth_= ( int )_value;
      break;
    case CV_CAP_PROP_POS_FRAMES:// 0-based index of the frame to be decoded/captured next
      {
        if( type_of_input_==IS_LIST_FILES||type_of_input_==IS_SINGLE_FILE )
        {
          if( _value>=0&&_value<=nameOfFiles_.size( ) )
            numFrame_=( unsigned int )_value;
          else
            CV_Error( CV_StsBadArg, "CV_CAP_PROP_POS_FRAMES: _value out of range!" );
        }
      };
      break;
    case CV_CAP_PROP_FORMAT://The format of the Mat objects returned by retrieve( )
    case CV_CAP_PROP_POS_MSEC://Film current position in milliseconds or video capture timestamp
    case CV_CAP_PROP_FPS://Frame rate
    case CV_CAP_PROP_FOURCC://4-character code of codec
    case CV_CAP_PROP_MODE:// A backend-specific value indicating the current capture mode
    case CV_CAP_PROP_BRIGHTNESS:// Brightness of the image ( only for cameras )
    case CV_CAP_PROP_CONTRAST:// Contrast of the image ( only for cameras )
    case CV_CAP_PROP_SATURATION:// Saturation of the image ( only for cameras )
    case CV_CAP_PROP_HUE:// Hue of the image ( only for cameras )
    case CV_CAP_PROP_GAIN:// Gain of the image ( only for cameras )
    case CV_CAP_PROP_EXPOSURE:// Exposure ( only for cameras )
    case CV_CAP_PROP_FRAME_COUNT://Number of frames in the video file
      //case CV_CAP_PROP_WHITE_BALANCE://Currently unsupported
    case CV_CAP_PROP_RECTIFICATION://TOWRITE ( note: only supported by DC1394 v 2.x backend currently )
      {
        if( type_of_input_==IS_LIST_FILES||type_of_input_==IS_SINGLE_FILE||type_of_input_==IS_DIRECTORY )
          return false;
        //else, do nothing as the property is already assigned
      }
      break;
    case CV_CAP_PROP_POS_AVI_RATIO:
      //Relative position of the video file ( 0 - start of the film, 1 - end of the film )
      {
        if( (type_of_input_==IS_LIST_FILES && suffix_ != "Not a dynamic list" )||
          type_of_input_== IS_SINGLE_FILE )
          return false;
        else
        {
          if( type_of_input_==IS_DIRECTORY )
          {
            numFrame_=( unsigned int ) ( _value*nameOfFiles_.size( ) );
          }
        }
      }
      break;
    default:
      return false;
      break;
    }

    //if the motion stream come from capture_, update the structure too:
    return true;
  };

  double MotionProcessor::getProperty( int idProp )
  {
    double valOut=0;
    switch( type_of_input_ )
    {
    case IS_WEBCAM:
    case IS_VIDEO:
      valOut = capture_.get( idProp );
      break;
    case IS_LIST_FILES:
    case IS_SINGLE_FILE:
      {
        switch ( idProp )
        {
        case CV_CAP_PROP_CONVERT_RGB://Boolean flags indicating whether images should be converted to RGB
          {
            valOut=convertToRGB_;
          };
          break;
        case CV_CAP_PROP_FRAME_HEIGHT://Height of the frames in the video stream
          {
            valOut=wantedHeight_;
          };
          break;
        case CV_CAP_PROP_FRAME_WIDTH://Width of the frames in the video stream
          {
            valOut=wantedWidth_;
          };
          break;
        case CV_CAP_PROP_POS_FRAMES:// 0-based index of the frame to be decoded/captured next
          {
            valOut=numFrame_;
          };
          break;
        case CV_CAP_PROP_POS_AVI_RATIO://Relative position of the video file ( 0 - start of the film, 1 - end of the film )
          {
            if( type_of_input_==IS_LIST_FILES||type_of_input_==IS_SINGLE_FILE )
              CV_Error( CV_StsBadArg, "MotionProcessor unknow property !" );
            else
            {
              if( type_of_input_==IS_DIRECTORY )
              {
                valOut=( (double )numFrame_/nameOfFiles_.size( ) );
              }
            }
          }
          break;
        case CV_CAP_PROP_FRAME_COUNT://Number of frames in the video file
          {
            if( type_of_input_==IS_LIST_FILES||type_of_input_==IS_SINGLE_FILE )
              CV_Error( CV_StsBadArg, "MotionProcessor unknow property !" );
            else
            {
              if( type_of_input_==IS_DIRECTORY )
              {
                numFrame_=( unsigned int )idProp;
              }
            }
          }
          break;
        case CV_CAP_PROP_FORMAT://The format of the Mat objects returned by retrieve( )
        case CV_CAP_PROP_POS_MSEC://Film current position in milliseconds or video capture timestamp
        case CV_CAP_PROP_FPS://Frame rate
        case CV_CAP_PROP_FOURCC://4-character code of codec
        case CV_CAP_PROP_MODE:// A backend-specific value indicating the current capture mode
        case CV_CAP_PROP_BRIGHTNESS:// Brightness of the image ( only for cameras )
        case CV_CAP_PROP_CONTRAST:// Contrast of the image ( only for cameras )
        case CV_CAP_PROP_SATURATION:// Saturation of the image ( only for cameras )
        case CV_CAP_PROP_HUE:// Hue of the image ( only for cameras )
        case CV_CAP_PROP_GAIN:// Gain of the image ( only for cameras )
        case CV_CAP_PROP_EXPOSURE:// Exposure ( only for cameras )
          //case CV_CAP_PROP_WHITE_BALANCE://Currently unsupported
        case CV_CAP_PROP_RECTIFICATION://TOWRITE ( note: only supported by DC1394 v 2.x backend currently )
        default:
          CV_Error( CV_StsBadArg, "MotionProcessor unknow property !" );
          break;
        }
      }
      break;
    }

    return valOut;
  }

}