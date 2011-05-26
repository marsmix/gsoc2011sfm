#include "MotionProcessor.h"

using cv::Mat;
using cv::Size;
using cv::Vec;
using cv::VideoCapture;
using std::string;
using cv::imread;
using std::ostringstream;

namespace OpencvSfM{

  MotionProcessor::MotionProcessor(void)
  {
    numFrame_=0;
    wantedWidth_=-1;
    wantedHeight_=-1;
    convertToRGB_=false;
  }


  MotionProcessor::~MotionProcessor(void)
  {
    switch(type_of_input_)
    {
    case IS_WEBCAM:
    case IS_VIDEO:
      capture_.release();
      break;
    case IS_LIST_FILES:
    case IS_SINGLE_FILE:
      break;//nothing to do...
    }
  }

  bool MotionProcessor::setInputSource(int idWebCam)
  {
    if(this->capture_.open(idWebCam))
    {
      this->type_of_input_=IS_WEBCAM;
      //if we have some properties, set them to the webcam:
      this->capture_.set(CV_CAP_PROP_CONVERT_RGB,this->convertToRGB_);
      if(wantedHeight_>0)
        this->capture_.set(CV_CAP_PROP_FRAME_HEIGHT,this->wantedHeight_);
      if(wantedWidth_>0)
        this->capture_.set(CV_CAP_PROP_FRAME_WIDTH,this->wantedWidth_);
      return true;
    };
    return false;
  };

  bool MotionProcessor::setInputSource(string nameOfFile,bool openAsMovie)
  {
    this->sourceName_=nameOfFile;
    //first try to open the file as a video:
    if(openAsMovie)
    {
      this->capture_.open(nameOfFile);
      this->type_of_input_=IS_VIDEO;
      return true;
    }else{
      this->type_of_input_=IS_SINGLE_FILE;
    }
    return true;
  };

  bool MotionProcessor::setInputSource(string prefix,string suffix,int startNumber/*=0*/)
  {
    this->sourceName_=prefix;
    this->suffix_=suffix;
    this->numFrame_=startNumber;
    this->type_of_input_=IS_LIST_FILES;
    return true;//always true because the error will occur when user will try to get the frame, not now...
  };

  Mat MotionProcessor::getFrame()
  {
    Mat imgTmp;

    //Is the current cursor in the middle of the video?
    if(type_of_input_==IS_LIST_FILES && numFrame_<nameOfFiles_.size())
    {
      //Someone as changed the position of cursor...
      //Reload the wanted file:
      imgTmp=imread(nameOfFiles_[numFrame_],convertToRGB_);
      this->numFrame_++;//and move to the next frame
    }
    else
    {
      switch(type_of_input_)
      {
      case IS_WEBCAM:
      case IS_VIDEO:
        {
          //Get the current image from file:
          bool aNewFrameIsAvailable=capture_.grab();
          if(aNewFrameIsAvailable)
          {
            capture_.retrieve(imgTmp);
          }
          break;
        }
      case IS_LIST_FILES:
        {
          ostringstream oss;
          oss<<this->sourceName_<<this->numFrame_<<this->suffix_;

          imgTmp=imread(oss.str().c_str(),convertToRGB_);

          unsigned int stop=this->numFrame_+10;
          while(imgTmp.empty()&&stop>this->numFrame_){
            this->numFrame_++;
            oss.str("");
            oss<<this->sourceName_<<this->numFrame_<<this->suffix_;
            imgTmp=imread(oss.str().c_str(),convertToRGB_);
          }
          nameOfFiles_.push_back(oss.str());//add the new file
          if(! imgTmp.empty())
            this->numFrame_++;
        }
        break;
      case IS_SINGLE_FILE:
        {
          imgTmp=imread(this->sourceName_.c_str(),convertToRGB_);
        }
        break;
      }
    }

    if(!imgTmp.empty())
    {
      //now we ensure the file as the good properties:
      //First the colors:
      if(((convertToRGB_&&imgTmp.channels()<=1)||(!convertToRGB_&&imgTmp.channels()!=1)))
      {
        Mat correctImg;
        if(convertToRGB_)
          cvtColor(imgTmp,correctImg,CV_RGB2GRAY);
        else
          cvtColor(imgTmp,correctImg,CV_GRAY2RGB);
        imgTmp=correctImg;
      }
      //then the width and height:
      if(((wantedHeight_>0)&&(wantedHeight_!=imgTmp.rows))||
        ((wantedWidth_>0)&&(wantedWidth_!=imgTmp.cols)))
      {
        Mat correctImg;
        int widthTmp=wantedWidth_;
        if(wantedWidth_<=0)
          widthTmp=imgTmp.cols;
        int heightTmp=wantedHeight_;
        if(wantedHeight_<=0)
          heightTmp=imgTmp.cols;
        cv::resize(imgTmp,correctImg,Size(widthTmp,heightTmp));
        imgTmp=correctImg;
      }
    }

    return imgTmp;
  }

  bool MotionProcessor::setProperty(int _idProp,double _value)
  {
    bool videoCaptureLikeThisProperty=true;
    if(type_of_input_==IS_WEBCAM||type_of_input_==IS_VIDEO)
      videoCaptureLikeThisProperty = capture_.set(_idProp,_value);
    if(! videoCaptureLikeThisProperty)
      return false;//as the VideoCapture object don't want this property, don't do anything else.

    //First store the property using attributs:
    switch (_idProp)
    {
    case CV_CAP_PROP_CONVERT_RGB://Boolean flags indicating whether images should be converted to RGB
      convertToRGB_ = (_value>0);
      break;
    case CV_CAP_PROP_FRAME_HEIGHT://Height of the frames in the video stream
      wantedHeight_=(int)_value;
      break;
    case CV_CAP_PROP_FRAME_WIDTH://Width of the frames in the video stream
      wantedWidth_=(int)_value;
      break;
    case CV_CAP_PROP_POS_FRAMES:// 0-based index of the frame to be decoded/captured next
      {
        if(type_of_input_==IS_LIST_FILES||type_of_input_==IS_SINGLE_FILE)
        {
          if(_value>=0&&_value<=nameOfFiles_.size())
            numFrame_=(unsigned int)_value;
          else
            CV_Error( CV_StsBadArg, "CV_CAP_PROP_POS_FRAMES: _value out of range!" );
        }
      };
      break;
    case CV_CAP_PROP_FORMAT://The format of the Mat objects returned by retrieve()
    case CV_CAP_PROP_POS_MSEC://Film current position in milliseconds or video capture timestamp
    case CV_CAP_PROP_POS_AVI_RATIO://Relative position of the video file (0 - start of the film, 1 - end of the film)
    case CV_CAP_PROP_FPS://Frame rate
    case CV_CAP_PROP_FOURCC://4-character code of codec
    case CV_CAP_PROP_FRAME_COUNT://Number of frames in the video file
    case CV_CAP_PROP_MODE:// A backend-specific value indicating the current capture mode
    case CV_CAP_PROP_BRIGHTNESS:// Brightness of the image (only for cameras)
    case CV_CAP_PROP_CONTRAST:// Contrast of the image (only for cameras)
    case CV_CAP_PROP_SATURATION:// Saturation of the image (only for cameras)
    case CV_CAP_PROP_HUE:// Hue of the image (only for cameras)
    case CV_CAP_PROP_GAIN:// Gain of the image (only for cameras)
    case CV_CAP_PROP_EXPOSURE:// Exposure (only for cameras)
      //case CV_CAP_PROP_WHITE_BALANCE://Currently unsupported
    case CV_CAP_PROP_RECTIFICATION://TOWRITE (note: only supported by DC1394 v 2.x backend currently)
      if(type_of_input_==IS_LIST_FILES||type_of_input_==IS_SINGLE_FILE)
        return false;
      break;
    default:
      return false;
      break;
    }

    //if the motion stream come from capture_, update the structure too:
    return true;
  };

  double MotionProcessor::getProperty(int idProp)
  {
    double valOut=0;
    switch(type_of_input_)
    {
    case IS_WEBCAM:
    case IS_VIDEO:
      valOut = capture_.get(idProp);
      break;
    case IS_LIST_FILES:
    case IS_SINGLE_FILE:
      {
        switch (idProp)
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
        case CV_CAP_PROP_FORMAT://The format of the Mat objects returned by retrieve()
        case CV_CAP_PROP_POS_MSEC://Film current position in milliseconds or video capture timestamp
        case CV_CAP_PROP_POS_AVI_RATIO://Relative position of the video file (0 - start of the film, 1 - end of the film)
        case CV_CAP_PROP_FPS://Frame rate
        case CV_CAP_PROP_FOURCC://4-character code of codec
        case CV_CAP_PROP_FRAME_COUNT://Number of frames in the video file
        case CV_CAP_PROP_MODE:// A backend-specific value indicating the current capture mode
        case CV_CAP_PROP_BRIGHTNESS:// Brightness of the image (only for cameras)
        case CV_CAP_PROP_CONTRAST:// Contrast of the image (only for cameras)
        case CV_CAP_PROP_SATURATION:// Saturation of the image (only for cameras)
        case CV_CAP_PROP_HUE:// Hue of the image (only for cameras)
        case CV_CAP_PROP_GAIN:// Gain of the image (only for cameras)
        case CV_CAP_PROP_EXPOSURE:// Exposure (only for cameras)
          //case CV_CAP_PROP_WHITE_BALANCE://Currently unsupported
        case CV_CAP_PROP_RECTIFICATION://TOWRITE (note: only supported by DC1394 v 2.x backend currently)
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