#include "Camera.h"

namespace OpencvSfM{

	Camera::Camera(Mat intra_params/*=Mat::eye(3, 3, CV_64F)*/,Vec<double, 6> radial_dist/*=Vec(0.0)*/,Vec<double, 2> tangential_dist/*=Vec(0.0,0.0)*/)
	{
		CV_Assert( intra_params.rows==3 && intra_params.cols==3 );

		this->intra_params_=intra_params;
		this->radial_dist_=radial_dist;
		this->tangential_dist_=tangential_dist;
		this->config_=0;
		numFrame_=0;
	}


	Camera::~Camera(void)
	{
	}

	bool Camera::setInputSource(int idWebCam)
	{
		if(this->capture_.open(idWebCam))
		{
			this->type_of_input_=MASK_IS_WEBCAM;
			return true;
		};
		return false;
	};

	bool Camera::setInputSource(string nameOfFile)
	{
		this->sourceName_=nameOfFile;
		//first try to open the file as a video:
		if(this->capture_.open(nameOfFile))
		{
			this->type_of_input_=MASK_IS_AVI;
			return true;
		}else{
			//two options: it's not a video file OR it's a video which can't be opened...
			//It's not our problem for now: when user will try to get the frame later, an error will occur then, and user will understand how to solve it.
			//so we set the type of input to single image:
			this->type_of_input_=MASK_IS_FILE;
		}
		return true;
	};

	bool Camera::setInputSource(string prefix,string suffix,int startNumber/*=0*/)
	{
		this->sourceName_=prefix;
		this->suffix_=suffix;
		this->numFrame_=startNumber;
		this->type_of_input_=MASK_IS_LIST_FILES;
		return true;//always true because the error will occur when user will try to get the frame, not now...
	};

	FieldOfView Camera::getFrame(int numFrame/*=-1*/)
	{
		CV_Assert( numFrame<pointsOfView_.size() );

		if(numFrame<0)
		{
			switch(type_of_input_)
			{
			case MASK_IS_WEBCAM:
			case MASK_IS_AVI:
				{
					//Get the current image from file:
					bool aNewFrameIsAvailable=capture_->grab();
					if(aNewFrameIsAvailable)
					{
						Mat imgTmp;
						imgTmp;
						//Now add to our FoV vector a new FoV:
						pointsOfView_.push_back(FieldOfView(this,imgTmp));
					}else{
						//problem with stream... Probably the end of the movie!
						return FieldOfView();//return a fake FoV !
					}
					break;
				}
			case MASK_IS_LIST_FILES:
				{
					ostringstream oss;
					oss<<this->sourceName_<<this->numFrame_<<this->suffix_;

					Mat imgTmp=imread(oss.str().c_str());

					int stop=this->numFrame_+10;
					while(imgTmp.cols==0&&stop>this->numFrame_){
						this->numFrame_++;
						oss.str("");
						oss<<this->sourceName_<<this->numFrame_<<this->suffix_;
						imgTmp=imread(oss.str().c_str());
					}
					if(imgTmp.cols==0)
					{
						//problem with image, are we at the end of list ?
						return FieldOfView();//return a fake FoV !
					}else{
						pointsOfView_.push_back(FieldOfView(this,imgTmp));
					}
				}
				break;
			case MASK_IS_FILE:
				{
					Mat imgTmp=imread(this->sourceName_.c_str());
					if(imgTmp.cols==0)
					{
						//problem with image
						return FieldOfView();//return a fake FoV !
					}else{
						pointsOfView_.push_back(FieldOfView(this,imgTmp));
					}
				}
				break;
			default:
				return FieldOfView();//return a fake FoV !
			}
			this->numFrame_++;
		}else{//We don't want a new image, use numFrame to return the wanted Field of view :
			return pointsOfView_[numFrame];
		}

		//if we are here, we just have to return the last FoV :
		return pointsOfView_[pointsOfView_.size()-1];
	}
}