#include <iostream>
#include <sstream>

#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#ifdef USE_CUDA
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudalegacy.hpp>
#endif 


#include "mog.hpp"

using namespace cv;
using namespace std;
const char* params
    = "{ help h         |           | Print usage }"
      "{ input          | vtest.avi | Path to a video or a sequence of image }"
      "{ algo           | MOG2      | Background subtraction method (KNN, MOG2) }";
int CBGSubstruct::main(std::string videoName)
{

    //create Background Subtractor objects
    Ptr<BackgroundSubtractor> pBackSub;
    pBackSub = createBackgroundSubtractorMOG2(); // createBackgroundSubtractorKNN();

    VideoCapture capture(videoName );
    if (!capture.isOpened()){
        //error in opening the video input
        cerr << "Unable to open video "<< endl;
        return 0;
    }

    Mat frame, fgMask;
    while (true) {
        capture >> frame;
        if (frame.empty())
            break;
        //update the background model
        pBackSub->apply(frame, fgMask);
        //get the frame number and write it on the current frame
        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
                  cv::Scalar(255,255,255), -1);
        stringstream ss;
        ss << capture.get(CAP_PROP_POS_FRAMES);
        string frameNumberString = ss.str();
        putText(frame, frameNumberString.c_str(), cv::Point(15, 15), FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));

        if (m_debugLevle > 0) {
            imshow("Frame", frame);
            imshow("FG Mask", fgMask);
        }
    }
    return 0;
}


/*-------------------------------------------------------------------------------------------------------------------------------------------------------
 * parameters details:  https://www.programmersought.com/article/2393693386/

 * Ptr<BackgroundSubtractorMOG2> mog = createBackgroundSubtractorMOG2(100,25,false);

 ** History:
 	 The number of frames used to train the background. The default is 500 frames. If you do not manually set the learningRate,
 	 the history is used to calculate the current learningRate. The larger the history, the smaller the learningRate and the slower the background update.
 	 [ trainRate = 1 use only last image for learning b.k.]

 ** varThreshold:
 	 The variance threshold used to determine whether the current pixel is foreground or background.
 	 Generally, the default is 16. If the illumination changes obviously, such as the water surface under the sun,
 	 it is recommended to set it to 25, 36. It is not very troublesome to try it. The larger the value, the lower the sensitivity;

  ** detectShadows:
  	  Whether to detect shadows, set to true for detection, false for no detection, detection shadow will increase the program time complexity,
  	  if there is no special requirement, it is recommended to set to false;

  ** Apply() learningRate	
      The value between 0 and 1 that indicates how fast the background model is learnt. 
            Negative parameter value makes the algorithm to use some automatically chosen learning rate. 
            0   means that the background model is not updated at all, 
            1   means that the background model is completely reinitialized from the last frame. 

  	 MORE BGS algo's:
  	    cv.BackgroundSubtractorCNT();
    	cv.BackgroundSubtractorLSBP();
    	cv.BackgroundSubtractorGSOC();
 *--------------------------------------------------------------------------------------------------------------------------------------------------------*/
void CBGSubstruct::init(int History, double varThreshold, bool detectShadows, int emphasize, bool gpuExit)
{
    m_isGPU = gpuExit;


    //create Background Subtractor objects
#ifdef USE_CUDA
    if (!m_isGPU)
	    m_pBackSub = createBackgroundSubtractorMOG2( History, varThreshold, detectShadows); // createBackgroundSubtractorKNN();
    else 
        m_pBackSub = cv::cuda::createBackgroundSubtractorMOG2(History, varThreshold, detectShadows);
#else
    m_pBackSub = createBackgroundSubtractorMOG2(History, varThreshold, detectShadows); // createBackgroundSubtractorKNN();

#endif 
	m_emphasize = emphasize;

}

cv::Mat  CBGSubstruct::process(cv::Mat frame)
{

    Mat fgMask;

    if (frame.empty())
        return cv::Mat();

    m_frameNum = (m_frameNum+1) % 99999;

#ifdef USE_CUDA
        if (!m_isGPU) {
            m_pBackSub->apply(frame, fgMask, m_learningRate);
        }
    else {
        cv::cuda::GpuMat d_frame(frame);  // can also :  d_frame.upload(frame);
        cv::cuda::GpuMat d_fgMask;
        m_pBackSub->apply(d_frame, d_fgMask, m_learningRate);
        //d_frame.download(frame);
        d_fgMask.download(fgMask);
    }
#else
    m_pBackSub->apply(frame, fgMask, m_learningRate);

#endif 
	if (m_frameNum < 5) // wormup subStruction
		return cv::Mat();


	if (m_emphasize > 0)
		emphasizeMask(fgMask, m_emphasize);
    return fgMask;
}



void CBGSubstruct::emphasizeMask(cv::Mat &mask, int enlargeDepth)
{

	// Fill holes within the pupil area 
	int kernel_size = 1; // DDEBUG CONST 
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1), cv::Point(kernel_size, kernel_size));
	kernel_size = 3; // DDEBUG CONST 
	cv::Mat element3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1), cv::Point(kernel_size, kernel_size));

	// Enlarge (and stitch together)
	for (int i = 0; i < enlargeDepth; i++)
		cv::dilate(mask, mask, element3);
	// Reduce half way 
	for (int i = 0; i < enlargeDepth/2; i++)
		cv::erode(mask, mask, element3);



}
