#pragma once
#include <memory>
#include <deque>
#include <opencv2/opencv.hpp>

//#include "utils.hpp"


template <class T>
class CKalman3_
{
		cv::KalmanFilter KF;
		cv::Mat_<T> measurement, prediction, estimated;
	public:
		// smoothness, rapidness: smaller is more smooth/rapid
		// bUseAccel: set true to smooth out velocity
		//void init(float x, float y);
		void init(T smoothness = 0.1, T rapidness = 0.1, bool bUseAccel = false);
		void update(T x, T y);

		// void update(float x, float y);
		//void predict();

	cv::Point2f getCenter() { return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));  }

private:
	
	cv::Point2f m_predictPt;
	//    CV_WRAP KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F );
	//cv::KalmanFilter m_KF = cv::KalmanFilter(9, 2, 0);
	//cv::Point2f m_statePt;

};

