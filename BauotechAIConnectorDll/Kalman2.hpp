#pragma once
#include <memory>
#include <deque>
#include <opencv2/opencv.hpp>

#include "utils.hpp"


enum PREDICTION_TYPE {
	predict = 1,
	correct
};

class CKalman2
{
public:
	void init();
	void init(float x, float y, float vx, float vy, float ax, float ay);
	void update(float x, float y);
	void predict();

	cv::Point2f getCenter() { return m_lastDetection == PREDICTION_TYPE::correct ? getEstimation() : getPrediction(); }
	cv::Point2f getPrediction() { return cv::Point2f(m_prediction(0), m_prediction(1)); }
	// 	m_predictPt = cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));

	cv::Point2f getEstimation() { return cv::Point2f(m_estimated(0), m_estimated(1)); }

private:
	void set_state_vector(float x, float y, float vx, float vy, float ax, float ay);

private:
	cv::Mat_<float>  m_estimated, m_prediction;
	cv::Mat_<float> m_measurement;
	//    CV_WRAP KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F );
	cv::KalmanFilter m_KF; 
	//cv::Point2f m_predictPt;
	//cv::Point2f m_statePt;
	PREDICTION_TYPE  m_lastDetection;

	float m_smoothness = 1 / 100000.;
	float m_rapidness =  1 / 10.;
};

