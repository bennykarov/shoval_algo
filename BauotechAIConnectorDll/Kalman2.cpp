#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"


#include "utils.hpp"
#include "Kalman2.hpp"

using namespace cv;


void CKalman2::init(float x, float y, float vx, float vy, float ax, float ay)
{
	init();
	set_state_vector(x, y, vx, vy, ax, ay); // make immidiate prediction (otherwise need many correction's inputs)
}


void CKalman2::init()
{
	m_KF  = cv::KalmanFilter(6, 2, 0);
	setIdentity(m_KF.measurementMatrix);
	setIdentity(m_KF.processNoiseCov, Scalar::all(m_smoothness));
	setIdentity(m_KF.measurementNoiseCov, Scalar::all(m_rapidness));
	setIdentity(m_KF.errorCovPost, Scalar::all(.1));

	float dt = 1.;
	float dt2 = 0.5*pow(dt, 2);
	m_KF.transitionMatrix = (cv::Mat_<float>(6, 6) <<	1,   0,   dt,   0,  dt2,0, 
														0,   1,   0,   dt,  0,  dt2,
														0,   0,   1,   0,  dt,  0, 
														0,   0,   0,   1,  0,   dt, 
														0,   0,   0,   0,  1,   0, 
														0,   0,   0,   0,  0,   1 );

	m_KF.measurementMatrix.setTo(0);
	m_KF.measurementMatrix.at<float>(0, 0) = 1;  // x
	m_KF.measurementMatrix.at<float>(1, 1) = 1;  // y


	/*  dont touch statePre!
	m_KF.statePre.at<float>(0) = x;
	m_KF.statePre.at<float>(1) = y;
	m_KF.statePre.at<float>(2) = 0;
	m_KF.statePre.at<float>(3) = 0;
	m_KF.statePre.at<float>(4) = 0;
	m_KF.statePre.at<float>(5) = 0;
	*/

	// opencv doest this init , redundent...)
	m_measurement = cv::Mat_<float>(2, 1);
	m_measurement.setTo(Scalar(0));

}

/*----------------------------------------------------------------------------
	Set the initial statePre & statePost half baked 
	make immidiate prediction (otherwise need many correction's inputs)
 ----------------------------------------------------------------------------*/
void CKalman2::set_state_vector(float x, float y, float vx, float vy, float ax, float ay)
{
	m_KF.statePre.at<float>(0) = x;
	m_KF.statePost.at<float>(0) = x;
	m_KF.statePre.at<float>(1) = y;
	m_KF.statePost.at<float>(1) = y;
	m_KF.statePre.at<float>(2) = vx;
	m_KF.statePost.at<float>(2) = vx;
	m_KF.statePre.at<float>(3) = vy;
	m_KF.statePost.at<float>(3) = vy;
	m_KF.statePre.at<float>(4) = ax;
	m_KF.statePost.at<float>(4) = ax;
	m_KF.statePre.at<float>(5) = ay;
	m_KF.statePost.at<float>(5) = ay;

	//m_prediction = m_KF.predict();   // First predict, to update the internal statePre variable
	}

void CKalman2::update(float x, float y)
{
	
	m_prediction = m_KF.predict();   // First predict, to update the internal statePre variable

	//Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1));
	m_measurement(0) = x;
	m_measurement(1) = y;
	m_estimated = m_KF.correct(m_measurement);
}

void CKalman2::predict()
{
	m_prediction = m_KF.predict();
}
