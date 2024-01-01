#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"


#include "utils.hpp"
#include "Kalman.h"


CKalman::CKalman()
{
	_dt = 1.;
	_mode = 1;
	_R_noise_amp = 0.01;
	_Q_noise_amp = 0.01;
	build_acceleration_model();
}

CKalman::CKalman(float dt, int mode, float R_noise_amp, float Q_noise_amp)
{
	_dt = dt;
	_mode = mode;
	_R_noise_amp = R_noise_amp;
	_Q_noise_amp = Q_noise_amp;
	if (mode == KF_ACCELERATION)
	{
		build_acceleration_model();
	}
	else
	{
		build_velocity_model();
	}
}


/*-----------------------------------------------------------------
 * Acceleation Model:
  * State   = 12
  * Measure = 4
 *-----------------------------------------------------------------*/
void CKalman::build_acceleration_model()
{
	float dt2 = 0.5f * _dt * _dt;  

	m_linearKalman.init(12, 4, 0, CV_32F);   //   dynamParams,measureParams,controlParams, type
	m_linearKalman.transitionMatrix = (cv::Mat_<float>(12, 12) <<
		1., 0, 0, 0, _dt, 0, 0, 0, dt2, 0, 0, 0,
		0, 1., 0, 0, 0, _dt, 0, 0, 0, dt2, 0, 0,
		0, 0, 1., 0, 0, 0, _dt, 0, 0, 0, dt2, 0,
		0, 0, 0, 1., 0, 0, 0, _dt, 0, 0, 0, dt2,
		0, 0, 0, 0, 1., 0, 0, 0, _dt, 0, 0, 0,
		0, 0, 0, 0, 0, 1., 0, 0, 0, _dt, 0, 0,
		0, 0, 0, 0, 0, 0, 1., 0, 0, 0, _dt, 0,
		0, 0, 0, 0, 0, 0, 0, 1., 0, 0, 0, _dt,
		0, 0, 0, 0, 0, 0, 0, 0, 1., 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1., 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1., 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.);

	//cv::setIdentity(m_linearKalman.measurementMatrix);
	
	m_linearKalman.measurementMatrix = (cv::Mat_<float>(4, 12) <<
		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);

	//cv::setIdentity(m_linearKalman.processNoiseCov, cv::Scalar::all(_Q_noise_amp));
	cv::setIdentity(m_linearKalman.processNoiseCov, _Q_noise_amp);

	//cv::setIdentity(m_linearKalman.measurementNoiseCov, cv::Scalar::all(_R_noise_amp));
	m_linearKalman.measurementNoiseCov = (cv::Mat_<float>(4, 4) <<
		_R_noise_amp, 0, 0, 0 ,
		0 , _R_noise_amp, 0, 0,
		0, 0, _R_noise_amp, 0,
		0, 0, 0, _R_noise_amp );

}

void CKalman::build_velocity_model()
{
	float n1 = pow(_dt, 4.f) / 4.f;
	float n2 = pow(_dt, 3.f) / 2.f;
	float n3 = pow(_dt, 2.f);

	m_linearKalman.init(8, 4, 0);
	m_linearKalman.transitionMatrix = (cv::Mat_<float>(8, 8) <<
		1, 0, 0, 0, _dt, 0, 0, 0,
		0, 1, 0, 0, 0, _dt, 0, 0,
		0, 0, 1, 0, 0, 0, _dt, 0,
		0, 0, 0, 1, 0, 0, 0, _dt,
		0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 1);

	//cv::setIdentity(m_linearKalman.measurementMatrix);
	m_linearKalman.measurementMatrix = (cv::Mat_<float>(8, 4) <<
		1, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0);

	m_linearKalman.processNoiseCov = (cv::Mat_<float>(8, 8) <<
		n1, 0, 0, 0, n2, 0, 0, 0,
		0, n1, 0, 0, 0, n2, 0, 0,
		0, 0, n1, 0, 0, 0, n2, 0,
		0, 0, 0, n1, 0, 0, 0, n2,
		n2, 0, 0, 0, n3, 0, 0, 0,
		0, n2, 0, 0, 0, n3, 0, 0,
		0, 0, n2, 0, 0, 0, n3, 0,
		0, 0, 0, n2, 0, 0, 0, n3);

	m_linearKalman.processNoiseCov *= _Q_noise_amp;

	m_linearKalman.measurementNoiseCov = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);

	m_linearKalman.measurementNoiseCov *= _R_noise_amp;

}

void CKalman::set_kalman_state_vector()
{
	m_linearKalman.statePre.at<float>(0) = 0;
	m_linearKalman.statePost.at<float>(0) = 0;
	m_linearKalman.statePre.at<float>(1) = 0;
	m_linearKalman.statePost.at<float>(1) = 0;
	m_linearKalman.statePre.at<float>(2) = 0;
	m_linearKalman.statePost.at<float>(2) = 0;
	m_linearKalman.statePre.at<float>(3) = 0;
	m_linearKalman.statePost.at<float>(3) = 0;
	m_linearKalman.statePre.at<float>(4) = 0;
	m_linearKalman.statePost.at<float>(4) = 0;
	m_linearKalman.statePre.at<float>(5) = 0;
	m_linearKalman.statePost.at<float>(5) = 0;
	m_linearKalman.statePre.at<float>(6) = 0;
	m_linearKalman.statePost.at<float>(6) = 0;
	m_linearKalman.statePre.at<float>(7) = 0;
	m_linearKalman.statePost.at<float>(7) = 0;

	if (_mode == 1)
	{
		m_linearKalman.statePre.at<float>(8) = 0;
		m_linearKalman.statePost.at<float>(8) = 0;
		m_linearKalman.statePre.at<float>(9) = 0;
		m_linearKalman.statePost.at<float>(9) = 0;
		m_linearKalman.statePre.at<float>(10) = 0;
		m_linearKalman.statePost.at<float>(10) = 0;
		m_linearKalman.statePre.at<float>(11) = 0;
		m_linearKalman.statePost.at<float>(11) = 0;
	}
}

void CKalman::set_kalman_state_vector(float xs, float ys, float ws, float hs)
{
	m_linearKalman.statePre.at<float>(0) = xs;
	m_linearKalman.statePost.at<float>(0) = xs;
	m_linearKalman.statePre.at<float>(1) = ys;
	m_linearKalman.statePost.at<float>(1) = ys;
	m_linearKalman.statePre.at<float>(2) = ws;
	m_linearKalman.statePost.at<float>(2) = ws;
	m_linearKalman.statePre.at<float>(3) = hs;
	m_linearKalman.statePost.at<float>(3) = hs;
	m_linearKalman.statePre.at<float>(4) = 0;
	m_linearKalman.statePost.at<float>(4) = 0;
	m_linearKalman.statePre.at<float>(5) = 0;
	m_linearKalman.statePost.at<float>(5) = 0;
	m_linearKalman.statePre.at<float>(6) = 0;
	m_linearKalman.statePost.at<float>(6) = 0;
	m_linearKalman.statePre.at<float>(7) = 0;
	m_linearKalman.statePost.at<float>(7) = 0;
	if (_mode == 1)
	{
		m_linearKalman.statePre.at<float>(8) = 0;
		m_linearKalman.statePost.at<float>(8) = 0;
		m_linearKalman.statePre.at<float>(9) = 0;
		m_linearKalman.statePost.at<float>(9) = 0;
		m_linearKalman.statePre.at<float>(10) = 0;
		m_linearKalman.statePost.at<float>(10) = 0;
		m_linearKalman.statePre.at<float>(11) = 0;
		m_linearKalman.statePost.at<float>(11) = 0;
	}
	predict();
}

void CKalman::set_kalman_state_vector(float xs, float ys, float ws, float hs, float vx, float vy, float vw, float vh)
{
	m_linearKalman.statePre.at<float>(0) = xs;
	m_linearKalman.statePost.at<float>(0) = xs;
	m_linearKalman.statePre.at<float>(1) = ys;
	m_linearKalman.statePost.at<float>(1) = ys;
	m_linearKalman.statePre.at<float>(2) = ws;
	m_linearKalman.statePost.at<float>(2) = ws;
	m_linearKalman.statePre.at<float>(3) = hs;
	m_linearKalman.statePost.at<float>(3) = hs;
	m_linearKalman.statePre.at<float>(4) = vx;
	m_linearKalman.statePost.at<float>(4) = vx;
	m_linearKalman.statePre.at<float>(5) = vy;
	m_linearKalman.statePost.at<float>(5) = vy;
	m_linearKalman.statePre.at<float>(6) = vw;
	m_linearKalman.statePost.at<float>(6) = vw;
	m_linearKalman.statePre.at<float>(7) = vh;
	m_linearKalman.statePost.at<float>(7) = vh;

	if (_mode == 1)
	{
		m_linearKalman.statePre.at<float>(8) = 0;
		m_linearKalman.statePost.at<float>(8) = 0;
		m_linearKalman.statePre.at<float>(9) = 0;
		m_linearKalman.statePost.at<float>(9) = 0;
		m_linearKalman.statePre.at<float>(10) = 0;
		m_linearKalman.statePost.at<float>(10) = 0;
		m_linearKalman.statePre.at<float>(11) = 0;
		m_linearKalman.statePost.at<float>(11) = 0;
	}
	predict();
}

void CKalman::set_kalman_state_vector(float xs, float ys, float ws, float hs, float vx, float vy, float vw, float vh, float ax, float ay, float aw, float ah)
{
	m_linearKalman.statePre.at<float>(0) = xs;
	m_linearKalman.statePost.at<float>(0) = xs;
	m_linearKalman.statePre.at<float>(1) = ys;
	m_linearKalman.statePost.at<float>(1) = ys;
	m_linearKalman.statePre.at<float>(2) = ws;
	m_linearKalman.statePost.at<float>(2) = ws;
	m_linearKalman.statePre.at<float>(3) = hs;
	m_linearKalman.statePost.at<float>(3) = hs;
	m_linearKalman.statePre.at<float>(4) = vx;
	m_linearKalman.statePost.at<float>(4) = vx;
	m_linearKalman.statePre.at<float>(5) = vy;
	m_linearKalman.statePost.at<float>(5) = vy;
	m_linearKalman.statePre.at<float>(6) = vw;
	m_linearKalman.statePost.at<float>(6) = vw;
	m_linearKalman.statePre.at<float>(7) = vh;
	m_linearKalman.statePost.at<float>(7) = vh;

	if (_mode == 1)
	{
		m_linearKalman.statePre.at<float>(8) = ax;
		m_linearKalman.statePost.at<float>(8) = ax;
		m_linearKalman.statePre.at<float>(9) = ay;
		m_linearKalman.statePost.at<float>(9) = ay;
		m_linearKalman.statePre.at<float>(10) = aw;
		m_linearKalman.statePost.at<float>(10) = aw;
		m_linearKalman.statePre.at<float>(11) = ah;
		m_linearKalman.statePost.at<float>(11) = ah;
	}
	predict();
}

cv::Mat CKalman::get_kalman_state_vector()
{
	return m_linearKalman.statePre;
}

void CKalman::predict()
{
	cv::Mat prediction_point = m_linearKalman.predict();
	_px = prediction_point.at<float>(0);
	_py = prediction_point.at<float>(1);
	_pw = prediction_point.at<float>(2);
	_ph = prediction_point.at<float>(3);
}

void CKalman::correct(float x, float y, float w, float h)
{
	cv::Mat measurement(4, 1, CV_32F);
	measurement.at<float>(0) = x;
	measurement.at<float>(1) = y;
	measurement.at<float>(2) = w;
	measurement.at<float>(3) = h;
	m_linearKalman.correct(measurement);
	//predict();
}

#if 0
void CKalman::correct_(float x, float y, float w, float h)
{
	cv::Mat measurement(4, 1, CV_32F);
	measurement.at<float>(0) = x;
	measurement.at<float>(1) = y;
	measurement.at<float>(2) = w;
	measurement.at<float>(3) = h;
	m_linearKalman.correct(measurement);
}

void CKalman::update(point p, bool using_predict_points)
{
	if (using_predict_points)
	{
		correct(_px, _py, _pw, _ph);
	}
	else
	{
		correct(p.x, p.y, p.w, p.h);
		predict();
	}
}
#endif 


void CKalman::update(kfPoint p)
{
	/*
	correct(p.x, p.y, p.w, p.h);
	predict();
	*/
	predict();
	correct(p.x, p.y, p.w, p.h);

}


void CKalman::update(cv::Rect b, cv::Point2f center)
{
	/*
	correct(center.x, center.y, b.width, b.height);
	predict();
	*/
	predict();
	correct(center.x, center.y, b.width, b.height);
}

// No real data to correct, use only prediction
void CKalman::update()
{
	/*
	Need the Correct() even w/o real data becouse:
	"After every prediction, you should copy the predicted state (statePre) into the corrected state (statePost). 
	This should also be done for the state covariance (errorCovPre -> errorCovPost). [=correct() do this very thing , b.k.] 
	This prevents the filter from getting stuck in a state when no corrections are executed. 
	The reason is that predict() makes use of the state values stored in statePost, 
	that do not change if no corrections are called."
	*/
	//correct(_px, _py, _pw, _ph); 
	predict();
}

kfPoint CKalman::get_last_predict()
{
	kfPoint p;
	p.x = _px;
	p.y = _py;
	p.w = _pw;
	p.h = _ph;
	return p;
}


cv::Rect2f CKalman::getBBox()
{
	kfPoint p = get_last_predict();
	return cv::Rect2f(p.x - p.w/2., p.y - p.h/2., p.w, p.h);
}

CKalman::~CKalman()
{

}
