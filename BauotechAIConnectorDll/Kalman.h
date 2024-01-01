#pragma once
#include <memory>
#include <deque>
#include <opencv2/opencv.hpp>

#include "utils.hpp"



enum {
	KF_ACCELERATION = 1,
	KF_VELOCITY
};

struct kfPoint {
	float x;
	float y;
	float w;
	float h;
};


class CKalman
{
public:
	CKalman();
	CKalman(float dt, int mode, float R_noise_amp, float Q_noise_amp);
	void set_kalman_state_vector();
	void set_kalman_state_vector(float xs, float ys, float ws, float hs, float vx, float vy, float vw, float vh);
	void set_kalman_state_vector(float xs, float ys, float ws, float hs, float vx, float vy, float vw, float vh, float ax, float ay, float aw, float ah);
	void set_kalman_state_vector(float xs, float ys, float ws, float hs);
	cv::Mat get_kalman_state_vector();
	//void update(kfPoint p, bool using_predict_points);
	void update(cv::Rect b, cv::Point2f center);
	void update(kfPoint p);
	void update(); // prediction
	kfPoint get_last_predict();
	cv::Rect2f getBBox();
	~CKalman();

private:
	float _R_noise_amp;
	float _Q_noise_amp;
	int _mode;
	float _dt;
	float _px = 0;
	float _py = 0;
	float _ph = 1;
	float _pw = 1;
	cv::KalmanFilter m_linearKalman;
	void build_acceleration_model();
	void build_velocity_model();
	void predict();
	void correct(float x, float y, float w, float h);
	//void correct_(float x, float y, float w, float h); // w/o predict()
};

