#pragma once 
	/*-----------------------------------------------------------------------
	Predict by prev motions
	Check that predicted rect overlapped the motion rect
 -----------------------------------------------------------------------*/
#include "CObject2.hpp"
#include "Kalman.h"

#include "Kalman2.hpp"


class CPredict {
public:
	//CPredict(int type = 1) {if (type == 10) initKF();}
	bool initKF(std::vector <cv::Rect> bboxes);
	bool initKF(CObject obj);
	bool initKF2(CObject obj);
	void resetKF2() { m_predictionFrames = 0; }
	cv::Rect2f predict(CObject obj); // Linear prediction algo's
	cv::Rect2f predictKF(CObject obj, float &confidence); // Kalman Filter 
	//cv::Point2f predictKF2(CObject obj, float &confidence); // Kalman Filter 
	cv::Point2f predictKF2(CObject obj, float &confidence, int frameNum); // Kalman Filter 
	cv::Point2f predictKF2(float &confidence); // Kalman Filter 
	cv::Rect2f predictNext(CObject obj, cv::Rect2f mogROI, DET_TYPE &type);

private:
	void _initState(cv::Mat measurement);
	bool sanityTrackCheck(kfPoint s1, kfPoint s2, kfPoint s3) { return true; } // DDEBUG fake true
private:
	// CKalman::CKalman()
	CKalman _KF = CKalman(1., KF_ACCELERATION, 0.01, 0.01); // float dt, int mode (1=acceleration) , float R_noise_amp, float Q_noise_amp  >> DDEBUG CONST 
	//CKalman _KF = CKalman(0.07, 1, 0.01, 0.01); // float dt, int mode (1=acceleration) , float R_noise_amp, float Q_noise_amp  >> DDEBUG CONST 
	CKalman2 _KF2;

	int m_predictionFrames = 0;

};