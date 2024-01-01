/*-----------------------------------------------------------------------
	Predict by prev motions
	Check that predicted rect overlapped the motion rect
 -----------------------------------------------------------------------*/
#include  <numeric>

#include <opencv2/opencv.hpp>

#include "utils.hpp"
#include "config.hpp"
#include "prediction.hpp"


cv::Rect2f CPredict::predict(CObject obj)
{
	//int skip = 2; // optimize motion step - increase motion speed
	const int PredictionDepth = CONSTANTS::FPS;
	const float MinOverLappedRatio = 0.2;
	const int BackStepsForMotion = 10;

	if (obj.m_bboxes.size() < 2)
		return obj.m_bboxes.back();

	int steps = MIN(PredictionDepth, obj.m_bboxes.size() - 1);
	int lastInd = obj.m_bboxes.size() - 1;
	cv::Rect2f predictBox;


	int predictionMethod = 3; // <<<--------  DDEBUG 

	if (predictionMethod == 1) {
		cv::Point2f motion;
		std::vector <cv::Point2f> motion2f;
		std::vector <cv::Point2f> accel;

		for (int i = 0; i < steps; i++) {
			motion = centerOf(obj.m_bboxes[lastInd - i]) - centerOf(obj.m_bboxes[lastInd - i - 1]);
			motion2f.push_back(cv::Point2f(motion.x, motion.y));
		}

		for (int j = 0; j < motion2f.size() - 1; j++)
			accel.push_back(motion2f[j + 1] - motion2f[j]);

		cv::Point2f meanMotion;
		cv::Point2f meanVel = std::accumulate(motion2f.begin(), motion2f.end(), cv::Point2f(0, 0)) / (float)motion2f.size();
		cv::Point2f meanAcc(0, 0);

		bool useAcceleration = false;
		if (useAcceleration) {
			cv::Point2f meanAcc(0, 0);
			if (!accel.empty())
				meanAcc = std::accumulate(accel.begin(), accel.end(), cv::Point2f(0, 0)) / (float)accel.size();
		}

		meanMotion = meanVel + meanAcc;


		predictBox = obj.m_bboxes[lastInd] + meanMotion;
		//if (meanMotion.x < 1. || meanMotion.y < 1.) ......
	}
	if (predictionMethod == 10) {
		int steps = MIN(PredictionDepth, obj.m_bboxes.size() - 1);
		int lastInd = obj.m_bboxes.size() - 1;

		cv::Point2f motion10steps = centerOf(obj.m_bboxes[lastInd]) - centerOf(obj.m_bboxes[lastInd - 10]);
		cv::Point2f motion1step = motion10steps / 10.;
		predictBox = obj.m_bboxes[lastInd] + motion1step;
	}
	else
	{
		// init motion vectors
		vector<double> iData, xData, yData;
		int fromIndex = obj.m_bboxes.size() - steps - 1;
		int ii = fromIndex;
		for (; ii < obj.m_bboxes.size(); ii++) {
			iData.push_back(double(ii));
			xData.push_back(centerOf(obj.m_bboxes[ii]).x);
			yData.push_back(centerOf(obj.m_bboxes[ii]).y);
		}
		double predictX, predictY;

		if (predictionMethod == 2) {			//boost::math::interpolators::cardinal_cubic_b_spline<double> spline(xData.begin(), f.end(), x0, dx);


			bool extrapolate = true;
			predictX = interpolate(iData, xData, double(ii), extrapolate);
			predictY = interpolate(iData, yData, double(ii), extrapolate);

			//predictBox = moveByCenter(obj.m_bboxes[lastInd], cv::Point((int)predictX1, (int)predictY1));

			//cv::Point2f meanMotion2 = cv::Point((int)predictX, (int)predictY) - centerOf(obj.m_bboxes[lastInd]);
		}
		else if (predictionMethod == 3) {
			std::vector<double> xx;
			xx.push_back(ii);
			std::vector<double> predictXX = interpolation2(iData, xData, xx);
			std::vector<double> predictYY = interpolation2(iData, yData, xx);

			predictX = predictXX[0];
			predictY = predictYY[0];
		}

		predictBox = moveByCenter(obj.m_bboxes[lastInd], cv::Point2f(predictX, predictY));

	}
	return predictBox;
}


cv::Rect2f CPredict::predictKF(CObject obj, float &confidence)
{
	cv::Rect2f predictBox;
	//int skip = 2; // optimize motion step - increase motion speed
	//const int PredictionDepth = CONSTANTS::FPS;
	//const float MinOverLappedRatio = 0.2;
	//const int BackStepsForMotion = 10;

	if (obj.m_bboxes.size() < CONSTANTS::StableLenForPrediction) {
		return cv::Rect2f();
		confidence = 0;
	}
	else if (m_predictionFrames == 0) 
		initKF(obj);  	// Kickoff kalman

	cv::Rect2f debugBox = _KF.getBBox();

	_KF.update();
	confidence = 100.; // fake confidence !

	cv::Rect2f pBox = _KF.getBBox();

	m_predictionFrames++;

	return pBox;

}

/*----------------------------------------------------------------------------------------
 * PREDICT MULTIPLE OPTIONS:
 * (1) On first time - send to KF2 init() ( build first estimation using last positions )
 * (2) On going with position - update KF and return estimation
 * (3) On going with out position - return predictob only 
 ----------------------------------------------------------------------------------------*/
cv::Point2f  CPredict::predictKF2(CObject obj, float &confidence, int frameNum)
{
	cv::Point2f newCenter(-1, -1);

	// Option 1: Kick off prediction:
	//--------------------------------
	if (m_predictionFrames == 0) {
		if (obj.m_bboxes.size() < CONSTANTS::StableLenForPrediction) {// Too few history 

			if (obj.m_lastDetected < frameNum - 2)   // DDEBUG : Must start only after a single detection miss (assuming  dt = 1)
				std::cout << " (obj.m_lastDetected < frameNum - 2) by " << frameNum - obj.m_lastDetected << "\n";

			confidence = 0;
			return newCenter;
		}
		std::cout << "Start prediction for obj len = " << obj.len() << "\n"; // DDEBUG PRINT 
		initKF2(obj);  	// Kickoff kalman
	}

	_KF2.predict();
	confidence = 100.; // fake confidence !

	m_predictionFrames++;

	if (obj.m_lastDetected == frameNum) { // Option 2
		cv::Point2f measuredP = centerOf(obj.m_bboxes.back());
		_KF2.update(measuredP.x, measuredP.y);
		newCenter = _KF2.getEstimation();
	}
	else 
		newCenter = _KF2.getPrediction(); // Option 1


		return newCenter;

}

#if 0
/*-----------------------------------------------------------------
	Predict at start (actually init - using prev positions) 
 -----------------------------------------------------------------*/
cv::Point2f  CPredict::predictKF2(CObject obj, float &confidence)
{
	cv::Point2f newCenter(-1,-1);

	// Kick off prediction:
	//-----------------------
	if (m_predictionFrames == 0) {
		if (obj.m_bboxes.size() < CONSTANTS::StableLenForPrediction) {
			confidence = 0;
			return centerOf(obj.m_bboxes.back());
		}
		initKF2(obj);  	// Kickoff kalman
	}

	//cv::Point2f debugP = _KF2.getCenter();

	_KF2.predict();
	confidence = 100.; // fake confidence !

	newCenter = _KF2.getCenter();

	m_predictionFrames++;

	return newCenter;

}
#endif 

/*-----------------------------------------------------------------
 Predict only (no update)
 -----------------------------------------------------------------*/
cv::Point2f  CPredict::predictKF2(float &confidence)
{
	cv::Point2f predCenter(-1, -1);


	if (m_predictionFrames == 0) {
		std::cout << "Prediction Error: prediction w\\o init \n";
		confidence = 0;
		return predCenter;
	}


	_KF2.predict();
	confidence = 100.; // fake confidence !

	predCenter = _KF2.getCenter();

	m_predictionFrames++;

	return predCenter;

}

	/*-----------------------------------------------------------------------
		Naive prediction :
		predict by prev motions
		Check that predicted rect overlapped the motion rect
	 -----------------------------------------------------------------------*/
cv::Rect2f CPredict::predictNext(CObject obj, cv::Rect2f mogROI, DET_TYPE &type)
	{
		 const int PredictionDepth = 4;
		 const float MinOverLappedRatio = 0.2;
		 if (obj.m_bboxes.size() < 2)
			 return obj.m_bboxes.back();
		 
		 int steps = MIN(PredictionDepth, obj.m_bboxes.size() - 1);
		 int lastInd = obj.m_bboxes.size() - 1;
		 
		 cv::Point2f motion;
		 std::vector <cv::Point2f> motion2f;
		 std::vector <cv::Point2f> accel;

		 for (int i = 0; i < steps; i++) {
			 motion = centerOf(obj.m_bboxes[lastInd - i]) - centerOf(obj.m_bboxes[lastInd - i - 1]);
			 motion2f.push_back(cv::Point2f(motion.x, motion.y));
			 //motion += centerOf(obj.m_bboxes[lastInd - i]) - centerOf(obj.m_bboxes[lastInd - i - 1]);
		 }

		 for (int i = 0; i < steps-1; i++)
			 accel.push_back(motion2f[i+1] - motion2f[i]);

		 cv::Point2f meanVel = std::accumulate(motion2f.begin(), motion2f.end(), cv::Point2f(0, 0)) / (float)motion2f.size();
		 cv::Point2f meanAcc(0,0);
		 if (!accel.empty())
			meanAcc = std::accumulate(accel.begin(), accel.end(), cv::Point2f(0, 0)) / (float)accel.size();

		 cv::Point2f meanMotion = meanVel + meanAcc;
		 cv::Rect2f predictBox = obj.m_bboxes[lastInd] + meanMotion;

		 // Require minimal overlapping for none-hidden type 
		 if (bboxesBounding(predictBox, mogROI) > MinOverLappedRatio) {
			 //predictBox = predictBox & mogROI; // Update size with real ROI
			 type = DET_TYPE::Prediction;
		 }
		 else
			 type = DET_TYPE::Hidden;

		return predictBox;
	}


	 // Kalman Filter prediction
	 void CPredict::_initState(cv::Mat measurement)
	 {

		 if (int(measurement.total()) == 4) {
			 _KF.set_kalman_state_vector(measurement.at<float>(0), measurement.at<float>(1),
				 measurement.at<float>(2), measurement.at<float>(3));
		 }
		 else if (int(measurement.total()) == 8) {
			 _KF.set_kalman_state_vector(measurement.at<float>(0), measurement.at<float>(1),
				 measurement.at<float>(2), measurement.at<float>(3), measurement.at<float>(4),
				 measurement.at<float>(5), measurement.at<float>(6), measurement.at<float>(7));
		 }
		 else if (int(measurement.total()) == 12) {
			 _KF.set_kalman_state_vector(measurement.at<float>(0), measurement.at<float>(1),
				 measurement.at<float>(2), measurement.at<float>(3), measurement.at<float>(4),
				 measurement.at<float>(5), measurement.at<float>(6), measurement.at<float>(7),
				 measurement.at<float>(8), measurement.at<float>(9), measurement.at<float>(10),
				 measurement.at<float>(11));
		 }
		 else
		 {
			 _KF.set_kalman_state_vector();
		 }

	 }


	 bool CPredict::initKF(std::vector <cv::Rect> bboxes)
	 {
		 kfPoint sp[CONSTANTS::maxLenForPrediction];
		 std::vector <float> xVec, yVec;
		 int skipStartFrames = 10; // Skip first frames data - not stable 

		 int len = MIN(bboxes.size(), CONSTANTS::maxLenForPrediction);// lenForPrediction or less id object len smaller 
		 len -= skipStartFrames;

		 if (len < 5) {
			 std::cout << "Error: Too few points for prediction is availbale \n";
			 return 0;
		 }

		 int startInd = bboxes.size() - len;

		 for (int i = 0; i < len; i++) {// first 3 points
			 cv::Point2f center = centerOf(bboxes[startInd + i]);
			 xVec.push_back(center.x);
			 yVec.push_back(center.y);
		 }

		 /*
		 cv::GaussianBlur(xVec, xVec, cv::Size(3, 1), 0, 0);
		 cv::GaussianBlur(yVec, yVec, cv::Size(3, 1), 0, 0);
		 */


		 for (int i = 0; i < len; i++) {// first 3 points
			//cv::Point2f center = centerOf(obj.m_bboxes[startInd+i]);
			// obj.m_centers[startInd + i] = center; // for cv::Rect none Rect2f
			 sp[i].x = xVec[i];
			 sp[i].y = yVec[i];
			 sp[i].w = bboxes[startInd + i].width;
			 sp[i].h = bboxes[startInd + i].height;
		 }


		 //if (sanityTrackCheck(si, sj, sk)) 

			 // Put first point
		 cv::Mat measurement = (cv::Mat_<float>(4 * 3, 1) <<
			 sp[0].x, sp[0].y, sp[0].w, sp[0].h,
			 sp[1].x - sp[0].x, sp[1].y - sp[0].y, sp[1].w - sp[0].w, sp[1].h - sp[0].h, // velocity 
			 //0, 0, 0, 0);
			 sp[2].x + sp[0].x - 2 * sp[1].x, sp[2].y + sp[0].y - 2 * sp[1].y, sp[2].w + sp[0].w - 2 * sp[1].w, sp[2].h + sp[0].h - 2 * sp[1].h); // Accel
		/*
		 sk.x, sk.y, sk.w, sk.h, // pos
		 sj.x - sk.x, sj.y - sk.y, sj.w - sk.w, sj.h - sk.h, // velocity
		 si.x + sk.x - 2 * sj.x, si.y + sk.y - 2 * sj.y, si.w + sk.w - 2 * sj.w, si.h + sk.h - 2 * sj.h); // Accel
		 */

		 _initState(measurement);
		 for (int i = 1; i < len; i++)
			 _KF.update(sp[i]);

		 return 1;
	 }


	 bool CPredict::initKF(CObject obj)
	 {
		 kfPoint sp[CONSTANTS::maxLenForPrediction];
		 std::vector <float> xVec, yVec;
		 int skipStartFrames = 10; // Skip first frames data - not stable 

		 int len = MIN(obj.m_bboxes.size(), CONSTANTS::maxLenForPrediction);// lenForPrediction or less id object len smaller 
		 len -= skipStartFrames;

		 if (len < 5) {
			 std::cout << "Error: Too few points for prediction is availbale \n";
			 return 0;
		 }

		 int startInd = obj.m_bboxes.size() - len;

		 for (int i = 0; i < len; i++) {// first 3 points
			 cv::Point2f center = centerOf(obj.m_bboxes[startInd + i]);
			 xVec.push_back(center.x);
			 yVec.push_back(center.y);
		 }

		 /*
		 cv::GaussianBlur(xVec, xVec, cv::Size(3, 1), 0, 0);
		 cv::GaussianBlur(yVec, yVec, cv::Size(3, 1), 0, 0);
		 */

		 for (int i = 0; i <len ; i++) {// first 3 points
			//cv::Point2f center = centerOf(obj.m_bboxes[startInd+i]);
			// obj.m_centers[startInd + i] = center; // for cv::Rect none Rect2f
			 sp[i].x = xVec[i];
			 sp[i].y = yVec[i];
			 sp[i].w = obj.m_bboxes[startInd + i].width;
			 sp[i].h = obj.m_bboxes[startInd + i].height;
		 }

		 //if (sanityTrackCheck(si, sj, sk)) 

			 // Put first point
		 cv::Mat measurement = (cv::Mat_<float>(4 * 3, 1) <<
			 sp[0].x, sp[0].y, sp[0].w, sp[0].h,
			 sp[1].x - sp[0].x, sp[1].y - sp[0].y, sp[1].w - sp[0].w, sp[1].h - sp[0].h, // velocity 
			 //0, 0, 0, 0);
		     sp[2].x + sp[0].x - 2 * sp[1].x, sp[2].y + sp[0].y - 2 * sp[1].y, sp[2].w + sp[0].w - 2 * sp[1].w, sp[2].h + sp[0].h - 2 * sp[1].h); // Accel
		/*
		 sk.x, sk.y, sk.w, sk.h, // pos
		 sj.x - sk.x, sj.y - sk.y, sj.w - sk.w, sj.h - sk.h, // velocity
		 si.x + sk.x - 2 * sj.x, si.y + sk.y - 2 * sj.y, si.w + sk.w - 2 * sj.w, si.h + sk.h - 2 * sj.h); // Accel
		 */

		 _initState(measurement);

		 for (int i = 1; i < len; i++)
			 _KF.update(sp[i]);

		 return 1;
	 }


	 // Init prediction - 
	 bool CPredict::initKF2(CObject obj)
	 {
		 int skipStartFrames = 10; // Skip first frames data - not stable 
		 kfPoint sp[CONSTANTS::maxLenForPrediction];

		 int len = MIN(obj.m_bboxes.size(), CONSTANTS::maxLenForPrediction);// lenForPrediction or less id object len smaller 
		 len -= skipStartFrames;
		 if (len % 2 == 0)
			 len--; // keep len odd for the mid point 

		 if (len < CONSTANTS::minLenForPrediction) { // DDEBUG CONST
			 std::cout << "Error: Too few points for prediction is availbale \n";
			 return 0;
		 }

		 int startInd = obj.m_bboxes.size() - len;
		 

		 for (int i = 0; i < len; i++) {// first 3 points
			 cv::Point2f center = centerOf(obj.m_bboxes[startInd + i]);
			 // obj.m_centers[startInd + i] = center; // for cv::Rect none Rect2f
			 sp[i].x = center.x;
			 sp[i].y = center.y;
		 }

		 
		 int lastP = len - 1;
		 int midP = floor(len / 2);
		 /*
		 float x = sp[0].x;
		 float y = sp[0].y;
		 float vx = sp[1].x - sp[0].x;
		 float vy = sp[1].y - sp[0].y;
		 float ax = sp[2].x + sp[0].x - 2 * sp[1].x;
		 float ay = sp[2].y + sp[0].y - 2 * sp[1].y;
		 */
		 float x = sp[0].x;
		 float y = sp[0].y;
		 float vx = (sp[midP].x - sp[0].x) / (float)midP;
		 float vy = (sp[midP].y - sp[0].y) / (float)midP;
		 float ax = 0; // sp[lastP].x + sp[0].x - 2 * sp[midP].x;
		 float ay = 0; // sp[lastP].y + sp[0].y - 2 * sp[midP].y;

		 //_KF2.init();
		 _KF2.init(x,y,vx,vy,ax,ay);
		 for (int i = 1; i < len; i++) {
			 _KF2.update(sp[i].x, sp[i].y);
			 if (1) { // DDEBUG 
				 std::cout << "prediction " << _KF2.getPrediction();
				 std::cout << "   ------    estimation " << _KF2.getEstimation() << "\n";
			 }
		 }


		 return 1;
	 }
