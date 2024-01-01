#include <opencv2/opencv.hpp>

//#include "defs.h"

#include "utils.hpp" 
#include "MotionTrack.hpp"

using namespace std;
using namespace cv;

#define ELAD_DEBUG_MODE 0

bool  CMotionTracker::track(cv::Mat frame_)
{

	if (m_bBox.empty())
		return false;

	cv::Mat frame;
	if (frame_.channels() > 1)
		cv::cvtColor(frame_, frame, cv::COLOR_BGR2GRAY);
	else
		frame = frame_;

	if (m_prevFrame.empty()) {
		m_prevFrame = frame;
		return false;
	}

	int pointsNum = optTrack(frame, m_bBox);

	m_prevFrame = frame;

	return pointsNum > 0;
}


bool  CMotionTracker::track(cv::Mat frame, cv::Rect roi)
{
	m_bBox = roi;
	return track(frame);
}


int CMotionTracker::optTrack(cv::Mat curFrame, cv::Rect &BBox)
{
	const int MIN_MOTION = 3; // DDEBUG CONST 
	std::vector<cv::Point2f> points[2];

	int temp_idx = 0;
	int idx, l = 0;
	vector<uchar> status;
	vector<float> err;
	bool selection = true; // DDEBUG 

	Size  winSize(m_params.winSize, m_params.winSize);
	Mat frameDisp;

	cv::Mat roiImg = m_prevFrame(BBox);

	// -1- Selection
	if (selection) {
		goodFeaturesToTrack(roiImg, points[0], MAX_FEATURES, m_params.quality_level, m_params.block_size, Mat(), 3, m_params.use_harris, m_params.harris_k);
		for (auto &p : points[0])
			p = p + cv::Point2f(BBox.tl());
	}
	else
		points[0] = m_tp_point; // Start with last tracked point 

	if (points[0].size() == 0)
		return false;

	// -2- Tracking
	calcOpticalFlowPyrLK(m_prevFrame, curFrame, points[0], points[1], status, err, winSize, 3, m_params.termcrit, 0, 0.001);


	m_tp_point.clear();

	temp_idx = 0;
	cv::Point2f motionVector(0,0);

	for (idx = 0; idx < (int)points[1].size(); idx++) {
		if (status[idx] == 0)
			continue;
		if (distance(points[1][idx], points[0][idx]) < MIN_MOTION)
			continue;
		m_tp_point.push_back(points[1][idx]);

		motionVector = motionVector + (points[0][idx] - points[1][idx]);

#if 1
		/*
		//std::vector <int> directions;
		cv::Point2d dir_1; //DIRECTIONS OF THE MOVEMENT OF THE TRACKED OBJECTS
		dir_1.x = distance(points[1][idx], points[0][idx]); //SIZE OF THE MOVEMENT
		dir_1.y = std::atan2((double)(points[1][idx].y - points[0][idx].y), (double)(points[1][idx].x - points[0][idx].x)); //DIRECTION OF THE MOVEMENT
		//directions.push_back(dir_1.x);
		*/



#endif 
		temp_idx++;
	}

	motionVector = motionVector/temp_idx;

	return temp_idx;
}



cv::Rect CMotionTracker::track(cv::Mat frame2_, cv::Mat frame1_, cv::Rect BBox)
{
	cv::Mat frame1, frame2;
	const int MIN_MOTION = 1; // DDEBUG CONST 
	const int MIN_GOOD_POINTS = 4;  // DDEBUG CONST 
	std::vector<cv::Point2f> points[2];

	int temp_idx = 0;
	int idx, l = 0;
	vector<uchar> status;
	vector<float> err;

	Size  winSize(m_params.winSize, m_params.winSize);
	Mat frameDisp;

	if (frame1_.channels() > 1) {
		cv::cvtColor(frame1_, frame1, cv::COLOR_BGR2GRAY);
		cv::cvtColor(frame2_, frame2, cv::COLOR_BGR2GRAY);
	}
	else {
		frame1 = frame1_;
		frame2 = frame2_;
	}

	cv::Mat roiImg = frame1(BBox);

	// -1- Selection	
	goodFeaturesToTrack(roiImg, points[0], MAX_FEATURES, m_params.quality_level, m_params.block_size, Mat(), 3, m_params.use_harris, m_params.harris_k);
	for (auto &p : points[0])
		p = p + cv::Point2f(BBox.tl());

	if (points[0].size() == 0)
		return cv::Rect();

	// -2- Tracking
	calcOpticalFlowPyrLK(frame1, frame2, points[0], points[1], status, err, winSize, 3, m_params.termcrit, 0, 0.001);

	m_tp_point.clear();

	int trackedPoints = 0;
	//cv::Point2f motionVector(0, 0);

	if (1) // DDEBUG 
	{
		for (auto p : points[0])
			cv::circle(frame1, p, 5, 0, -1);
		for (auto p : points[1])
			cv::circle(frame2, p, 5, 0, -1);
	}


	for (idx = 0; idx < (int)points[1].size(); idx++) {
		if (status[idx] == 0)
			continue;
		//if (distanceSQR(points[1][idx], points[0][idx]) < (double)MIN_MOTION*MIN_MOTION)
		if (fabs(points[1][idx].x - points[0][idx].x) < 0.5 && fabs(points[1][idx].y - points[0][idx].y) < 0.5) // zero motion  
			continue;
		m_tp_point.push_back(points[1][idx]);

		//motionVector = motionVector + (points[0][idx] - points[1][idx]);

#if 0
		//std::vector <int> directions;
		cv::Point2d dir_1; //DIRECTIONS OF THE MOVEMENT OF THE TRACKED OBJECTS
		dir_1.x = distance(points[1][idx], points[0][idx]); //SIZE OF THE MOVEMENT
		dir_1.y = std::atan2((double)(points[1][idx].y - points[0][idx].y), (double)(points[1][idx].x - points[0][idx].x)); //DIRECTION OF THE MOVEMENT
		//directions.push_back(dir_1.x);
#endif 
		trackedPoints++;
	}

	if (trackedPoints < MIN_GOOD_POINTS)
		return cv::Rect();

	//motionVector = motionVector / trackedPoints;

	m_bBox = boundingRect(m_tp_point);

	return m_bBox;
}




// =========================================================================================================================
// ================================================= HELPER FUNCTIONS ======================================================
// =========================================================================================================================


void CMotionTracker::findCentroidBB(float *tp_point_x, float *tp_point_y, int len, Point2d &centroid, Rect ROI_Track)
{
	float dist_x = 0;
	cv::Point2f point1;
	double dist;
	float avg_dist_x = 0, avg_dist_y = 0;
	int dist_counter = 0;
	for (int idx = 0; idx < len; idx++) {
		point1.x = tp_point_x[idx] + ROI_Track.x;
		point1.y = tp_point_y[idx] + ROI_Track.y;
		dist = distance(cv::Point2d(point1.x, point1.y), centroid);
		//dist_x = fabs(tp_point_x[idx] - centroid.x);
		if (dist < 40)
		{
			avg_dist_x += point1.x;
			avg_dist_y += point1.y;
			dist_counter++;
		}
	}
	if (dist_counter > 0) {
		centroid.x = (int)avg_dist_x / dist_counter;
		centroid.y = (int)avg_dist_y / dist_counter;
	}
}

void CMotionTracker::findCentroid(float *tp_point_x, float *tp_point_y, int len, Point2d &centroid)
{
	float dist_x = 0;
	cv::Point2f point1;
	double dist;
	float avg_dist_x = 0, avg_dist_y = 0;
	int dist_counter = 0;
	for (int idx = 0; idx < len; idx++) {
		point1.x = tp_point_x[idx];
		point1.y = tp_point_y[idx];
		//dist = distance(point1, centroid);
		dist = 200;
		//dist_x = fabs(tp_point_x[idx] - centroid.x);
		if (dist < 500)
		{
			avg_dist_x += point1.x;
			avg_dist_y += point1.y;
			dist_counter++;
		}
	}
	if (dist_counter > 0) {
		centroid.x = (int)avg_dist_x / dist_counter;
		centroid.y = (int)avg_dist_y / dist_counter;
	}
}


void CMotionTracker::init(double quality_level, double min_distance ,int block_size ,bool use_harris ,double harris_k ,cv::TermCriteria termcrit ,int winSize )
{
	m_params.quality_level = quality_level;
	m_params.min_distance = min_distance;
	m_params.block_size = block_size;
	m_params.use_harris = use_harris;
	m_params.harris_k = harris_k;
	m_params.termcrit = termcrit;
	m_params.winSize = winSize;

}