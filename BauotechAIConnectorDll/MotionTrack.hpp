#pragma once

#define MAX_FEATURES 400
#define MIN_MOTION_DISTANCE 1.0

class CParams {
public:
	double quality_level = 0.01;
	double min_distance = 16.0;
	int block_size = 10;
	bool use_harris = false;
	double harris_k = 0.04;
	cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
	int winSize = 7;

};

class CMotionTracker {
public:
	void init(int w, int h) { WIDTH = w; HEIGHT = h; }
	void init(double quality_level, double min_distance, int block_size, bool use_harris, double harris_k, cv::TermCriteria termcrit, int winSize);
	bool  track(cv::Mat frame);
	bool track(cv::Mat frame, cv::Rect roi);
	bool track(cv::Rect roi);
	cv::Rect track(cv::Mat frame2, cv::Mat frame1, cv::Rect roi1); // basic tracking 
	int  optTrack(cv::Mat curFrame, cv::Rect &BBox);
	int  optTrack_OLD(cv::Mat prevFrame, cv::Mat curFrame, cv::Point2d &centroid, cv::Rect &BBox, std::vector<cv::Point2d> &directions);
	void findCentroidBB(float *tp_point_x, float *tp_point_y, int len, cv::Point2d &centroid, cv::Rect ROI_Track);
	void findCentroid(float *tp_point_x, float *tp_point_y, int len, cv::Point2d &centroid);
	cv::Rect getBBox() { return m_bBox;  }

private:
	int WIDTH, HEIGHT;

	std::vector <cv::Point2f> m_tp_point;
	//float m_tp_point_x[MAX_FEATURES];
	//float m_tp_point_y[MAX_FEATURES];

	CParams m_params;
	cv::Mat m_prevFrame;
	cv::Rect m_bBox;


};
