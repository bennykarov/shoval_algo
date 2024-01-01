#pragma once


#define MAN_MOG_THREASHOLD  20.
#define PIPE_MOG_THREASHOLD  10.

class CMan_detector {
public:
	CMan_detector() :
		m_debugLevel(0),
		m_mog_scale(1.)
	{
	}

	void init( double mog_scale, cv::Size imgDim, int debug_level, int mogHistory=400, double mogThreshold = 46.);
	std::vector<cv::Point2f> track(cv::Mat  img, cv::Mat mask = cv::Mat());
private:

	int m_debugLevel;
	CTrackMog m_trackMan_MOG;
	double m_mog_scale;
	cv::Size m_imgDim;

};

