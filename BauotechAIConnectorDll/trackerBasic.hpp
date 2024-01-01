#pragma once
 
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>


#define MAX_BAD_DETECTION_SEC 1 // 4

enum TRACK_TYPE {
    BOOSTING = 0,
	MIL, 		// 1
	KCF, 		// 2
	TLD, 		// 3
	MEDIANFLOW, // 4
	GOTURN,  	// 5
	CSRT,  		// 6
	TYPES_LEN  
};

////////////////   UTILS  //////////////////////////////////////
////////////////////////////////////////////////////////////////



#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>

inline cv::Ptr<cv::Tracker> createTrackerByName(const std::string& name)
{
	using namespace cv;

	cv::Ptr<cv::Tracker> tracker;

	if (name == "KCF")
		tracker = cv::TrackerKCF::create();
	else if (name == "TLD")
		tracker = legacy::upgradeTrackingAPI(legacy::TrackerTLD::create());
	else if (name == "BOOSTING")
		tracker = legacy::upgradeTrackingAPI(legacy::TrackerBoosting::create());
	else if (name == "MEDIAN_FLOW")
		tracker = legacy::upgradeTrackingAPI(legacy::TrackerMedianFlow::create());
	else if (name == "MIL")
		tracker = cv::TrackerMIL::create();
	else if (name == "GOTURN")
		tracker = cv::TrackerGOTURN::create();
	else if (name == "MOSSE")
		tracker = legacy::upgradeTrackingAPI(legacy::TrackerMOSSE::create());
	else if (name == "CSRT")
		tracker = cv::TrackerCSRT::create();
	else
		CV_Error(cv::Error::StsBadArg, "Invalid tracking algorithm name\n");

	return tracker;
}

inline cv::Ptr<cv::legacy::Tracker> createTrackerByName_legacy(const std::string& name)
{
	using namespace cv;

	cv::Ptr<cv::legacy::Tracker> tracker;

	if (name == "KCF")
		tracker = legacy::TrackerKCF::create();
	else if (name == "TLD")
		tracker = legacy::TrackerTLD::create();
	else if (name == "BOOSTING")
		tracker = legacy::TrackerBoosting::create();
	else if (name == "MEDIAN_FLOW")
		tracker = legacy::TrackerMedianFlow::create();
	else if (name == "MIL")
		tracker = legacy::TrackerMIL::create();
	else if (name == "GOTURN")
		CV_Error(cv::Error::StsNotImplemented, "FIXIT: migration on new API is required");
	else if (name == "MOSSE")
		tracker = legacy::TrackerMOSSE::create();
	else if (name == "CSRT")
		tracker = legacy::TrackerCSRT::create();
	else
		CV_Error(cv::Error::StsBadArg, "Invalid tracking algorithm name\n");

	return tracker;
}

////////////////////////////////////////////////////////////////


class CTracker {
public:
	bool init(int TrackerType, int debugLevel, int badFramesToreset = MAX_BAD_DETECTION_SEC * CONSTANTS::FPS);
	bool init(); 
	void reset()
		{ m_frameNum = 0; falseDetectionLen = 0; m_bbox = cv::Rect(); init();}
	bool  track(cv::Mat frame);
	bool  track(cv::Mat frame, cv::Rect roi);
	void setROI(const cv::Mat &img, cv::Rect bbox);
	bool isActive() { return m_frameNum > 0; }  
	bool isDetected() { return falseDetectionLen == 0;} // currently detected
	// void setDebugLevlel(int l) { m_debugLevel = l;}

	static cv::Rect setROI_GUI(cv::Mat img); // DDEBUG function

	cv::Rect getBBox() { return m_bbox; }

	int getFlaseDetectionLen() { return falseDetectionLen; }
	int track_main(std::string videoFName, int trackerTypeInd, int skip=0);

private:
	// 										0		   1	  2      3       4           5         6
    //std::string m_trackerTypes_str[7] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "CSRT"};
	std::vector  <std::string> m_trackerTypes_str = { "KCF","TLD","BOOSTING","MEDIAN_FLOW","MIL","GOTURN","MOSSE" ,"CSRT" };

    // vector <string> trackerTypes(types, std::end(types));
    int m_trackerType = -1;
    std::string trackerType;
    int m_frameNum = 0; 			// tracking processed frames
	cv::Rect m_bbox;
	int falseDetectionLen = 0; 	//count detection failure tail length

	int m_debugLevel=0;

	int m_badFramesToReset = MAX_BAD_DETECTION_SEC * CONSTANTS::FPS;

    cv::Ptr<cv::Tracker> m_tracker;


};
