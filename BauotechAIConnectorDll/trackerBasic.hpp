#pragma once
 
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>

#include "utils.hpp"
#include "CObject.hpp"


#define MAX_TRACKERS_ALLOWED  20
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

using namespace  cv;



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
	else if (name == "SIAM_RPN") {
		std::string m_pathToModel = "c:\\data\\models";
		std::string net;// = parser.get<String>("net");
		std::string kernel_cls1;// = parser.get<String>("kernel_cls1");
		std::string kernel_r1;// = parser.get<String>("kernel_r1");
		int backend;// = parser.get<int>("backend");
		int target;// = parser.get<int>("target");


		//inputName = R"(C:\Data\office\office_multiCars.ts)";
		net = m_pathToModel + "\\dasiamrpn_model.onnx"; // R"(C:\Data\models\dasiamrpn_model.onnx)";
		kernel_cls1 = m_pathToModel + "\\dasiamrpn_kernel_cls1.onnx";
		kernel_r1 = m_pathToModel + "\\dasiamrpn_kernel_r1.onnx";
		backend = 5; //  CUDA
		target = 7;  //  6=CUDA OR 7=CUDA 16fp 

		if (!UTILS::isFileExist(net))
		{
			std::cout << "missing SIAM tracker file  " << net << "\n"; return tracker;
		}
		if (!UTILS::isFileExist(kernel_cls1))
		{
			std::cout << "missing SIAM tracker file " << kernel_cls1 << "\n"; return tracker;
		}
		if (!UTILS::isFileExist(kernel_r1))
		{
			std::cout << "missing SIAM tracker file  " << kernel_r1 << "\n"; return tracker;
		}

		TrackerDaSiamRPN::Params params;
		params.model = samples::findFile(net);
		params.kernel_cls1 = samples::findFile(kernel_cls1);
		params.kernel_r1 = samples::findFile(kernel_r1);
		params.backend = backend;
		params.target = target;
		tracker = TrackerDaSiamRPN::create(params);

	}
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

	void setROIs(std::vector <cv::Rect> bboxes, cv::Mat frame, bool clearHistory);

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
    int m_trackerType = 0;
    std::string trackerType;
    int m_frameNum = 0; 			// tracking processed frames
	cv::Rect m_bbox;
	int falseDetectionLen = 0; 	//count detection failure tail length

	int m_debugLevel=0;

	int m_badFramesToReset = MAX_BAD_DETECTION_SEC * CONSTANTS::FPS;

    Ptr<cv::Tracker> m_tracker;


};


class CMTracker {
public:
	bool init(int TrackerType, int debugLevel, int badFramesToreset = MAX_BAD_DETECTION_SEC * CONSTANTS::FPS);
	bool init();
	void reset()
	{
		m_frameNum = 0; falseDetectionLen = 0; m_bboxs.clear(); init();
	}
	int len() { return m_algorithms.size(); }

	std::vector <cv::Rect2d> getBBOXes() { return m_bboxs; }

	void setROIs(std::vector <cv::Rect> bboxes, std::vector <int> objID, std::vector <int> label , cv::Mat frame, bool clearHistory = false);

	/*
	int  track_(cv::Mat frame, std::vector<cv::Rect>& trackerOutput);
	int  track(cv::Mat frame, std::vector<cv::Rect>& trackerOutput);
	*/
	int  track(cv::Mat frame, std::vector<CObject>& trackerOutput, int frameNum);
	void setROI(const cv::Mat& img, cv::Rect bbox);
	bool isActive() { return m_frameNum > 0; }
	bool isDetected() { return falseDetectionLen == 0; } // currently detected

	void clear(); 
	void clear(int ind);
	void clear(std::vector <int> indices);

	//static cv::Rect setROI_GUI(cv::Mat img); // DDEBUG function

	std::vector <cv::Rect2d> getBBoxs() { return m_bboxs; }

	int getFlaseDetectionLen() { return falseDetectionLen; }


	//int track_main(std::string videoFName, int trackerTypeInd, int skip = 0);


private:
	std::vector  <std::string> m_trackerTypes_str = { "KCF","TLD","BOOSTING","MEDIAN_FLOW","MIL","GOTURN","MOSSE" ,"CSRT", "SIAM_RPN"};

	// vector <string> trackerTypes(types, std::end(types));
	int m_trackerType = 6;
	std::string trackerType;
	int m_frameNum = 0; 			// tracking processed frames
	int falseDetectionLen = 0; 	//count detection failure tail length

	int m_debugLevel = 0;

	int m_badFramesToReset = MAX_BAD_DETECTION_SEC * CONSTANTS::FPS;

	//cv::legacy::MultiTracker  m_multiTracker;

	//std::vector<Ptr<Tracker>> m_algorithms; // TRACKER_NO_LEGACY 
	std::vector<Ptr<legacy::Tracker>> m_algorithms;
	std::vector <cv::Rect2d> m_bboxs;
	std::vector <int> m_objID;
	std::vector <int> m_labels;




};
