#pragma once

#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"

#include "mog.hpp"
//#include "prediction.hpp"
//#include "CObject.hpp"

#include "yolo/types.hpp"
#include "CObject.hpp"
#include "yolo/YOLO_mngr.hpp"
#include "yolo/yolo.hpp"
//#include "trackerBasic.hpp"
#include "dasiamrpn_tracker.hpp"
#include "concluder.hpp"


class CRoi2frame {
public:
	cv::Rect   bbox;
	int frameNum;
};


enum STAGE {
	AGE_1=9999,
	BORN = 0,
	STARTER, // 1,
	FINE,	// 2
	SENIOR, // 3 - once was STABLE 
	STABLE // 4
};


class CDetector {
public:
	// init
	bool init(int camIndex, int w, int h, int imgSize, int pixelWidth, int invertImage, float scaleDisplay = 0.5);
	bool InitGPU();
	void setCamerasInfo(std::vector <CAlert> camerasInfo);
	void setMinPersonDim(int minPersonDim) { m_decipher.setMinPersonDim(minPersonDim); }
	// Process
	int process(void* dataTemp, ALGO_DETECTION_OBJECT_DATA* pObjects);
	int process(cv::Mat frame, ALGO_DETECTION_OBJECT_DATA* pObjects, int frameNum, uint64_t timeStamp);
	int processFrame(cv::Mat &frame);

	int debugSaveDetectionsImages(std::vector <CObject>	detectedObjs, std::vector <int> camIDs = std::vector <int>());

	int getAlertObjectsCount() { return m_decipher.getAlertObjectsNum();  }
	void setDrawing(bool flag) {  doDrawing = flag; }
	
	void addFalseImg(cv::Mat frame, Labels alertLabel)
	{
		m_decipher.addFalseImg(frame, alertLabel);
	}

private:
	bool motionDetected(cv::Mat mask);
	std::vector <cv::Rect> detectByContours(cv::Mat bgMask);

	bool timeForMotion();
	bool timeForDetection();
	bool timeForTracking();

private:
	int m_width = 0;
	int m_height = 0;
	void *m_data = NULL;
	int m_frameNum = 0;
	uint64_t m_timeStamp = 0;
	int m_lastFrameNum_detection = -100;
	int m_lastFrameNum_motion = -100;
	int m_lastFrameNum_tracking = -100;
	int m_cycleNum = 0; // count actual processed frame#
	float m_scaleDisplay = 1.;// 0.7;
	bool  m_motionDetectet = false;


	cv::Mat m_frameOrg; // Original image
	cv::Mat m_frameROI;
	cv::Mat m_frame; // working image
	cv::Mat m_prevFrame; // Prev
	cv::Mat m_bgMask; // MOG2
	cv::Mat m_display;

	CYolo8 m_yolo;
	CSiamTracker      m_tracker;
	CDecipher m_decipher;
	std::vector <CAlert> m_camerasInfo;
	std::vector <int> m_lables_to_detect;
	int m_cameraID = 0;
	int m_invertImg = 0;
	std::vector <Labels> m_detectionTypes; // store all types for detections 

private:
	std::vector<YDetection> m_Yolotput;
	std::vector<cv::Rect>	m_TrackerOutput;
	std::vector<CObject>	m_TrackerObjects;
	std::vector <cv::Rect>  m_BGSEGoutput; // humna candidates
	std::vector <cv::Rect>  m_BGSEGoutputLarge; // Larger objects (not a human)

	CBGSubstruct   m_bgSeg;
	int m_colorDepth = 4;
	Config m_params;
	std::vector <CRoi2frame>  m_roiList;
	unsigned int m_objectID_counter = 0;
	cv::Rect m_camROI = cv::Rect(0,0,0,0);
	bool m_isCuda;
	bool doDrawing = false;
	bool m_motionOnly = false; // if all alerts are motion alerts

};
