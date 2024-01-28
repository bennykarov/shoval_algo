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
#include "yolo/yolo5.hpp"
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
	bool init(int w, int h, int imgSize, int pixelWidth,  char * cameraConfig, float scaleDisplay = 0.5);
	bool InitGPU();
	int process(void* dataTemp, ALGO_DETECTION_OBJECT_DATA* pObjects);
	int process(cv::Mat frame, ALGO_DETECTION_OBJECT_DATA* pObjects);
	int processFrame(cv::Mat &frame);
	/*
	void draw_old(cv::Mat& img, float scale);   // by Concluder (good objects)
	void draw(cv::Mat& img, std::vector <CObject> detections, float scale);   // by Concluder (good objects)
	void draw(cv::Mat &img, std::vector<YDetection> Youtput, float scale);   // for Yolo
	void draw(cv::Mat &img, std::vector<cv::Rect>  rois, float scale);		 // for BGSeg
	void drawInfo(cv::Mat &img);		
	void drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale);
	int getDetectionCount();
	*/
	void setDrawing(bool flag) {  doDrawing = flag; }


private:
	bool motionDetected(cv::Mat mask);
	std::vector <cv::Rect> detectByContours(cv::Mat bgMask);

	bool timeForMotion();
	bool timeForDetection();

private:
	int m_width = 0;
	int m_height = 0;
	void *m_data = NULL;
	int m_frameNum = 0;
	//float m_calcScale = 0.5;
	float m_scaleDisplay = 1.;// 0.7;
	bool  m_motionDetectet = false;


	cv::Mat m_frameOrg; // Original image
	cv::Mat m_frameROI;
	cv::Mat m_frame; // working image
	cv::Mat m_prevFrame; // Prev
	cv::Mat m_bgMask; // MOG2
	cv::Mat m_display;

	CYolo5 m_yolo;
	//CTracker       m_tracker;
	CDecipher m_decipher;
	std::vector <CAlert> m_camerasInfo;


private:
	std::vector<YDetection> m_Youtput;
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

};
