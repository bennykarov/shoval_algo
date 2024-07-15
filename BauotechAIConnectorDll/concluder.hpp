#ifndef CONCLUDER_HPP
#define CONCLUDER_HPP

#pragma once

#include "CObject.hpp"
#include "alert.hpp"

namespace CONCLUDER_CONSTANTS
{
	//const int CLOSE_ENOUGH = 30; // pixels
	const int MAX_MOTION_PER_FRAME = 20;//  10;
	const int GOOD_TRACKING_LEN = 20;
	const int INHERIT_LABEL_LEN  = 30 * 1; //  1 sec
	const int KEEP_HIDDEN_FRAMES = 30*2;
	const int MAX_YOLO_HIDDEN_FRAMES = int(KEEP_HIDDEN_FRAMES/2);
	const int MAX_SIREN_HIDDEN_FRAMES = 3;
	// YOLO related const:
	const int MIN_STDDEV_FOR_BOX = 20; // min color variation (contrast) rquired in detection image
	const float MIN_BOX_RATION_FOR_PERSON = 0.8; // Person box should be nerrow (far from square)
	const float HIGH_YOLO_CONFIDANCE = 0.9; // Person box should be nerrow (far from square)
	const int   MIN_OBJECT_SIDE_SIZE = 20; // Object box width or height (the bigger of the two)

	//const int MIN_STABLE_DETECTION_FRAMES = 7;
	const int MIN_STABLE_DETECTION_FRAMES = 3; // 1 for load balancer where framerate is LOW ;

}

cv::Point2f center(cv::Rect r);

class CDecipher {
public:
	void init(cv::Size imgDim, int debugLevel);
	void setPersonDim(cv::Size dim) { m_maxPresonDim = dim; } // max person size in pixels
	void setObjectDim(cv::Size dim) { m_maxObjectDim = dim; } // max person size in pixels
	void add_(std::vector <cv::Rect>  BGSEGoutput, std::vector <YDetection> YoloOutput, int frameNum);
	//std::vector <int>    add(std::vector <cv::Rect>& trackerOutput, std::vector <YDetection> YoloOutput, int frameNum);
	std::vector <int>    add(std::vector <CObject>& trackerOutput, std::vector <YDetection> YoloOutput, int frameNum);
	void add(std::vector <YDetection> YoloOutput,int frameNum);
	//static std::vector <std::tuple<int, int>>  findDuplicated2(std::vector <cv::Rect> trackerBoxes, std::vector <cv::Rect> yoloBoxes);
	int track_old();
	int track(int mode);
	std::vector <CObject> getObjects(int frameNum = -1); 
	std::vector <CObject> getObjects(Labels label); 
	std::vector <CObject> getNewObjects(int frameNum = -1); 
	std::vector <CObject> getBadROITrackerObject(int frmaeNum);


	std::vector <CObject> getPersonObjects(int frameNum); //  { return m_goodObjects; }
	std::vector <CObject> getVehicleObjects(int frameNum, bool only_moving); //  Other labeled objects (not a persons) & BGSeg 
	std::vector <CObject> getOtherObjects(int frameNum, bool only_moving); //  Other labeled objects (not a persons) & BGSeg 
	std::vector <int> getObjectsInd(int frameNum);
	std::vector <CObject> getHiddenObjects(int curFrameNum, int backward);
	std::vector <CObject> getLastLostObjects(int curFrameNum, int label = -1);

	int size() { return int(m_objects.size()); }

	CObject get(int i) { return m_objects[i].back(); }
	int  numberOfPersonsOnBoard(); // return number of person on board (including hidden objects)


	void    clear()
	{
		m_alerts.clear();
	}

	void	set(std::vector<cv::Point> contour, int label, int motionType, int max_allowed, int ployID)
	{
		m_alerts.push_back(CAlert(contour, label, motionType, max_allowed, ployID));
	}


	bool suspectedAsFalse(CObject obj, Labels  alertLabel, cv::Mat *frameImg);

	int Siren();

	std::vector <CObject> getSirenObjects(float scale = 1., cv::Mat* frame = nullptr);
	//std::vector <CObject> getNewSirenObjects(float scale = 1.);

	std::vector <int> getIDStoPrune() { return m_pruneObjIDs; }
	//std::vector <int> getIDStoRenew() { return m_renewObjIDs; }
	std::vector <CObject> getTrkObjsToRenew();

private:
	std::vector <int>  findDuplicated(std::vector <cv::Rect> trackerBoxes, std::vector <cv::Rect> yoloBoxes);
	std::vector <int> removeDuplicated(std::vector <cv::Rect>& trackerOutput, std::vector <YDetection> YoloOutput);
	bool isMoving(std::vector <CObject> obj);
	int  isStatic(std::vector <CObject> obj);
	bool isStatic_OLD(std::vector <CObject> obj);
	bool isLarge(std::vector <CObject> obj);
	bool isHidden(std::vector <CObject> obj) { return obj.back().m_frameNum < m_frameNum; }
	bool isHidden(CObject obj) { return obj.m_frameNum < m_frameNum; }
	int  getHiddenLen(CObject obj) { return  m_frameNum - obj.m_frameNum; }
	int  getHiddenLen(std::vector <CObject> objects, DETECT_TYPE  detectionType);

private:
	int match(std::vector <cv::Rect>);
	int match(std::vector <YDetection> detecions);
	//int bestMatch(YDetection Yobj);
	int bestMatch(cv::Rect r, float overlappedRatio, std::vector <int> ignore= std::vector <int>(), int frameNum = -1);
	//CObject   consolidateObj_(std::vector <CObject> objectList);
	CObject   consolidateObj(std::vector <CObject> &objectList);
	int    calcFinalLable(std::vector <CObject> obj);
	int		  pruneObjects();
	std::vector <Labels>   getActiveLabels();
	float stableConfidance(std::vector <CObject> obj);

private:

	unsigned long m_UniqueID = 0;

	std::vector <CAlert> m_alerts;

	std::vector <std::vector <CObject>> m_objects;
	std::vector <CObject> m_detectedObjects;
	//std::vector <CObject> m_personObjects;
	//std::vector <CObject> m_otherObjects;

	int m_idCounter = 0;
	bool m_active = false;
	cv::Size m_maxPresonDim = cv::Size(150, 200); // DDEBUG CONST 
	cv::Size m_maxObjectDim = cv::Size(350, 350); // DDEBUG CONST 
	cv::Size m_imgDim;
	//cv::Mat *m_framePtr;
	int m_frameNum;
	int m_debugLevel = 0;
	std::vector <int> m_pruneObjIDs; // keep id to prune for Tracker 
	std::vector <int> m_TrkToRenewIDs; // Renew (setROI) lost objects for Tracker 
};

#endif 