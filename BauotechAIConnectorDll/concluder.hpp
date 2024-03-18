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
	const int SAVE_HIDDEN_FRAMES = 2 * 30; // 4;
	const int MAX_SIREN_HIDDEN_FRAMES = 1;
	const int MAX_YOLO_HIDDEN_FRAMES = 30;

}

cv::Point2f center(cv::Rect r);

class CDecipher {
public:
	void init(int debugLevel);
	void setPersonDim(cv::Size dim) { m_maxPresonDim = dim; } // max person size in pixels
	void setObjectDim(cv::Size dim) { m_maxObjectDim = dim; } // max person size in pixels
	void add_(std::vector <cv::Rect>  BGSEGoutput, std::vector <YDetection> YoloOutput, int frameNum);
	std::vector <int>    add(std::vector <cv::Rect>  &trackerOutput, std::vector <YDetection> YoloOutput, int frameNum);
	void add(std::vector <YDetection> YoloOutput,int frameNum);
	std::vector <int>  findDuplicated(std::vector <cv::Rect> trackerBoxes, std::vector <cv::Rect> yoloBoxes);
	std::vector <int> removeDuplicated(std::vector <cv::Rect>& trackerOutput, std::vector <YDetection> YoloOutput);
	int track();
	int track2(int mode);
	std::vector <CObject> getObjects(int frameNum = -1); //  { return m_goodObjects; }
	std::vector <CObject> getObjects(Labels label); //  { return m_goodObjects; }
	std::vector <CObject> getPersonObjects(int frameNum); //  { return m_goodObjects; }
	std::vector <CObject> getVehicleObjects(int frameNum, bool only_moving); //  Other labeled objects (not a persons) & BGSeg 
	std::vector <CObject> getOtherObjects(int frameNum, bool only_moving); //  Other labeled objects (not a persons) & BGSeg 
	std::vector <int> getObjectsInd(int frameNum);
	std::vector <CObject> getHiddenObjects(int curFrameNum, int backward);
	std::vector <CObject> getLastLostObjects(int curFrameNum, int label = -1);

	int size() { return int(m_objects.size()); }

	CObject get(int i) { return m_objects[i].back(); }
	int  numberOfPersonsOnBoard(); // return number of person on board (including hidden objects)

	void	set(std::vector<cv::Point> contour, int label, int max_allowed, int ployID)
	{
		m_alerts.push_back(CAlert(contour, label, max_allowed, ployID));
	}

	int Siren();

	std::vector <CObject> getSirenObjects(float scale = 1.);
	std::vector <CObject> getNewSirenObjects(float scale = 1.);

private:
	bool isMoving(std::vector <CObject> obj);
	bool isStatic(std::vector <CObject> obj);
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
	int		  pruneObjects(int hiddenLen);

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
	cv::Size m_dim;
	int m_frameNum;
	int m_debugLevel = 0;
};

#endif 