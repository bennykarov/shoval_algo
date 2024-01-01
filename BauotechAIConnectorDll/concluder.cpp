#include <stdio.h>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"


#include "CObject.hpp"
#include "yolo/yolo5.hpp"

#include "utils.hpp"
#include "concluder.hpp"



void CDecipher::init(int debugLevel) { m_debugLevel = debugLevel; 	m_active = true; }

void CDecipher::add(std::vector <cv::Rect>  BGSEGoutput, std::vector <YDetection> YoloOutput, int frameNum)
{
	if (!m_active)
		return;

	std::vector <int> matchInds;

	m_frameNum = frameNum;
	// Add the new BGSeg object - match to current BGSeg objects (if does)
	// Ignore those were matched with YOLO objects 
	//-------------------------------------------------------------------------
	for (auto bgRect : BGSEGoutput) {
		CObject newObj(bgRect, frameNum, 0, DETECT_TYPE::BGSeg, Labels::nonLabled);  // 	CObject(cv::Rect  r, int frameNum, int id, DETECT_TYPE  detectionType, Labels label)
		int ind = bestMatch(bgRect, 0.2);// , matchInds);
		if (ind >= 0) {
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else {
			// New object
			m_objects.push_back(std::vector <CObject>());
			m_objects.back().push_back(newObj);
		}

		//m_objects.back().back().m_finalLabel = calcFinalLable(m_objects.back());
	}

	// Add YOLO object - match to BGSeg objects (if does)
	//----------------------------------------------------
	for (auto Yobj : YoloOutput) {
		CObject newObj(Yobj.box, frameNum, 0, DETECT_TYPE::ML, (Labels)Yobj.class_id);  // 	CObject(cv::Rect  r, int frameNum, int id, DETECT_TYPE  detectionType, Labels label)

		if (newObj.m_label == Labels::person)
			int debug = 10;

		int ind = bestMatch(Yobj.box, 0.5);
		if (ind >= 0) {
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else {
			// New object
			m_objects.push_back(std::vector <CObject>());
			m_objects.back().push_back(newObj);
			if (newObj.m_label == Labels::person)
				int debug = 10;
		}

		//m_objects.back().back().m_finalLabel = calcFinalLable(m_objects.back());
	}

	int rem = pruneObjects();
	/*
	if (m_debugLevel > 2 && rem > 0)
		std::cout << "Number of pruned (elders) object = " << rem << "\n";  //int debug = 10;
	*/

}

void CDecipher::add(std::vector <YDetection> YoloOutput, int frameNum)
{
}


/*------------------------------------------------------------------
 *	Match RECT to prev RECTs in objects list 
 *-----------------------------------------------------------------*/
int CDecipher::bestMatch(cv::Rect box, float overlappedRatio, std::vector <int> ignoreInds)
{
	std::vector <int>  bestInds;
	std::vector <float>  bestScores;

	for (int i = 0; i < m_objects.size(); i++) {
		if (std::find(ignoreInds.begin(), ignoreInds.end(), i) != ignoreInds.end())
			continue; // Ignore this object
		for (auto obj : m_objects[i]) {
			float overlappingRatio = bboxesBounding(obj.m_bbox, box); // most new box overlapped old box
			// Check (1) overlapping ratio (2) The sizes are similars (kind of) (3) the distance is reasonable 
			if (overlappingRatio > overlappedRatio && 
				similarAreas(obj.m_bbox, box, 0.6*0.6) &&
				distance(obj.m_bbox, box) < CONCLUDER_CONSTANTS::MAX_MOTION_PER_FRAME) {
				bestInds.push_back(i);
				bestScores.push_back(overlappingRatio);
			}
		}
	}

	if (bestScores.empty())
		return -1;

	auto it = max_element(bestScores.begin(), bestScores.end());
	int index = it - bestScores.begin();
	return bestInds[index];
}

int CDecipher::track() 
{ 
	m_detectedObjects.clear(); // TEMP clearing

	for (auto& obj : m_objects) {
		CObject consObj = consolidateObj(obj);
		if (!consObj.empty() && obj.back().m_finalLabel != Labels::nonLabled) {
			consObj.m_moving = isMoving(obj);
			m_detectedObjects.push_back(consObj);
		}
	}

	return 0; 
}

#if 0
/*---------------------------------------------------------------------------
	Return only "good" objects - with Label or long enough (with no label)
	A good object is:
	(1) Detected on current (last) frame
	(2) Detected by YOLO or is long enough (by any kind of detection)
 ---------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getObjects_(int frameNum)
{

	for (auto obj : m_objects) {
		obj.back().m_finalLabel = obj.back().m_label =  calcFinalLable(obj);

		if (obj.back().m_frameNum == frameNum &&
			(obj.back().m_finalLabel != Labels::nonLabled || obj.size() >= GOOD_TRACKING_LEN))
			m_goodObjects.push_back(obj.back());
	}

	return goodObjects;
}
#endif 


std::vector <CObject> CDecipher::getObjects(int frameNum, Labels  label)
{
	std::vector <CObject> personObjects;

	std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(personObjects),
		[label](CObject obj) { return obj.m_label == label; });

	return personObjects;
}

/*---------------------------------------------------------------------------
	Get object : person labeled and stable BGSeg  
 ---------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getPersonObjects(int frameNum)
{
	std::vector <CObject> personObjects;

	std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(personObjects),
		[](CObject obj) { return obj.m_label == Labels::person; });

	return personObjects;

}

std::vector <CObject> CDecipher::getVehicleObjects(int frameNum, bool onlyMoving)
{

	std::vector <CObject> vehicleObjects;

	if (onlyMoving)
		std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(vehicleObjects),
			[](CObject obj) { return (obj.m_label == Labels::car || obj.m_label == Labels::truck || obj.m_label == Labels::bus) &&    obj.m_moving > 0; });
	else
		std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(vehicleObjects),
			[](CObject obj) { return obj.m_label == Labels::car || obj.m_label == Labels::truck || obj.m_label == Labels::bus; });


	return vehicleObjects;
}


/*-----------------------------------------------------------------------------------------
 * Get all objects that are NOT person or BGSeg - all other Labeles objects
 * "onlyMoving" - return only objects in motion  
 -----------------------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getOtherObjects(int frameNum, bool onlyMoving)
{

		std::vector <CObject> otherObjects;
		
		if (onlyMoving)
			std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(otherObjects),
				[](CObject obj) { return obj.m_label != Labels::person && obj.m_label != Labels::nonLabled && obj.m_moving > 0; });
		else 
			std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(otherObjects),
				[](CObject obj) { return obj.m_label != Labels::person && obj.m_label != Labels::nonLabled; });


		return otherObjects;
}




std::vector <int> CDecipher::getObjectsInd(int frameNum)
{
	std::vector <int> goodObjectsInd;

	for (int i = 0; i< m_objects.size();i++) {
		if (m_objects[i].back().m_frameNum == frameNum &&
			(m_objects[i].back().m_finalLabel != Labels::nonLabled || m_objects[i].size() >= CONCLUDER_CONSTANTS::GOOD_TRACKING_LEN))
			goodObjectsInd.push_back(i);
	}

	return goodObjectsInd;
}

#if 0
std::vector <CObject> CDecipher::getHiddenObjects(int curFrameNum, int backward)
{
	std::vector <CObject> hiddenObjects;

	for (auto obj : m_objects) {
		if (obj.back().m_frameNum < curFrameNum  && obj.back().m_frameNum >= (curFrameNum - backward) &&
			(obj.back().m_finalLabel != Labels::nonLabled || obj.size() >= GOOD_TRACKING_LEN))
			hiddenObjects.push_back(obj.back());
	}

	return hiddenObjects;
}
#endif 

Labels    CDecipher::calcFinalLable(std::vector <CObject> objectList)
{
	for (auto obj : objectList) {
		if (obj.m_label == Labels::person)
			return Labels::person;
	}
	for (auto obj : objectList) {
		if (obj.m_label != Labels::nonLabled)
			return obj.m_label;
	}

	return Labels::nonLabled;

}

#if 0
CObject   CDecipher::consolidateObj_(std::vector <CObject> objectHist)
{
	CObject consObj = objectHist.back();

	if (objectHist.size() < CONSTANTS::StableLen || m_frameNum - objectHist.back().m_frameNum > CONCLUDER_CONSTANTS::MAX_HIDDEN_FRAMES)
		return CObject();

	// Obj detected by BGSEG - inherit the latest YOLO attribute 
	int backward = MIN(CONCLUDER_CONSTANTS::INHERIT_LABEL_LEN, objectHist.size());
	auto startIt = objectHist.end() - backward;

	auto  itPerson = std::find_if(startIt, objectHist.end(), [](CObject s) { return s.m_label == Labels::person; });
	if (itPerson != objectHist.end()) {
	consObj.m_finalLabel = consObj.m_label = itPerson->m_label;
		// Set original YOLO size to BGSeg rect:
		cv::Point tl = objectHist.back().m_bbox.tl();
		consObj.m_bbox = centerBox(centerOf(objectHist.back().m_bbox), cv::Size(itPerson->m_bbox.width, itPerson->m_bbox.height));
		return consObj;
	}

	auto  itClassified = std::find_if(startIt, objectHist.end(), [](CObject s) { return s.m_label != Labels::nonLabled; });
	if (itClassified != objectHist.end()) {
		consObj.m_finalLabel = consObj.m_label = itClassified->m_label;
		//consObj.m_bbox = resize(itClassified->m_bbox, cv::Size(itClassified->m_bbox.width, itClassified->m_bbox.height));
		return consObj;
	}

	// CASE 3:   BGSeg (survivle) object 
	if (objectHist.size() >= CONCLUDER_CONSTANTS::GOOD_TRACKING_LEN) {
		// smooth rect & pos ...
		return 	objectHist.back();
	}

	return CObject();


}
#endif 

/*----------------------------------------------------------------------------------------------
 * Gather all history information to a final (single) object:
 * If (even) once in history the obj was labeled - inherit this label (for INHERIT_LABEL_LEN  frames)
----------------------------------------------------------------------------------------------*/
CObject   CDecipher::consolidateObj(std::vector <CObject> &objectHist)
{
	CObject consObj = objectHist.back();

	// Obj detected by BGSEG - inherit the latest YOLO attribute 
	int backward = MIN(CONCLUDER_CONSTANTS::INHERIT_LABEL_LEN, objectHist.size());
	int lastInd = objectHist.size() - backward;

	// -1- Look  for  latest  'person' labeled objects
	//----------------------------------------------------
	int i = objectHist.size()-1;
	//auto  itPerson = std::find_if(startIt, objectHist.end(), [](CObject s) { return s.m_label == Labels::person; });
	while (i >= lastInd && objectHist[i].m_label != Labels::person) i--;
	if (i >= lastInd) {
		consObj.m_finalLabel = consObj.m_label = objectHist.back().m_finalLabel = objectHist[i].m_label;
		// Inherit latest labled rect size :
		cv::Point tl = objectHist.back().m_bbox.tl();
		consObj.m_bbox = centerBox(centerOf(objectHist.back().m_bbox), cv::Size(objectHist[i].m_bbox.width, objectHist[i].m_bbox.height));
		return consObj;
	}

	// -2- Other labels than Person
	//---------------------------------
	i = objectHist.size() - 1;
	while (i >= lastInd && objectHist[i].m_label == Labels::nonLabled) i--;
	if (i >= lastInd) {
		consObj.m_finalLabel = consObj.m_label = objectHist.back().m_finalLabel = objectHist[i].m_label;
		/*
		// Inherit original (latest) rect size
		cv::Point tl = objectHist.back().m_bbox.tl();
		consObj.m_bbox = cv::Rect(tl.x, tl.y, objectHist[i].m_bbox.width, objectHist[i].m_bbox.height);
		*/
		return consObj;
	}

	// CASE 3:   BGSeg (unlabeled) & stable  object 
	if (objectHist.size() >= CONCLUDER_CONSTANTS::GOOD_TRACKING_LEN) {
		// smooth rect & pos ...
		return 	objectHist.back();
	}

	return CObject();
}


/*----------------------------------------------------------------
 * Utilities 
 -----------------------------------------------------------------*/
bool CDecipher::isMoving(std::vector <CObject> obj)
{
	const int MIN_LEN = 20;  // in frames 
	const double MIN_DISTANCE = (double)MIN_LEN * 0.5; // in pixels


	if (obj.size() < MIN_LEN)
		return false;

	int dist = distance(centerOf(obj.back().m_bbox), centerOf(obj[obj.size() - MIN_LEN].m_bbox));
	if (dist >= MIN_DISTANCE)
		return true;

	return false;
}

bool CDecipher::isStatic(std::vector <CObject> obj)
{
	const int MIN_LEN = 7;  // in frames 
	const int TOLERANCE = 7;

	if (obj.size() < MIN_LEN)
		return false;

	int dist = distance(centerOf(obj.back().m_bbox), centerOf(obj[obj.size() - MIN_LEN - 1].m_bbox));
	if (dist > TOLERANCE)
		return false;

	return false;
}

inline bool CDecipher::isLarge(std::vector <CObject> obj)
{
	return (obj.back().m_bbox.width > m_maxPresonDim.width || obj.back().m_bbox.height > m_maxPresonDim.height);
}



int CDecipher::pruneObjects()
{
	int orgSize = m_objects.size();
	// Remove un-detected objects (fade)
	for (int i = 0; i < m_objects.size(); i++) {
		int expiredLen = (m_objects[i].back().m_label == Labels::person) ? CONCLUDER_CONSTANTS::MAX_PERSON_HIDDEN_FRAMES : CONCLUDER_CONSTANTS::MAX_OTHERS_HIDDEN_FRAMES;
		if (m_frameNum - m_objects[i].back().m_frameNum > expiredLen)
			m_objects.erase(m_objects.begin() + i--);
	}

	return orgSize - m_objects.size();
}


// return number of person on board (including hidden objects)
int CDecipher::numberOfPersonsOnBoard()
{
	return getPersonObjects(m_frameNum).size();
}

/*-------------------------------------------------------------------
* Selects "bad" (alert)  objects from ALL detected objects
-------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getSirenObjects(float scale)
{
	std::vector <CObject> scaledDetectedObjects;

	// Scale back to origin dimensions is required
	for (auto obj : m_detectedObjects) {
		scaledDetectedObjects.push_back(obj);
		scaledDetectedObjects.back().m_bbox = scaleBBox(obj.m_bbox, scale);
	}

	return m_alert.selectObjects(scaledDetectedObjects);
}
