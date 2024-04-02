#include <stdio.h>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"


#include "CObject.hpp"
#include "yolo/yolo.hpp"

#include "utils.hpp"
#include "concluder.hpp"



void CDecipher::init(int debugLevel) { m_debugLevel = debugLevel; 	m_active = true; }


//----------------------------------------------------------------------------------------------
// Find trackerBoxes that is similar (overlapping) to yoloBoxes
// return thier Indices list
//----------------------------------------------------------------------------------------------
std::vector <int>  CDecipher::findDuplicated(std::vector <cv::Rect>  trackerBoxes, std::vector <cv::Rect> yoloBoxes)
{
	float nearDistance = 20;
	float nearRelativeDist = 0.3;
	float similarDimRatio = 0.6;
	float similarOverlappingRatio = 0.5;

	std::vector <int> trackRemoveIndices, yoloRemoveIndices;


	for (int i = 0; i < trackerBoxes.size(); i++)
		for (int j = 0; j < yoloBoxes.size(); j++) {
			cv::Rect box1 = trackerBoxes[i];
			cv::Rect box2 = yoloBoxes[j];


			if (OverlappingRatio(cv::Rect2f(box1), cv::Rect2f(box2)) > similarOverlappingRatio) {
				trackRemoveIndices.push_back(i);
				yoloRemoveIndices.push_back(j);
				{j = 1000; continue; }// out of yoloBoxes loop
				
			}

			float dimRatrio = similarBox(box1, box2);
			if (dimRatrio < similarDimRatio)
				{ j = 1000; continue;}// out of yoloBoxes loop

			float dist = distance(box1, box2);
			float dim = (float(box1.width + box2.width) / 2. + float(box1.height + box2.height) / 2.) / 2.;
			float relativeDist = dist / dim;
			if (relativeDist > nearRelativeDist)
				{ j = 1000; continue;}// out of yoloBoxes loop

			// At this point bboxes detected as OVERLAPPED 
			trackRemoveIndices.push_back(i);
			yoloRemoveIndices.push_back(j);
		}

	return trackRemoveIndices;
	
}
#if 0
//----------------------------------------------------------------------------------------------
// Find trackerBoxes that is similar (overlapping) to yoloBoxes
// return thier Indices list
//----------------------------------------------------------------------------------------------
std::vector <tuple<int, int>>  CDecipher::findDuplicated2(std::vector <cv::Rect>  trackerBoxes, std::vector <cv::Rect> yoloBoxes)
{
	float nearDistance = 20;
	float nearRelativeDist = 0.3;
	float similarDimRatio = 0.6;
	float similarOverlappingRatio = 0.5;

	std::vector <int> trackRemoveIndices, yoloRemoveIndices;
	std::vector <tuple<int, int>> matchedIndices;


	for (int i = 0; i < trackerBoxes.size(); i++)
		for (int j = 0; j < yoloBoxes.size(); j++) {
			cv::Rect box1 = trackerBoxes[i];
			cv::Rect box2 = yoloBoxes[j];


			if (OverlappingRatio(cv::Rect2f(box1), cv::Rect2f(box2)) > similarOverlappingRatio) {
				trackRemoveIndices.push_back(i);
				yoloRemoveIndices.push_back(j);
				matchedIndices.push_back(std::make_tuple(i, j));
				j = 1000; continue;// out of yoloBoxes loop

			}

			float dimRatrio = similarBox(box1, box2);
			if (dimRatrio < similarDimRatio)
				j = 1000; continue;// out of yoloBoxes loop

			float dist = distance(box1, box2);
			float dim = (float(box1.width + box2.width) / 2. + float(box1.height + box2.height) / 2.) / 2.;
			float relativeDist = dist / dim;
			if (relativeDist > nearRelativeDist)
				j = 1000; continue;// out of yoloBoxes loop

			// At his point hboxes defined as overlapped
			trackRemoveIndices.push_back(i);
			yoloRemoveIndices.push_back(j);
		}

	return matchedIndices;

}
#endif 

/*--------------------------------------------------------------------------------------------------------------------------------
* Remove Tacker objects that match YOLO (take the YOLO as master)
 *--------------------------------------------------------------------------------------------------------------------------------*/
std::vector <int> CDecipher::removeDuplicated(std::vector <cv::Rect>& trackerOutput, std::vector <YDetection> YoloOutput)
{
	std::vector <int> duplicateInds;
	int duplicateTrkBoxes = 0;
	if (!YoloOutput.empty() && !trackerOutput.empty()) { // && !m_detectedObjects.empty()) {
		std::vector <cv::Rect> yoloBoxs;
		for (auto obj : YoloOutput)
			yoloBoxs.push_back(obj.box);
		


		duplicateInds = findDuplicated(trackerOutput, yoloBoxs);

		//bool hasDuplicates = std::adjacent_find(duplicateInds.begin(), duplicateInds.end()) != duplicateInds.end();

		// Remove duplicated obj from tracker output:
		std::sort(duplicateInds.begin(), duplicateInds.end(), greater<int>());
		for (auto ind : duplicateInds) {
			trackerOutput.erase(trackerOutput.begin() + ind);
		}
	}

	return duplicateInds;
}

#if 0
//----------------------------------------------------------------------------------------------
// Remove indices to remove from trackerBoxes that is similar (overlapping) yoloBoxes
//----------------------------------------------------------------------------------------------
int CDecipher::removeDuplicated_(std::vector <CObject> &m_detectedObjects, std::vector <cv::Rect> yoloBoxes)
{
	std::vector <cv::Rect> trackerBoxes;

	float closeDistance = 20;
	float goodRelativeDist = 0.1;
	float goodDimRatio = 0.8;

	std::vector <int> trackRemoveIndices, yoloRemoveIndices;


	for (int i = 0; i < trackerBoxes.size(); i++)
		for (int j = 0; j < yoloBoxes.size(); j++) {
			cv::Rect box1 = trackerBoxes[i];
			cv::Rect box2 = yoloBoxes[j];
			float dimRatrio = similarBox(box1, box2);
			if (dimRatrio < goodDimRatio)
				continue;

			float dist = distance(box1, box2);
			float dim = (float(box1.width + box2.width) / 2. + float(box1.height + box2.height) / 2.) / 2.;
			float relativeDist = dist / dim;
			if (relativeDist > goodRelativeDist)
				continue;

			// At his point hboxes defined as overlapped
			trackRemoveIndices.push_back(i);
			yoloRemoveIndices.push_back(j);
		}

	// Remove boxes from tracker list
	for (int i = 0; i < trackerBoxes.size(); i++) {
		if (std::find(trackRemoveIndices.begin(), trackRemoveIndices.end(), i) != trackRemoveIndices.end()) {
			trackerBoxes.erase(trackerBoxes.begin() + i--);
		}
	}

	return trackRemoveIndices.size();


}
#endif 


std::vector <int>   CDecipher::add(std::vector <CObject>& trackerObjects, std::vector <YDetection> YoloOutput, int frameNum)
{
	if (!m_active)
		return std::vector <int>();

	std::vector <int> matchInds;
	m_frameNum = frameNum;


	std::vector <Labels>  activeLabels = getActiveLabels();

	//-------------------------------------------------------------------------
	// Add YOLO objects 
	//----------------------------------------------------
	for (auto Yobj : YoloOutput) {
		
		if (std::find(activeLabels.begin(), activeLabels.end(),  (Labels)Yobj.class_id) == activeLabels.end())
			continue;

		CObject newObj(Yobj.box, frameNum, 0, DETECT_TYPE::ML, (Labels)Yobj.class_id);  // 	CObject(cv::Rect  r, int frameNum, int id, DETECT_TYPE  detectionType, Labels label)

		int ind = bestMatch(Yobj.box, 0.3, matchInds);

		//int ind = bestMatch(Yobj.box, 0.01, matchInds);
		if (ind >= 0) {
			newObj.m_ID = m_objects[ind].back().m_ID;
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else {
			// New object
			newObj.m_ID = m_UniqueID++;
			m_objects.push_back(std::vector <CObject>());
			m_objects.back().push_back(newObj);
		}

	}

	//--------------------------------------------------------
	// Add Tracker objects 
	//----------------------------------------------------

	// (Tracker-A) First remove tracker boxes  that overlapped with yolo 
	//-------------------------------------------------------------------
	std::vector <cv::Rect> trackerOutput;
	for (auto obj : trackerObjects)
		trackerOutput.push_back(obj.m_bbox);

	std::vector <int> duplicateInds;
	if (YoloOutput.size() > 0 && trackerOutput.size() > 0) {
		duplicateInds = removeDuplicated(trackerOutput, YoloOutput);
		// Remove duplicated obj from :
		std::sort(duplicateInds.begin(), duplicateInds.end(), greater<int>());
		for (auto ind : duplicateInds)
			trackerObjects.erase(trackerObjects.begin() + ind);

	}

	// (Tracker-B) Add tracker boxes  
	//-------------------------------
	matchInds.clear();

	for (auto obj : trackerObjects) {
		//CObject newObj(obj.m_bbox, frameNum, 0, DETECT_TYPE::Tracking, getYoloClassIndex("giraffe"));  // DDEBUG trkFrame tyep for tracker 
		CObject newObj(obj.m_bbox, frameNum, 0, DETECT_TYPE::Tracking, obj.m_label);  // DDEBUG trkFrame tyep for tracker 

		int ind = bestMatch(obj.m_bbox, 0.01, matchInds); // 'frameNum' -> ignore objects currently detected  by YOLO 
		if (ind >= 0 && m_objects[ind].back().m_frameNum < frameNum) {
			newObj.m_ID = m_objects[ind].back().m_ID;
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else 			
			int debug = 10;  // already detected by YOLO -OR- a new object (ignore)
	}

	int rem = pruneObjects(CONCLUDER_CONSTANTS::SAVE_HIDDEN_FRAMES);

	return duplicateInds;

}
void CDecipher::add_(std::vector <cv::Rect>  BGSEGoutput, std::vector <YDetection> YoloOutput, int frameNum)
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
			newObj.m_ID = m_objects[ind].back().m_ID;
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else {
			// New object (vector of objects along time)
			newObj.m_ID = m_UniqueID++;
			m_objects.push_back(std::vector <CObject>());
			m_objects.back().push_back(newObj);
		}
	}

	// Add YOLO object - match to BGSeg objects (if does)
	//----------------------------------------------------
	for (auto Yobj : YoloOutput) {
		CObject newObj(Yobj.box, frameNum, 0, DETECT_TYPE::ML, (Labels)Yobj.class_id);  // 	CObject(cv::Rect  r, int frameNum, int id, DETECT_TYPE  detectionType, Labels label)

		int ind = bestMatch(Yobj.box, 0.3, matchInds);
		if (ind >= 0) {
			newObj.m_ID = m_objects[ind].back().m_ID;
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else {
			// New object
			newObj.m_ID = m_UniqueID++;
			m_objects.push_back(std::vector <CObject>());
			m_objects.back().push_back(newObj);
		}

		//m_objects.back().back().m_finalLabel = calcFinalLable(m_objects.back());
	}


	int rem = pruneObjects(CONCLUDER_CONSTANTS::SAVE_HIDDEN_FRAMES);

}

#if 0
void CDecipher::add(std::vector <YDetection> YoloOutput, int frameNum)
{
	if (!m_active)
		return;


	//m_objects.clear(); // NEW NEW !!!

	std::vector <int> matchInds;

	m_frameNum = frameNum;
	// Add the new BGSeg object - match to current BGSeg objects (if does)
	// Ignore those were matched with YOLO objects 
	//-------------------------------------------------------------------------

	// Add YOLO object - match to BGSeg objects (if does)
	//----------------------------------------------------
	for (auto Yobj : YoloOutput) {
		CObject newObj(Yobj.box, frameNum, 0, DETECT_TYPE::ML, (Labels)Yobj.class_id);  // 	CObject(cv::Rect  r, int frameNum, int id, DETECT_TYPE  detectionType, Labels label)

		int ind = bestMatch(Yobj.box, 0.3, matchInds);
		if (ind >= 0) {
			newObj.m_ID = m_objects[ind].back().m_ID;
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else {
			// New object
			newObj.m_ID = m_UniqueID++;
			m_objects.push_back(std::vector <CObject>());
			m_objects.back().push_back(newObj);
		}
	}


	int rem = pruneObjects(CONCLUDER_CONSTANTS::SAVE_HIDDEN_FRAMES); // remove "old "old" hidden objects 
}
#endif 


/*-------------------------------------------------------------------------------
 *	Match RECT to prev RECTs in 'm_objects' list 
 * Params:
 * ignoreInds - these indices was already matched in prev cycles, ignore 
 * frameNumToIgnore - Ignore object that detect in this frameNum (current frame)
 *--------------------------------------------------------------------------------*/
int CDecipher::bestMatch(cv::Rect box, float OverlappedThrash, std::vector <int> ignoreInds, int frameNumToIgnore)
{
	std::vector <int>  bestInds;
	std::vector <float>  bestScores;

	for (int i = 0; i < m_objects.size(); i++) {
		if (std::find(ignoreInds.begin(), ignoreInds.end(), i) != ignoreInds.end())
			continue; // Ignore this object
		if (m_objects[i].back().m_frameNum == frameNumToIgnore)
			continue;


		//for (auto obj : m_objects[i])
		auto obj = m_objects[i].back();
			{
			float overlappingRatio = bboxesBounding(obj.m_bbox, box); // most new box overlapped old box
			int maxDistance = MIN(100, CONCLUDER_CONSTANTS::MAX_MOTION_PER_FRAME * (m_frameNum - obj.m_frameNum));
			// Check (1) overlapping ratio (2) The sizes are similars (kind of) (3) the distance is reasonable 
			if (overlappingRatio > OverlappedThrash ||
				(similarBox(obj.m_bbox, box, 0.6*0.6) &&  distance(obj.m_bbox, box) < maxDistance)) 
			{
				bestInds.push_back(i);
				bestScores.push_back(overlappingRatio);
			}
			else {
				/*
				// DDEBUG INFO 
				int error;
				error = overlappingRatio > OverlappedThrash ? 0 : 1;
				error += similarBox(obj.m_bbox, box, 0.6 * 0.6) ? 0 : 10;
				error +=  distance(obj.m_bbox, box) < CONCLUDER_CONSTANTS::MAX_MOTION_PER_FRAME ? 0 : 100;
				int debug = 10;
				*/
			}
		}
	}

	if (bestScores.empty())
		return -1;

	auto it = max_element(bestScores.begin(), bestScores.end());
	int index = it - bestScores.begin();
	return bestInds[index];
}

#if 0
int CDecipher::track_old()
{
	m_detectedObjects.clear(); // TEMP clearing

	for (auto& obj : m_objects) {
		CObject consObj = consolidateObj(obj);
		if (!consObj.empty() && obj.back().m_finalLabel != Labels::nonLabled) {
			consObj.m_moving = isMoving(obj);
			consObj.m_age = obj.size();
			m_detectedObjects.push_back(consObj);
		}
	}

	return 0;
}
#endif 

/*---------------------------------------------------------------
	Update final (current) 'm_detectedObjects' from 'm_objects'
 ----------------------------------------------------------------*/
int CDecipher::track(int mode)
{
	m_detectedObjects.clear(); // TEMP clearing
	m_prunedIDs.clear();

	for (auto& obj : m_objects)
		//if (obj.back().m_frameNum == m_frameNum)
		if (getHiddenLen(obj.back()) <= CONCLUDER_CONSTANTS::MAX_SIREN_HIDDEN_FRAMES &&
			getHiddenLen(obj, DETECT_TYPE::ML) <= 30 * 2) // DDEBUG CONST   
		{
			obj.back().m_age = obj.size(); // update age 
			m_detectedObjects.push_back(obj.back());
		}
		else {
			if (obj.back().m_ID == 8)
				int debug = 10;

			m_prunedIDs.push_back(obj.back().m_ID);

			if (obj.back().m_ID == 5)
				int debug = 10;
			if (getHiddenLen(obj, DETECT_TYPE::ML) >= 6 * 3)
				int debug = 10;
		}
	return 0;
}


/*---------------------------------------------------------------------------
	get object of 'frameNum' (or later)
 ---------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getObjects(int frameNum)
{
	std::vector <CObject> labeledObjects;

	std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(labeledObjects),
		[frameNum](CObject obj) { return obj.m_frameNum >= frameNum; });

	return labeledObjects;
}
/*---------------------------------------------------------------------------
	get NEW object of 'frameNum' (or later)
 ---------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getNewObjects(int frameNum)
{
	std::vector <CObject> newObjects;

	std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(newObjects),
		[frameNum](CObject obj) { return obj.m_frameNum >= frameNum && obj.m_age == 1; });

	return newObjects;
}


/*--------------------------------------------------------------------------------------------
* Check if -current- YOLO box is  of a differnet dimension than the -prev- Tracker dimension
* Return these diff objects 
----------------------------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getBadROITrackerObject(int frameNum)
{
	const float minSimilarity = 0.7;
	std::vector <CObject> objectsToUpdate;

	for (auto obj : m_objects) {
		if (obj.size() > 1 && obj.back().m_frameNum == frameNum && obj.back().m_detectionType == DETECT_TYPE::ML && obj[obj.size() - 2].m_detectionType == DETECT_TYPE::Tracking) {
			auto r1 = obj.back().m_bbox;
			auto r2 = obj[obj.size() - 2].m_bbox;

			float ratioW = (float)r1.width / (float)r2.width;
			if (ratioW > 1.)
				ratioW = 1. / ratioW;
			if (ratioW >= minSimilarity)
				continue;

			float ratioH = (float)r1.height / (float)r2.height;
			if (ratioH > 1.)
				ratioH = 1. / ratioH;
			if (ratioH >= minSimilarity)
				continue;

			// At this point obj BBOX it sagnificantly different 
			objectsToUpdate.push_back(obj.back());
		}
	}

	//std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(badROIObjects),[frameNum](CObject obj) { return obj.m_frameNum == frameNum; && });

	return objectsToUpdate;


}

/*---------------------------------------------------------------------------
* RETUN specific label objects OR ALL TYPES if label == none
---------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getObjects(Labels  label)
{
	std::vector <CObject> labeledObjects;

	if (label == Labels::nonLabled)
		return m_detectedObjects;

	std::copy_if(m_detectedObjects.begin(), m_detectedObjects.end(), std::back_inserter(labeledObjects),
		[label](CObject obj) { return obj.m_label == label; });

	return labeledObjects;
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


std::vector <CObject> CDecipher::getLastLostObjects(int curFrameNum, int label)
{
	std::vector <CObject> lostObjects;

	for (auto obj : m_objects) {
		if ((obj.back().m_frameNum + 1) ==  curFrameNum)
			lostObjects.push_back(obj.back());
	}

	return lostObjects;
}

int   CDecipher::calcFinalLable(std::vector <CObject> objectList)
{
	for (auto obj : objectList) {
		if (obj.m_label == (int)Labels::person)
			return Labels::person;
	}
	for (auto obj : objectList) {
		if (obj.m_label != (int)Labels::nonLabled)
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
	return (obj.back().m_bbox.width > m_maxObjectDim.width || obj.back().m_bbox.height > m_maxObjectDim.height);
}



/*----------------------------------------------------------------
 * Prune objects that are too old (not detected for a long time)
 -----------------------------------------------------------------*/		
int CDecipher::pruneObjects(int hiddenLen)
{
	int orgSize = m_objects.size();
	// Remove un-detected objects (fade)
	for (int i = 0; i < m_objects.size(); i++) {
		if (m_frameNum - m_objects[i].back().m_frameNum > hiddenLen) {
			m_objects.erase(m_objects.begin() + i--);
		}
	}

	// Remove tracker long tracking w\o YOLO confirmation:
#if 0
	// remove after SIAM tracker  
		for (int i = 0; i < m_objects.size(); i++) {
		if (getHiddenLen(m_objects[i], DETECT_TYPE::ML) > CONCLUDER_CONSTANTS::MAX_YOLO_HIDDEN_FRAMES) {
			//Beep(1000, 50); 
			std::cout << "prune tracker only lenght for " << int(CONCLUDER_CONSTANTS::MAX_YOLO_HIDDEN_FRAMES) << " frames \n";// DDEBUG DDEBUG DDEBUG DDEBUG 
			m_objects.erase(m_objects.begin() + i--);
		}		
	}
#endif 

	return orgSize - m_objects.size();
}


// return number of person on board (including hidden objects)
int CDecipher::numberOfPersonsOnBoard()
{
	return getPersonObjects(m_frameNum).size();
}

/*-------------------------------------------------------------------
* Selects "intuder" (alert)  objects from ALL detected objects
-------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getSirenObjects(float scale)
{
	std::vector <CObject> scaledDetectedObjects, sirenObjects;

	// Scale back to origin dimensions is required
	for (auto obj : m_detectedObjects) {
		if (getHiddenLen(obj) <= CONCLUDER_CONSTANTS::MAX_SIREN_HIDDEN_FRAMES) { // Allow only fresh detected objects
			scaledDetectedObjects.push_back(obj);
			scaledDetectedObjects.back().m_bbox = UTILS::scaleBBox(obj.m_bbox, scale);
		}
	}

	for (auto alert : m_alerts) {
		std::vector <CObject> sirenSingleCam =  alert.selectObjects(scaledDetectedObjects);
		sirenObjects.insert(sirenObjects.end(), sirenSingleCam.begin(), sirenSingleCam.end());
	}


	return sirenObjects;
}


std::vector <CObject> CDecipher::getNewSirenObjects(float scale)
{
	std::vector <CObject> scaledDetectedObjects, sirenObjects;

	// Scale back to origin dimensions is required
	for (auto obj : m_detectedObjects) {
		if (!getHiddenLen(obj) < 2)  { // Allow only fresh detected objects
			scaledDetectedObjects.push_back(obj);
			scaledDetectedObjects.back().m_bbox = UTILS::scaleBBox(obj.m_bbox, scale);
		}
	}

	for (auto alert : m_alerts) {
		std::vector <CObject> sirenSingleCam = alert.selectObjects(scaledDetectedObjects);
		sirenObjects.insert(sirenObjects.end(), sirenSingleCam.begin(), sirenSingleCam.end());
	}
	return sirenObjects;
}


int CDecipher::Siren()
{
	int siren = 0;
	for (auto alert : m_alerts)
		siren += alert.Siren(m_detectedObjects);
	return siren;
}

/*---------------------------------------------------------------
* return length of hidden objects of a specific detection TYPE
---------------------------------------------------------------*/
int CDecipher::getHiddenLen(std::vector <CObject> objects, DETECT_TYPE  detectionType)
{
	for (int i = objects.size()-1; i>=0; i--) {
		if (objects[i].m_detectionType == detectionType)
			return (m_frameNum - objects[i].m_frameNum);
	}

	return objects.size();
}


std::vector <Labels>   CDecipher::getActiveLabels()
{
	std::vector <Labels> activeLabels;

	for (auto alert : m_alerts)
		activeLabels.push_back((Labels)alert.m_label);

	return activeLabels;
}
