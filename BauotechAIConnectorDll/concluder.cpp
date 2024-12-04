#include <stdio.h>
#include <algorithm>
#include <vector>
#include <numeric>
#include<ranges>

#include "opencv2/opencv.hpp"


#include "CObject.hpp"
#include "yolo/yolo.hpp"

#include "config.hpp"
#include "logger.hpp"
#include "utils.hpp"
#include "concluder.hpp"


#include <filesystem>

using std::filesystem::current_path;



using namespace CONCLUDER_CONSTANTS;

void CDecipher::init(int camID, cv::Size imgDim, int debugLevel) { 
	m_camID = camID;
	m_imgDim  = imgDim;  
	m_debugLevel = debugLevel; 	
	m_active = true; 

	//m_predictor.init();

	readFalseList();
}

/*--------------------------------------------------------------------------
// UTILITIES :
--------------------------------------------------------------------------*/

/*----------------------------------------------------
* Set yolo min conf according to:
* label - person required LOWER  conf
* isMoving - moving objects required  LOWER conf
* Size - large objects required HIGHER conf
 ----------------------------------------------------*/
float GET_MIN_YOLO_CONFIDENCE(CObject obj)
{
	const int smallDim = 50;
	float conf = 0;
	bool isMoving = obj.isMoving();

	switch (obj.m_label)
	{
	case Labels::person:
		if (obj.m_bbox.width < smallDim  && obj.m_bbox.width < smallDim)
			conf = isMoving ? 0.2 : 0.4;
		else
			conf = isMoving ? 0.4 : 0.5;
		break;
	default:
		conf = isMoving ? 0.4 : 0.5;
	}

	return conf;
}

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
		std::sort(duplicateInds.begin(), duplicateInds.end(), greater<int>()); // reverse order 
		for (auto ind : duplicateInds) {
			trackerOutput.erase(trackerOutput.begin() + ind);
		}
	}

	return duplicateInds;
}

int  CDecipher::addTrackerObjects(std::vector <CObject> trackerObjects, std::vector <YDetection> YoloOutput, int frameNum, bool prevWasYolo)
{
	std::vector <int> matchInds;
	std::vector <int> duplicateInds; // tracking and YOLO duplications 

	// (Tracker-A) First ignore tracker boxes  that overlapped with yolo (preferred YOLO)
	//------------------------------------------------------------------------------------------
	if (YoloOutput.size() > 0 && trackerObjects.size() > 0) {
		std::vector <cv::Rect> trackerOutput;

		for (auto obj : trackerObjects)
			trackerOutput.push_back(obj.m_bbox);

		//if (YoloOutput.size() > 0 && trackerOutput.size() > 0) 
		{
			duplicateInds = removeDuplicated(trackerOutput, YoloOutput);
			// remove duplicated :
			std::sort(duplicateInds.begin(), duplicateInds.end(), greater<int>());
			for (auto ind : duplicateInds)
				trackerObjects.erase(trackerObjects.begin() + ind);
		}
	}

	// (Tracker-B) Add tracker boxes  
	//-------------------------------
	matchInds.clear();

	for (auto obj : trackerObjects) {
		CObject newObj = obj;

		int ind = bestMatch(obj, matchInds); // 'frameNum' -> ignore objects currently detected  by YOLO 
		if (ind >= 0 && m_objects[ind].back().m_frameNum < frameNum) {
#if 0
			bool mergeBox = false;
			if (mergeBox)
			{
				// Merge bbox size - using YOLO last detection
				// Find last yolo box 
				auto it = find_if(m_objects[ind].rbegin(), m_objects[ind].rend(), [](CObject  obj) { return obj.m_detectionType == DETECT_TYPE::ML; });
				cv::Rect yoloBox = it->m_bbox;

				// Smoothing bboex over frames 
				// - give 0.8 of the YOLO size, than alphaBlend with prev one.
				float AlphaBland = 0.8;
				// -1- set 80% of the size from Yolo:
				cv::Rect yoloMergedBox = UTILS::mergeBBoxSize(obj.m_bbox, yoloBox, AlphaBland);
				// -2- Merge (0.5/0.5) with prev detected box:
				newObj.m_bbox = UTILS::mergeBBoxes(yoloMergedBox, m_objects[ind].back().m_bbox, 0.5);
			}
#endif 

			newObj.m_ID = m_objects[ind].back().m_ID;

			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
	}


	// Check that all yolo detections was tracked. 
	// If not add the untracked it to track list (setROI)
	if (!trackerObjects.empty() && prevWasYolo) {
		//checkUntrackedTrkObjects(); // fill the m_TrkToRenewIDs
		for (int i = 0; i < m_objects.size(); i++)
			if (m_objects[i].back().m_detectionType == DETECT_TYPE::ML) {  // Check only fresh YOLO objects 
				int id = m_objects[i].back().m_ID;
				auto it = find_if(trackerObjects.begin(), trackerObjects.end(), [id](CObject obj) { return obj.m_ID == id; });
				if (it == trackerObjects.end()) // objet is missing in Tracker detections
					m_TrkToRenewIDs.push_back(i);
			}

		prevWasYolo = false;
	}


	//return matchInds.size();
	return duplicateInds.size();
}

/*------------------------------------------------------------------------------------------------------------------------------
* Add new objects to the list of objects
* If object can be assigned to a object in the existing list (using bestMatch()) - add it to existing object
* If not - Add a new object to the list
-------------------------------------------------------------------------------------------------------------------------------*/
std::vector <int>   CDecipher::add(std::vector <CObject>& trackerObjects, std::vector <YDetection> YoloOutput, int frameNum)
{
	if (!m_active)
		return std::vector <int>();

	static bool prevWasYolo = false;
	if (!YoloOutput.empty())
		prevWasYolo = true;


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

		newObj.m_confidence = Yobj.confidence;

		int ind = bestMatch(newObj, matchInds);
		if (ind >= 0) {
			// Tracked object:
			newObj.m_ID = m_objects[ind].back().m_ID;
			m_objects[ind].push_back(newObj);
			matchInds.push_back(ind);
		}
		else {
			// A New object:
			newObj.m_ID = m_UniqueID++;

			m_objects.push_back(std::vector <CObject>());
			m_objects.back().push_back(newObj);
		}
	}

	//------------------------------------------------------------------------------------------
	// Add Tracker objects 
	//------------------------------------------------------------------------------------------
	std::vector <int> duplicateInds; // tracking and YOLO duplications 

	if (trackerObjects.size() > 0)
		addTrackerObjects(trackerObjects, YoloOutput, frameNum, prevWasYolo);
#if 0
	{
		// (Tracker-A) First ignore tracker boxes  that overlapped with yolo (preferred YOLO)
		//------------------------------------------------------------------------------------------
		if (YoloOutput.size() > 0 && trackerObjects.size() > 0) {
			std::vector <cv::Rect> trackerOutput;

			for (auto obj : trackerObjects)
				trackerOutput.push_back(obj.m_bbox);

			//if (YoloOutput.size() > 0 && trackerOutput.size() > 0) 
			{
				duplicateInds = removeDuplicated(trackerOutput, YoloOutput);
				// remove duplicated :
				std::sort(duplicateInds.begin(), duplicateInds.end(), greater<int>());
				for (auto ind : duplicateInds)
					trackerObjects.erase(trackerObjects.begin() + ind);
			}
		}

		// (Tracker-B) Add tracker boxes  
		//-------------------------------
		matchInds.clear();

		for (auto obj : trackerObjects) {
			CObject newObj = obj;

			int ind = bestMatch(obj.m_bbox, matchInds); // 'frameNum' -> ignore objects currently detected  by YOLO 
			if (ind >= 0 && m_objects[ind].back().m_frameNum < frameNum) {
				bool mergeBox = false;
				if (mergeBox)
				{
					// Merge bbox size - using YOLO last detection
					// Find last yolo box 
					auto it = find_if(m_objects[ind].rbegin(), m_objects[ind].rend(), [](CObject  obj) { return obj.m_detectionType == DETECT_TYPE::ML; });
					cv::Rect yoloBox = it->m_bbox;

					// Smoothing bboex over frames 
					// - give 0.8 of the YOLO size, than alphaBlend with prev one.
					float AlphaBland = 0.8;
					// -1- set 80% of the size from Yolo:
					cv::Rect yoloMergedBox = UTILS::mergeBBoxSize(obj.m_bbox, yoloBox, AlphaBland);
					// -2- Merge (0.5/0.5) with prev detected box:
					newObj.m_bbox = UTILS::mergeBBoxes(yoloMergedBox, m_objects[ind].back().m_bbox, 0.5);
				}

				newObj.m_ID = m_objects[ind].back().m_ID;

				m_objects[ind].push_back(newObj);
				matchInds.push_back(ind);
			}
		}


		// Check that all yolo detections was tracked. 
		// If not add the untracked it to track list (setROI)
		if (!trackerObjects.empty() && prevWasYolo) {
			//checkUntrackedTrkObjects(); // fill the m_TrkToRenewIDs
			for (int i = 0; i < m_objects.size(); i++)
				if (m_objects[i].back().m_detectionType == DETECT_TYPE::ML) {  // Check only fresh YOLO objects 
					int id = m_objects[i].back().m_ID;
					auto it = find_if(trackerObjects.begin(), trackerObjects.end(), [id](CObject obj) { return obj.m_ID == id; });
					if (it == trackerObjects.end()) // objet is missing in Tracker detections
						m_TrkToRenewIDs.push_back(i);
				}

			prevWasYolo = false;
		}
		// 
	}
#endif 

	int rem = pruneObjects();

	return duplicateInds;

}

/*-------------------------------------------------------------------------------
 *	Match RECT to prev RECTs in 'm_objects' list
 * Params:
 * ignoreInds - these indices was already matched in prev cycles, ignore
 * frameNumToIgnore - Ignore object that detect in this frameNum (current frame)
 *--------------------------------------------------------------------------------*/
int CDecipher::bestMatch(CObject srcObj, std::vector <int> ignoreInds, int frameNumToIgnore)
{
	std::vector <int>  bestInds;
	std::vector <float>  bestScores;

#if 0
	// DDEBUG DDEBUG
	cv::Rect targetBox = cv::Rect(708, 52, 53, 33);
	cv::Rect testArea = enlargeBbox(targetBox, cv::Size(targetBox.width * 2, targetBox.height * 2));
	if (1) // DDEBUG DDEBUG 
	{
		float overlap = OverlappingRatio_subjective(srcObj.m_bbox, testArea);
		if (overlap > 0.9)
			int debug= 10; // most new box overlapped old box
	}

#endif
	for (int i = 0; i < m_objects.size(); i++) {
		auto obj = m_objects[i].back();

		// Skip conditions:
		if (obj.m_label != srcObj.m_label)
			continue;
		if (std::find(ignoreInds.begin(), ignoreInds.end(), i) != ignoreInds.end())
			continue; // Ignore this object
		if (obj.m_frameNum == frameNumToIgnore)
			continue;

		bool Matching = false;
		float overlappedRatio;

		bool stableStatic = isStableStaticObj(obj);

		if (stableStatic) {
			// Check match for STATIC object 
			//overlappedRatio = bboxesBounding(obj.m_bbox, srcObj.m_bbox); // most new box area should overlappe old box
			overlappedRatio = OverlappingRatio(obj.m_bbox, srcObj.m_bbox); // most new box area should overlappe old box			
			Matching = (overlappedRatio > SENIOR_GOOD_STATIC_OVERLAPED_AREA);
		}
		else {
			// Check match for MOVING object 
			float OverlappedThrash = GOOD_MOVING_OVERLAPED_AREA;
			int durationFactor = min(5, m_frameNum - obj.m_frameNum);
			int maxDistance = singleStepSize(obj.m_bbox.width, stableStatic) * durationFactor; // (m_frameNum - obj.m_frameNum);

			//m_predictor.predict(obj, m_frameNum);

			overlappedRatio = OverlappingRatio(obj.m_bbox, srcObj.m_bbox); // most new box overlapped old box
			// Check (1) overlapping ratio (2) The sizes are similars (kind of) (3) the distance is reasonable 
			Matching = (overlappedRatio > OverlappedThrash) || (similarBox(obj.m_bbox, srcObj.m_bbox, 0.6) && distance(boxCenter(obj.m_bbox), boxCenter(srcObj.m_bbox)) < maxDistance);
		}

		if (Matching) {
			bestInds.push_back(i);
			bestScores.push_back(overlappedRatio);
		}
	}

	if (bestScores.empty())
		return -1;

	auto it = max_element(bestScores.begin(), bestScores.end());
	int index = it - bestScores.begin();
	return bestInds[index];
	}


/*---------------------------------------------------------------
	Update final (current) 'm_detectedObjects' from 'm_objects'
	Update motion typew (m_moving) and age of the object
 ----------------------------------------------------------------*/
int CDecipher::consolidate(int mode, std::vector <cv::Rect>  BGSEGoutput)
{
	m_detectedObjects.clear(); // TEMP clearing
	m_pruneObjIDs.clear();

	for (auto& obj : m_objects) {

		//-----------------------------
		// Set motion mode (m_moving): 
		// positive  for moving (num of moving frames), -X for X frames of static length 
		//-----------------------------


		// Set new static len :
		//-----------------------
		if (obj.size() > 1) {

			int prevInd = obj.size() - 2;
			if (isStableStaticObj(obj[prevInd]))
				// -1- Stable Static object:
				obj.back().m_moving = obj[prevInd].m_moving - 1;
			else {
				int staticLen = checkIfStatic(obj, BGSEGoutput);
				if (staticLen >= MIN_STATIC_FRAMES || (staticLen >= MIN_SHORT_STATIC_FRAMES && staticLen == obj.size()))
					// -2- Junior Static object:
					obj.back().m_moving = -staticLen; // Keep static len as NAGTIVE
				else
					// -3- Moving object:
					obj.back().m_moving = std::count_if(obj.begin(), obj.end(), [](CObject& ob) { return ob.m_moving > 0; });
			}
		}


		// Remove old object
		if (getHiddenLen(obj.back()) > GET_KEEP_HIDDEN_FRAMES(obj)) {
			// remove object later
			m_pruneObjIDs.push_back(obj.back().m_ID);
		}
		else {
			// Add object 
			obj.back().m_age = obj.size(); // update age 
			m_detectedObjects.push_back(obj.back());
			m_detectedObjects.back().m_confidence = stableConfidence(obj); // calc mean confidence of the last few frames 
		}
	}

	return m_detectedObjects.size();
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
		[frameNum](CObject obj) { return obj.m_frameNum >= frameNum && obj.m_age < MIN_STABLE_DETECTION_FRAMES; });
		//[frameNum](CObject obj) { return obj.m_frameNum >= frameNum && obj.m_age == 1; });
		
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
			[](CObject obj) { return (obj.m_label == Labels::car || obj.m_label == Labels::truck || obj.m_label == Labels::bus) &&    obj.isMoving(); });
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
				[](CObject obj) { return obj.m_label != Labels::person && obj.m_label != Labels::nonLabled && obj.isMoving(); });
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
			(m_objects[i].back().m_finalLabel != Labels::nonLabled || m_objects[i].size() >= GOOD_TRACKING_LEN))
			goodObjectsInd.push_back(i);
	}

	return goodObjectsInd;
}


std::vector <CObject> CDecipher::getLastLostObjects(int curFrameNum, int label)
{
	std::vector <CObject> lostObjects;

	for (auto obj : m_objects) {
		if ((obj.back().m_frameNum + 1) ==  curFrameNum)
			lostObjects.push_back(obj.back());
	}

	return lostObjects;
}

/*---------------------------------------------------------------------------------
 Calc confidence over time 
 calc mean confidence of the last few frames
*---------------------------------------------------------------------------------*/
float CDecipher::stableConfidence(std::vector <CObject> obj)
{
	if (obj.size() < MIN_STABLE_DETECTION_FRAMES) 
		return 0;

	float confAvg = 0;
	for (int i = obj.size() - MIN_STABLE_DETECTION_FRAMES; i < obj.size(); i++) {
		confAvg += obj[i].m_confidence;
	}

	confAvg /= (float)MIN_STABLE_DETECTION_FRAMES;

	// Give more weight to an elders objects - aerage the AVG with best confidence ever 
	if (obj.size() > MIN_STABLE_DETECTION_FRAMES * 3) { // DDEBUG CONST 
		auto it = std::max_element(obj.begin(), obj.end(), [](const CObject& a, const CObject& b) { return a.m_confidence < b.m_confidence; });
		if (1)
			confAvg = (confAvg + it->m_confidence) / 2.;
		else
			return it->m_confidence;
	}


	return confAvg;

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

#if 0
/*--------------------------------------------------------------------------------
* Four mode of static : 
* None (Move)
* Stable (long static)
* Regular 
* Short but solid (=all elements) (short static)
---------------------------------------------------------------------------------*/
STATIC_MODE  CDecipher::staticMode(std::vector <CObject> obj, std::vector <cv::Rect>  BGSEGoutput)
{
	int staticLen = checkIfStatic(obj, BGSEGoutput); // should oprimized - should moved once  to process()

	if (staticLen == 0)
		return STATIC_MODE::Move;

	if (staticLen >= MIN_STABLE_STATIC_FRAMES) // 15
				return STATIC_MODE::Stable;
	
	if (staticLen >= MIN_STATIC_FRAMES) // 8
		return STATIC_MODE::Normal;

	if (staticLen == obj.size() && staticLen >= MIN_SHORT_STATIC_FRAMES) // Short (3)  but all appearmences are the same pos (static)
		return STATIC_MODE::Normal;

}

/*--------------------------------------------------------------------------------
* Check if last <len> objects are static
 --------------------------------------------------------------------------------*/
bool CDecipher::isLastStatic(std::vector <CObject> objs, int len)
{
	if (objs.size() < len)
		return false;


	std::vector <CObject> shortObj = std::vector <CObject>(objs.end() - len, objs.end());
	if (checkIfStatic(shortObj) == len)
			return true;
	

	return false;

}
#endif 
/*----------------------------------------------------------------------
* Check if object is stable static object
* Check if CURRENT (last) bbox overlapped all others boexs in list
* if lesss than MIN_OVERLAP_RATIO is overlapped - it is NOT static
* Return : count of static frames ( 0 if moving)
-----------------------------------------------------------------------*/
int CDecipher::checkIfStatic(std::vector <CObject> objs, std::vector <cv::Rect>  BGSEGoutput)
{
	const int INSPECTED_LEN = 20;
	const float MIN_OVERLAP_RATIO = 0.65;
	int staticCount= 0;


	if (objs.size() < 2)
		return 0;

	// If BGSeg found motion within this object - not a static object
	for (auto bbox : BGSEGoutput) {
		if (OverlappingRatio(cv::Rect2f(objs.back().m_bbox), cv::Rect2f(bbox)) > 0)
			return 0;
	}

	auto curObj= objs.back();
	auto prevObj = objs[objs.size()-2];

	int overlapCount = 0;

	int start = MAX(int(objs.size() - INSPECTED_LEN), 0);
	int actualInspectLen = objs.size()- start;

	// Check current overlapped of all others boexs in list
	for (int i = start; i < objs.size()-1; i++) {
		//float overlapRatio = OverlappingRatio(cv::Rect2f(curObj.m_bbox), cv::Rect2f(objs[i].m_bbox));
		float overlapRatio = OverlappingRatio(cv::Rect2f(curObj.m_bbox), cv::Rect2f(objs[i].m_bbox));
		if (overlapRatio > GOOD_STATIC_OVERLAPED_AREA)
			overlapCount++;
	}

	float framesOverlapRatio = (float)(overlapCount + 1.) / (float)(actualInspectLen);
	if (framesOverlapRatio > MIN_OVERLAP_RATIO)
		staticCount = overlapCount + 1; // include the current object
	else
		staticCount = 0;

	return staticCount;
}



/*------------------------------------------------------------------------------
* Len to keep object hidden
* 5 static frames is enough to keep this object for KEEP_STATIC_HIDDEN_FRAMES
--------------------------------------------------------------------------------*/
int CDecipher::GET_KEEP_HIDDEN_FRAMES(std::vector <CObject> obj)
{
	if (checkIfStatic(obj) < MIN_STATIC_FRAMES_TO_KEEP_LONGER)
		return MIN(KEEP_MOVING_HIDDEN_FRAMES, obj.size());
	else
		return KEEP_STATIC_HIDDEN_FRAMES;
}



inline bool CDecipher::isLarge(std::vector <CObject> obj)
{
	return (obj.back().m_bbox.width > m_maxObjectDim.width || obj.back().m_bbox.height > m_maxObjectDim.height);
}

/*----------------------------------------------------------------
 * Prune objects that are too old (not detected for a long time)
 -----------------------------------------------------------------*/
int CDecipher::pruneObjects()
{
	if (m_pruneObjIDs.empty())
		return 0;

	int orgSize = m_objects.size();

	std::vector <int>   prunedInds; // object index (!m_ID)

	// convert  ObjID to vector index:
	for (auto i : m_pruneObjIDs) {
		auto it = find_if(m_objects.begin(), m_objects.end(), [i](std::vector <CObject>  obj) { return obj.back().m_ID == i; });
		if (it != m_objects.end())
			prunedInds.push_back(std::distance(m_objects.begin(), it));
	}

	std::sort(prunedInds.begin(), prunedInds.end(), greater<int>()); // reverse order 
	for (auto ind : prunedInds) {
		m_objects.erase(m_objects.begin() + ind);
	}

	return orgSize - m_objects.size();
}


int CDecipher::pruneObjects(int objID)
{
	int orgSize = m_objects.size();

	std::vector <int>   prunedInds; // object index (!m_ID)

	// convert  ObjID to vector index:
		auto it = find_if(m_objects.begin(), m_objects.end(), [objID](std::vector <CObject>  obj) { return obj.back().m_ID == objID; });
		if (it != m_objects.end()) {
			int ind = std::distance(m_objects.begin(), it);
			m_objects.erase(m_objects.begin() + ind);
			return 1;
			//prunedInds.push_back(std::distance(m_objects.begin(), it));
		}

		return 0;
}


// return number of person on board (including hidden objects)
int CDecipher::numberOfPersonsOnBoard()
{
	return getPersonObjects(m_frameNum).size();
}

/*--------------------------------------------------------------------------------
* Selects all detected object
* The AI DLL returns ALL detected objects, regardless the maxAllowed parameter
* The filtered Alert object are stores within the m_alertObjects;
* This function also calc the 'm_alertObjects' when ever called to 'getDetectedbjects'
----------------------------------------------------------------------------------*/
std::vector <CObject> CDecipher::getStableObjects(float scale, cv::Mat *frameImg)
{
	bool OnlyStaticObject = true; 
	m_alertObjects.clear();

	std::vector <CObject> scaledDetectedObjects, detectedObjects;

	// Scale back to origin dimensions is required
	for (auto obj : m_detectedObjects) {

		int maxHiddenFrame = obj.isMoving() ? MAX_MOVING_SIREN_HIDDEN_FRAMES : MAX_STATIC_SIREN_HIDDEN_FRAMES;

		if (obj.m_confidence >= GET_MIN_YOLO_CONFIDENCE(obj) && getHiddenLen(obj) <= maxHiddenFrame) { // Allow only fresh detected objects
			scaledDetectedObjects.push_back(obj);
			scaledDetectedObjects.back().m_bbox = UTILS::scaleBBox(obj.m_bbox, scale);
		}
	}

	for (auto alert : m_alerts) {
		std::vector <CObject> detectionSinglePloy =  alert.selectObjects(scaledDetectedObjects);		
		
		// Filter out  'suspectedAsFalse' objects
		for (auto obj : detectionSinglePloy) {
			if (!suspectedAsFalse(obj, Labels(alert.m_label), frameImg))
				detectedObjects.push_back(obj);
			//else pruneObjects(obj.m_ID); // DDEBUG WRONG !
		}

		if (detectedObjects.size() > alert.m_maxAllowed)
			m_alertObjects.insert(m_alertObjects.end(), detectedObjects.begin(), detectedObjects.end()); // gather all camera's alerts into one vector
	}


	// DDEBUG DDEBUG SAVE DETECTED IMAGE 
	if (0)
	if (detectedObjects.size() > 0 && isIn(cv::Point(175,220), detectedObjects[0].m_bbox)) {
		cv::Mat debugImg = *frameImg;
		cv::rectangle(debugImg, detectedObjects[0].m_bbox, cv::Scalar(0,0,255), 3);
		std::string fname = "C:/temp/detected_" +  std::to_string(m_camID) +".jpg";
		cv::imwrite(fname, debugImg);
	}

	return detectedObjects;
}


std::vector <CObject> CDecipher::removeOverlappedObjects(std::vector <CObject> detectionSinglePloy)
{
	std::vector <CObject>  finalObjects;


	return finalObjects;
}

/*------------------------------------------------------------------------------------------------
* False Laram section:
 ------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------
* Yolo has false alarams, guess if this is the case by:
* (1) Box touch the frame edges
* (2) Contrast is low
* [ (3) Not excelence confidence score ]
-----------------------------------------------------------------------------*/
bool CDecipher::suspectedAsFalse(CObject obj, Labels alertLabel, cv::Mat* frameImg)
{

	if (frameImg == nullptr)
		return false;

	if (isMatchFalseList(obj, *frameImg))
		return true;

	// Dont check high score detection
	if (obj.m_confidence > HIGH_YOLO_CONFIDENCE)
		return false;

	// Fail small objects (diff between width & height in Person detection
	//---------------------------------------------------------------------
	
	// Dimension for non person objects
	int Small_Box_H = 30;
	int Small_Box_W = Small_Box_H;
	// Dimension for person objects
	if (obj.m_label == Labels::person) {
		if (1) // DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG Allow small person objects
			Small_Box_H = 15;
		else
			Small_Box_H = 30;

			Small_Box_W = int((float)Small_Box_H / 3.);
	}

	// Remove small & young objects
	if (obj.m_age < 5) 
		if (obj.m_bbox.width < Small_Box_W || obj.m_bbox.height < Small_Box_H) {
			LOGGER::log(DLEVEL::ERROR2, "Detection suspected as FALSE (SIZE) , in frame = " + std::to_string(m_frameNum));
			return true;
		}
	bool touchEdge = (obj.m_bbox.x == 0 || obj.m_bbox.y == 0 || obj.m_bbox.br().x == frameImg->size().width - 1 || obj.m_bbox.br().y == frameImg->size().height - 1);

	// Ignore if  "touchEdge" and "lowCOntrast"
	//if (touchEdge) // DDEBUG REMOVE TEST 
		{

		float ratio = float(obj.m_bbox.width) / float(obj.m_bbox.height);
		bool fatDimension = (ratio > MIN_BOX_RATION_FOR_PERSON && ratio < (1. / MIN_BOX_RATION_FOR_PERSON));  // person usually has a nerrow dimensions

		//if (fatDimension) // DDEBUG REMOVE TEST 
		{
			double minVal;
			double maxVal;
			int histSize = 256;
			float range[] = { 0, 256 }; //the upper boundary is exclusive
			const float* histRange[] = { range };
			bool uniform = true, accumulate = false;


			cv::Mat grayImg;
			if (frameImg->channels() > 1)
				cv::cvtColor(*frameImg, grayImg, cv::COLOR_BGR2GRAY);
			else
				grayImg = *frameImg;


			cv::Scalar mean, stddev;
			cv::Scalar mean_, stddev_;

			cv::meanStdDev(grayImg(obj.m_bbox), mean, stddev);


			bool lowCOntrast = (stddev.val[0] < MIN_STDDEV_FOR_BOX); // Low variation gray levels in box

			if (lowCOntrast) {
				LOGGER::log(DLEVEL::ERROR2, "Detection suspected as FALSE (CONTRAST), in frame = " + std::to_string(m_frameNum));
				return true;
			}
		}
	}

	return false;

}


bool isTemplateMatch(cv::Mat &img, cv::Mat templ, float thresh)
{
	cv::Mat result;

	if (templ.empty())
		return false;

	cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED); // TM_SQDIFF
	double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	return maxVal > thresh;
}

/*--------------------------------------------------------------------------------
	Check if object matchs one of the 'false' in list
--------------------------------------------------------------------------------*/
bool CDecipher::isMatchFalseList(CObject obj, cv::Mat& frameImg)
{
	float thres = 0.7;
		

	for (auto templ : m_falseTamplates) {
		if (templ.m_label != obj.m_label)
			continue;
		//if (templ.m_camID != obj.m_ID)   continue;
		// Check obj dimension vs template dimension
		cv::Mat objImg;

		objImg = frameImg(obj.m_bbox);

		// Switch images due to size issues (template must be smaller than src image)
		if (obj.m_bbox.width < templ.m_img.cols && obj.m_bbox.height < templ.m_img.rows)
			if (isTemplateMatch(templ.m_img, objImg, thres))
				return true;

		// Enlarge detection box if neccessary 
		if (obj.m_bbox.width < templ.m_img.cols || obj.m_bbox.height < templ.m_img.rows) {
			cv::Rect largerBbox = enlargeBbox(obj.m_bbox, templ.m_img.size(), frameImg.size());
			objImg = frameImg(largerBbox);
		}

		if (isTemplateMatch(objImg, templ.m_img, thres))
			return true;
	}

	return false;
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



std::vector <CObject> CDecipher::getTrkObjsToRenew()
{
	std::vector <CObject> objsToRenew;
	for (int id : m_TrkToRenewIDs)
		objsToRenew.push_back(m_objects[id].back());
	
	return objsToRenew;

}


/*------------------------------------------------------------------------------------------------------------
* Read false images from (const) folder 
* File name convension is:
*   false_camid_XX.jpg\png\etc.
 ------------------------------------------------------------------------------------------------------------*/
int CDecipher::readFalseList()
{
	const std::filesystem::path FalseFolder{ R"(C:\Program Files\Bauotech\dll\algo\data\)" };

	if (!UTILS::isFolderExist(R"(C:\Program Files\Bauotech\dll\algo\data\)"))
		return 0;

	for (auto const& dir_entry : std::filesystem::directory_iterator{ FalseFolder }) {
		int camID = -1;
		std::string sPath = dir_entry.path().generic_string(); // convert to string 

		// get cam ID:
		//----------------------------------------------------------------
		std::string sFName = dir_entry.path().filename().generic_string();
		std::string sExt = dir_entry.path().extension().generic_string();
		
		auto ptr = sFName.find("camid_");
		if (ptr != std::string::npos) {
			auto dot = sFName.find(".");
			
			int IDStrLen = dot - ptr - 6;
			std::string sCamid = sFName.substr(ptr + 6, IDStrLen);
			camID = atoi(sCamid.c_str());
		}
		//----------------------------------------------------------------

		// Add false-image to the list
		if (camID == m_camID)  {
			cv::Mat falseImg = cv::imread(sPath);
			Labels alertLabel = Labels::person; // DDEBUG DDEBUG DDEBUG CONST 
			m_falseTamplates.push_back({ camID, falseImg, alertLabel });
		}
	}

	return m_falseTamplates.size();

}