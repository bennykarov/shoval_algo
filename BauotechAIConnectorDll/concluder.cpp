#include <stdio.h>
#include <algorithm>
#include <vector>
#include <numeric>

#include "opencv2/opencv.hpp"


#include "CObject.hpp"
#include "yolo/yolo.hpp"

#include "config.hpp"
#include "logger.hpp"
#include "utils.hpp"
#include "concluder.hpp"



using namespace CONCLUDER_CONSTANTS;

void CDecipher::init(cv::Size imgDim, int debugLevel) { m_imgDim  = imgDim;  m_debugLevel = debugLevel; 	m_active = true; }

/*--------------------------------------------------------------------------
// UTILITIES :
--------------------------------------------------------------------------*/

float GET_MIN_YOLO_CONFIDENCE(Labels label, bool  MovingObj)
{	
	float conf = 0;
	switch (label)
	{
	case Labels::person:
		conf = MovingObj == 0 ? 0.5 : 0.2;
		break;
	default:
		conf = MovingObj == 0 ? 0.5 : 0.4;
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

#if 0
		// Filter specilas cases: move to suspected
		// Small object must have HIGHER Confidence 
		const int Small_Box_Dim = 20;
		if (Yobj.box.area() <= Small_Box_Dim * Small_Box_Dim && Yobj.confidence < YOLO_CONFIDENCE_THRESHOLD_SMALL_DIM)
			continue;
#endif 
		//if (Yobj.box.width <= Small_Box_Dim && Yobj.box.height <= Small_Box_Dim)   int debug = 10;

		float overLappedTreshold = 0.3; // DDEBUG CONST
		int ind = bestMatch(Yobj.box, overLappedTreshold, matchInds);

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

	//------------------------------------------------------------------------------------------
	// Add Tracker objects 
	//------------------------------------------------------------------------------------------

	// (Tracker-A) First ignore tracker boxes  that overlapped with yolo (preferred YOLO)
	//------------------------------------------------------------------------------------------
	std::vector <int> duplicateInds;
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

		int ind = bestMatch(obj.m_bbox, 0.01, matchInds); // 'frameNum' -> ignore objects currently detected  by YOLO 
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

	int rem = pruneObjects();

	return duplicateInds;

}

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


		auto obj = m_objects[i].back();
		{
			float overlappingRatio = bboxesBounding(obj.m_bbox, box); // most new box overlapped old box
			int maxDistance = MIN(100, MAX_MOTION_PER_FRAME * (m_frameNum - obj.m_frameNum));
			// Check (1) overlapping ratio (2) The sizes are similars (kind of) (3) the distance is reasonable 
			bool Matching;
			// Different condition for moving and static objects:
			// In case of static object - dont match after start moving - better treats as a new object 
			// This done to avoid "jumping"  from one parking car to another 

			if (obj.m_moving > 0)
				Matching = (overlappingRatio > OverlappedThrash) || (similarBox(obj.m_bbox, box, 0.6 * 0.6) && distance(obj.m_bbox, box) < maxDistance);
			else // static object 
				Matching = (overlappingRatio > 0.8 && distance(obj.m_bbox, box) < 10);

			//if (overlappingRatio > OverlappedThrash ||  (similarBox(obj.m_bbox, box, 0.6 * 0.6) && distance(obj.m_bbox, box) < maxDistance))
			if (Matching) {
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



/*---------------------------------------------------------------
	Update final (current) 'm_detectedObjects' from 'm_objects'
 ----------------------------------------------------------------*/
int CDecipher::track(int mode)
{
	m_detectedObjects.clear(); // TEMP clearing
	m_pruneObjIDs.clear();

	for (auto& obj : m_objects) {

		// Remove old object
		if 	(getHiddenLen(obj.back()) > GET_KEEP_HIDDEN_FRAMES(obj.back().m_moving)) {
			// remove object later
			m_pruneObjIDs.push_back(obj.back().m_ID);
		}
		else {
			// Add object 
			obj.back().m_age = obj.size(); // update age 
			m_detectedObjects.push_back(obj.back());
			m_detectedObjects.back().m_confidence = stableConfidence(obj); // calc mean confidence of the last few frames 

			// Set m_moving  : 1 for moving, -X for X frames of static length 
			if (isStatic(obj) == 0)
				obj.back().m_moving = 1;
			else {
				if (obj.size() > 1)
					obj.back().m_moving = obj[obj.size() - 2].m_moving - 1; // increase (negative) count
				else
					obj.back().m_moving = -1;
			}

			m_detectedObjects.back().m_moving = obj.back().m_moving;
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

	// Give more weight to an elders objects - provide the best confidence of last detections 
	if (obj.size() > MIN_STABLE_DETECTION_FRAMES * 3) { // DDEBUG CONST 
		auto it = std::max_element(obj.begin(),obj.end(), [](const CObject& a, const CObject& b) { return a.m_confidence < b.m_confidence; });
		return it->m_confidence;
	}


	float confAvg = 0;
	for (int i = obj.size() - MIN_STABLE_DETECTION_FRAMES; i < obj.size(); i++) {
		confAvg += obj[i].m_confidence;
	}

	confAvg /= (float)MIN_STABLE_DETECTION_FRAMES;
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


/*------------------------------------------------------------------------------------------
* Check if object is static :
* return the len (num of frames) of the static object - up to max 'INSPECTED_LEN' len
* check :
	(1) BBox overlapping
	(2) Box position (center)
	(3) Detection confidence 
 *------------------------------------------------------------------------------------------*/
int CDecipher::isStatic(std::vector <CObject> obj)
{
	int MinLen = 3;
	int MinStatisticsPointsLen = 3;  // excluding outliers 
	int INSPECTED_SECONDS = 2;
	int INSPECTED_LEN = INSPECTED_SECONDS * CONSTANTS::FPS; // in frames
	float MIN_OVERLAPPING_RATIO = 0.8;
	float DIST_TO_BOX_RATIO = 1./20.; // Motion 'dist' proprional to obj dim (box)

	

	// Default is moving  here 
	if (obj.size() < MinLen)
		return 0;

	if (obj.back().m_ID == 10)
		int debug = 10;

	// 'Structure' keeps indices, centers and areas of detection vecs
	std::vector <int> indices;
	std::vector <cv::Point2f> centers;
	std::vector <float>  areas;


	int start = MAX(int(obj.size() - INSPECTED_LEN), 0);
	//int end = obj.size();

	for (int i = start; i <  obj.size(); i++) {
		cv::Point center = (obj[i].m_bbox.br() + obj[i].m_bbox.tl()) * 0.5;
		indices.push_back(i);
		centers.push_back(center);
		areas.push_back(obj[i].m_bbox.area());
	}


	// Find Median of centers 
	//---------------------------
	cv::Point2f massCenter;

	auto centersToSort = centers; // dont ruin the original indices 
	std::sort(centersToSort.begin(), centersToSort.end(), [](const cv::Point2f p1, const cv::Point2f p2) -> bool { return p1.x < p2.x; });
	massCenter.x = centersToSort[int(centersToSort.size() / 2)].x;
	std::sort(centersToSort.begin(), centersToSort.end(), [](const cv::Point2f p1, const cv::Point2f p2) -> bool { return p1.y < p2.y; });
	massCenter.y = centersToSort[int(centersToSort.size() / 2)].y;

	// MIN_STATIC_DIST is the distance from the Median location,
	// When obj.size > 10, we allowed large step 
	// Mark outliers (ind = -1)
	//-----------------------------------------------------------
	int MIN_STATIC_DIST = int(float(obj.back().m_bbox.size().width) * DIST_TO_BOX_RATIO);
	if (obj.size() > 10)
		MIN_STATIC_DIST *= 2;
	for (int i = 0; i < centers.size();i++) {
		float debug = distance(massCenter, centers[i]);
		if (distance(massCenter, centers[i]) > (float)MIN_STATIC_DIST)
			indices[i] = -1;// mark as outlier
	}

	int staticCount = std::count_if(indices.begin(), indices.end(), [](int i) { return i >= 0; });
	// Quit if too few static points 
	if (staticCount < MinStatisticsPointsLen)
		return 0;
	//----------------------
	// Check Areas diff
	//----------------------

	float areaMedian = findMedian(areas);
 
	for (int i = 0; i < indices.size(); i++) {
		if (indices[i] >= 0)
			if (UTILS::ratio(areaMedian, areas[i]) <  (float)MIN_OVERLAPPING_RATIO)
				indices[i] = -1;// mark as bad
	}


	staticCount = std::count_if(indices.begin(), indices.end(), [](int i) { return i >= 0; });
	// Quit if too few static points 
	if (staticCount < MinStatisticsPointsLen)
		return 0;

	// -1- check majority of static points (3/4 at least)
	if (staticCount < int((float)indices.size() * 0.75))
		return 0;

	auto it = std::find_if(indices.begin(), indices.end(), [](int i) { return 1 > -1;  });
	int firstInd = std::distance(indices.begin(), it);
	auto it2 = std::find_if(indices.rbegin(), indices.rend(), [](int i) { return 1 > -1;  });
	int lastInd = std::distance(it2, indices.rend()-1);

	// -2- check statics points spred nicely along the inspected area (period)
	int dist = lastInd - firstInd;
	if (dist < int((float)indices.size() / 2.))
		return 0;
	

	// Retun number of "good" points (excluded outliers)
	return 	staticCount;

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
		int MAX_HIDDEN_FRAME = obj.m_moving > 0 ? MAX_SIREN_HIDDEN_FRAMES : MAX_SIREN_HIDDEN_FRAMES * 10;
		if (obj.m_confidence >= GET_MIN_YOLO_CONFIDENCE((Labels)obj.m_label, obj.m_moving) && getHiddenLen(obj) <= MAX_HIDDEN_FRAME) { // Allow only fresh detected objects
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
		}

		if (detectedObjects.size() > alert.m_maxAllowed)
			m_alertObjects.insert(m_alertObjects.end(), detectedObjects.begin(), detectedObjects.end()); // gather all camera's alerts into one vector
	}


	return detectedObjects;
}



/*-----------------------------------------------------------------------------
* Yolo has false alarams, guess if this is the case by:
* (1) Box touch the frame edges
* (2) Contrast is low
* [ (3) Not excelence confidence score ]
-----------------------------------------------------------------------------*/
bool CDecipher::suspectedAsFalse(CObject obj, Labels alertLabel, cv::Mat* frameImg)
{
	if (alertLabel != Labels::person)
		return false;

	if (frameImg == nullptr)
		return false;


	// Small static object must have HIGHER Confidence 
	const int Small_Box_Dim = 20;
	if (obj.m_moving == 0 && obj.m_bbox.area() <= Small_Box_Dim * Small_Box_Dim && obj.m_confidence < YOLO_HIGH_CONFIDENCE)
		return true;


	// Ignore too small object
	if (1) { // DDEBUG DDEBUG REMOVED this for drone test
		int maxDim = max(obj.m_bbox.width, obj.m_bbox.height);
		if (maxDim < MIN_OBJECT_SIDE_SIZE)
			return true;
	}

	// Dont check high score detection
	if (obj.m_confidence > HIGH_YOLO_CONFIDENCE)
		return false;


	bool touchEdge = (obj.m_bbox.x == 0 || obj.m_bbox.y == 0 || obj.m_bbox.br().x == frameImg->size().width - 1 || obj.m_bbox.br().y == frameImg->size().height - 1);

	// Ignore if  "touchEdge" and "lowCOntrast"
	if (touchEdge) {
		// box touch one of the the edge
		float ratio = float(obj.m_bbox.width) / float(obj.m_bbox.height);
		bool fatDimension = (ratio > MIN_BOX_RATION_FOR_PERSON && ratio < (1. / MIN_BOX_RATION_FOR_PERSON));  // person usually has a nerrow dimensions

		if (fatDimension) {
			double minVal;
			double maxVal;
			//cv::Point minLoc;
			//cv::Point maxLoc;
			int histSize = 256;
			float range[] = { 0, 256 }; //the upper boundary is exclusive
			const float* histRange[] = { range };
			bool uniform = true, accumulate = false;
			cv::Mat hist;

			cv::Mat grayImg;
			if (frameImg->channels() > 1)
				cv::cvtColor(*frameImg, grayImg, cv::COLOR_BGR2GRAY);
			else
				grayImg = *frameImg;


			cv::Scalar mean, stddev;
			cv::Scalar mean_, stddev_;

			//cv::meanStdDev(grayImg, mean_, stddev_);
			cv::meanStdDev(grayImg(obj.m_bbox), mean, stddev);


			bool lowCOntrast = (stddev.val[0] < MIN_STDDEV_FOR_BOX); // Low variation gray levels in box

			if (lowCOntrast) {
				LOGGER::log(DLEVEL::ERROR2, "Person detection suspected as FALSE , in frame = " + std::to_string(m_frameNum));
				return true;
			}
		}
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

