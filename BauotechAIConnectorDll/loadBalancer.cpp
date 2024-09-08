#include <thread> 
#include <functional>  // std::ref
#include <future>    // std::promise
#include <condition_variable> 
#include <chrono>
#include <iostream> 
#include <mutex> 
#include <queue> 
#include <stdlib.h>     /* srand, rand */
#include <numeric>

#include "opencv2/opencv.hpp"

#include "AlgoApi.h" // for MAX_VIDEO const
#include "logger.hpp"
#include "loadBalancer.hpp"
#include "testBalancer.hpp" // DDEBUG monitor cams requests queue

#include "database.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

using namespace std::chrono_literals;


std::mutex qeue_mutex;

// sync results
static std::queue<CCycle>         g_resQueue;
static std::mutex              g_resMtx;
static std::condition_variable g_resCondV;

CDashboard m_dashboard;




#ifdef USE_LOAD_BALANCER



#ifdef _DEBUG
const int beatTick = 99;
#else
const int beatTick = 33;
#endif 


void CLoadBalaner::cameraCounter(int cams)
{ 
	m_camerasNum += cams;  
	tuneQueueParams(m_camerasNum);



} // Add or remove cameras counter



//--------------------------------------------------------
// thread to set priority according to detection results 
// recieved from algo process
//--------------------------------------------------------
void CLoadBalaner::priorityTH()
{
	CTimer timer;

	int processCounter = m_resourceNum-1; // for system kickoff at start
	int batchCounter = 0; // for print statistics 
	m_cycleCounter = 0;
	int statisticsModulu = (m_debugLevel > DLEVEL::ERROR2 ? 30 * 40 : 30 * 100);
	statisticsModulu = 400; // DDEBUG DDEBUG 
	//statisticsModulu = 50; // DDEBUG DDEBUG 
	timer.start();


	while (!m_terminate) {

		if (!priorityUpdate()) // No data (image) recieved from server
			continue;

		// Batch handling:
		processCounter++;
		if (processCounter == m_resourceNum) { // == timeToUpdateBatchList()
			m_cycleCounter++;
			batchCounter++;
			prepareNextBatch();
			processCounter = 0;

			if (0) {
				float elapsed = timer.sample();
				LOGGER::log(DLEVEL::INFO2, " batch elapsed  time = " + std::to_string(elapsed));
			}
			
		}


		if (m_printStatistics && batchCounter > statisticsModulu) { // DDEBUG PRINTING 

			LOGGER::log(DLEVEL::ERROR1, ">> Number of Cameras = " + std::to_string(m_camerasNum) + " Batch size = " + std::to_string(m_resourceNum));

			StatisticsInfo res = printStatistics(m_logBatchQueue, m_priorityQueue.getCamList(), m_resourceNum, m_actualTopPriority);
			m_logBatchQueue.clear();
			batchCounter = 0;

			m_dashboard.update(m_priorityQueue.getCamList(), res, m_actualTopPriority);
			m_dashboard.show();
		}
			

	}

}

/*-----------------------------------------------------------------------------
* resQueue handling : 
  1. push, 2. notify (called bY algoProcess thread)
* 3. priorityUpdate() runs after m_resource inputs 
* All three functions are protected by the g_resMtx mutex
-----------------------------------------------------------------------------*/
void CLoadBalaner::ResQueuePush(CCycle info)
{
	const std::lock_guard<std::mutex> lock(g_resMtx);
	g_resQueue.push(info);
}

void CLoadBalaner::ResQueueNotify()
{
	//std::unique_lock<std::mutex> lock(g_resMtx);
	g_resCondV.notify_one();
}

//--------------------------------------------------------
// main function called by priorityTH( thread )
// This function runs right after algoProcess camera completed 
// (Trigger transfred via condition var)
// Its calcs a new preiority according to camera results 
//--------------------------------------------------------
bool CLoadBalaner::priorityUpdate()
{
	auto dataTimeOut = 25ms; // DDEBUG CONST 

	bool balancerLogActive = true;
	std::unique_lock<std::mutex> lock(g_resMtx);

	//g_resCondV.wait(lock, [] { return !g_resQueue.empty(); });
	bool dataExist = g_resCondV.wait_for(lock, dataTimeOut, [] { return !g_resQueue.empty(); });
	if (!dataExist) {
		//LOGGER::log(DLEVEL::WARNING2, "Waiting too long for camera (Res Queue is empty) !\n");
		return false;
	}


	CCycle info = g_resQueue.front();
	g_resQueue.pop();  // release queue element
	m_resourceBouncer.release();

	m_camsProcessed.push_back(info.camID); // keep cams ahd been processed 

	// Set new priority 
	info.activeCamera = m_camType[info.camID] == CAMERA_TYPE::Active ? 1 : 0;

	//if ((info.motion + info.detections + info.alerts + info.activeCamera) > 0) { 		

	info.priority = calcPriority(info.motion, info.detections, info.alerts, info.activeCamera);

	if (info.priority > 0)
		int debug = 10; // DDEBUG 

	m_priorityQueue.set(info.camID, info.priority);
	
	m_logBatchQueue.push_back(info); // DDEBUG (low) : keep for monitoring 
	
	lock.unlock();  // Release the lock before processing the data to allow other threads to access the queue


	if (m_debugLevel > 3) {
		info.priority = m_camPriority_Debug[info.camID];
		std::cout << "cam " << info.camID << " (P=" << info.priority << ", A=" << info.activeCamera << ") ";
		if (info.detections > 0)
			std::cout << "; detection=" << info.detections;
		std::cout << "\n";
	}

	return true;
}




/*------------------------------------------------------------------------------------
* Read the top '10' of the queue
* Send them to Server (cameras streamer)  via callback 
* Update the priority constants (3d, etc.)
 ------------------------------------------------------------------------------------*/
void CLoadBalaner::prepareNextBatch()
{
	static int debugCounter = 0;
	// first increase un-handled cams prior:
	m_priorityQueue.beat(m_beatStep);

	if (m_debugLevel > DLEVEL::ALL)
		LOGGER::log(DLEVEL::INFO2, "Beat()");
	
	//m_priorityQueue.beatExclusive(m_cameraBatchList);
	// get next top priority cameras:			
	auto topQueue = m_priorityQueue.getTop(m_resourceNum);

	m_cameraBatchList.clear();
	//retrieve only the cams list 
	std::transform(std::cbegin(topQueue), std::cend(topQueue),
		std::back_inserter(m_cameraBatchList), [](const auto& data) { return data.key;});

	if (1) {// DDEBUG CHECK 
		const auto duplicate = std::adjacent_find(m_cameraBatchList.begin(), m_cameraBatchList.end());

		if (duplicate != m_cameraBatchList.end())
			LOGGER::log(DLEVEL::ERROR2, std::string("Duplicate element = " + std::to_string(*duplicate)));
	}


	// Send batch info to server:
	//if (m_cameraBatchList_prev.empty() || !equal(m_cameraBatchList.begin(), m_cameraBatchList.end(), m_cameraBatchList_prev.begin())) {
	if (m_bachCallback != nullptr) {
		uint32_t cameras[100];
		std::copy(m_cameraBatchList.begin(), m_cameraBatchList.end(), cameras);
		m_bachCallback(cameras, (int)m_cameraBatchList.size());

		updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.

	}


	m_camsProcessed.clear(); 

	if (true) { // DDEBUG
		//int skip = int(30 / 4 * 50);
		int skip = 1; // DDEBUG 
		if (++debugCounter % skip == 0) {
			if (m_debugLevel > DLEVEL::INFO2) {
				auto Queue = m_priorityQueue.status();
				std::string fullQueue = "P_QUEUE : ";
				for (auto cam : Queue)
					fullQueue.append("(" + std::to_string(cam.key) + "," + std::to_string(cam.priority) + ") ");
				LOGGER::log(DLEVEL::INFO2, fullQueue);
			}
		
			std::string batchListStr = "LB camera list request: ( ";
			for (auto cam : m_cameraBatchList) {
				batchListStr.append(std::to_string(cam));
				batchListStr.append(",");
			}
			batchListStr.append(" )");
			LOGGER::log(DLEVEL::INFO1, batchListStr);

			debugCounter = 0;
		}
	}

}


/*-------------------------------------------------------------------
* Init resource size (batchNum)
* init() return the debugLevel (for algoAPI)
-------------------------------------------------------------------*/
int CLoadBalaner::init(LB_SCHEME scheme)
{
	m_active = true;

	m_scheme = scheme;

	// setup resourceNum
	m_resourcesRange.push_back(CResourceRange(0, 10, -1));
	m_resourcesRange.push_back(CResourceRange(11, 90, 10));
	m_resourcesRange.push_back(CResourceRange(91, 150, 20));
	m_resourcesRange.push_back(CResourceRange(151, 250, 30));

	Config params;
	params.GPUBatchSize = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
	FILE_UTILS::readConfigFile(params);
	m_bestResourceNum = params.GPUBatchSize;
	//tuneQueueParams(m_camerasNum);

	//m_priorities.init()


	m_debugLevel = params.debugLevel_LB;


	m_resourceNum = params.GPUBatchSize; // Calc number of threads can run on this PC simultaneously  
	//m_resourceNum = calcPCResource(); // Calc number of threads can run on this PC simultaneously  
	//m_resourceNum = 100; // DDEBUG DDEBUG TEST !!
	
	//m_logBatchQueue.set_capacity(30*m_resourceNum*10); // DDEBUG CONST 
	m_logBatchQueue.set_capacity(30*100*10); // DDEBUG CONST 

	m_PrioirityTh = std::thread(&CLoadBalaner::priorityTH, this);

	m_camType.assign(MAX_VIDEOS, CAMERA_TYPE::Normal);
	m_camPriority_Debug.assign(MAX_VIDEOS, -1);


	bool DRAW_DASH_BOARD = false;
	if (DRAW_DASH_BOARD)
		m_dashboard.init(params.GPUBatchSize, m_resourceNum);


	return m_debugLevel;
}



/*------------------------------------------------------------------------------------------
* Set priority according to camera online parameters 
* Schem 1:
* 5 for activeCam + detection\alert
* 4 for activeCam + motion
* 3 for activeCam with nothing
* 4 for not-active + detection\alert
* 0 for none active + nothing
 ------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V1(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0;

	// First set priority from 0 to 5:
	//-----------------------------------



	if (activeCamera == 0) {
		if (alert > 0 )
			priority = 4;
		else if (detections > 0)
			priority = 4;
		else 
			priority = 0;
	}
	else {
		// Active Camera
		int topPrior = m_priorityQueue.top().priority;
		// camera is an observed cam
		if (detections > 0 || alert > 0)
			priority = 5;
		else if (motion > 0)
			priority = 4;
		else // no motion 
			priority = 3;
	}

	//int queuePriority = convertToQueuePriority_V1(priority);
	int queuePriority = convertToQueuePriority(priority);

	return queuePriority;
}

/*----------------------------------------------------------------------------------------------------
* Scheme 2:
* High priority to Active-Camera and cams with detections 
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V2(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0;

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 4;
		else if (detections > 0)
			priority = 4;
		else
			priority = 2;
	}
	else {
		// Active Camera
		if (detections > 0 || alert > 0)
			priority = 5;
		else if (motion > 0)
			priority = 4;
		else // no motion 
			priority = 3;
	}

	int queuePriority = convertToQueuePriority(priority);

	return queuePriority;
}

/*----------------------------------------------------------------------------------------------------
* Scheme 3:
* High priority to Active-Camera and cams WITHOUT  detections
* Once detected, camera get lower priority, in order to give priority to a NEW objects that apears on scene
* We prefered to delay the releasing of detected Detected camera rather to miss \ delay a new detection 
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V3(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0;
	motion = 1; // this scheme ignores motion flag !!!! DDEBUG 

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 2;
		else if (detections > 0)
			priority = 2;
		else
			priority = 4;
	}
	else {
		// Active Camera
		if (detections > 0 || alert > 0)
			priority = 4;
		else 
			priority = 5;
	}

	int queuePriority = convertToQueuePriority(priority);

	return queuePriority;
}

/*----------------------------------------------------------------------------------------------------
* Scheme 30:
* No-detection priority  -> 2
* With- Detection priority  -> 0
* Active cam priority  -> 2
* We prefered to delay the releasing of detected Detected camera rather to miss \ delay a new detection
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V30(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 0;
		else if (detections > 0)
			priority = 0;
		else
			priority = 2;
	}
	else {
		// Active Camera
		priority = 2;
	}

	int queuePriority = convertToQueuePriority_simple(priority);

	return queuePriority;
}


/*----------------------------------------------------------------------------------------------------
* Scheme 31:
* No-0detection priority  -> 1
* With- Detection priority  -> 0
* Active cam priority  -> 2
* We prefered to delay the releasing of detected Detected camera rather to miss \ delay a new detection
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V31(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 0;
		else if (detections > 0)
			priority = 0;
		else
			priority = 1;
	}
	else {
		// Active Camera
		priority = 2;
	}

	int queuePriority = convertToQueuePriority_simple(priority);

	return queuePriority;
}


/*----------------------------------------------------------------------------------------------------
* Scheme 0:
* All cams born equal (except ActiveCam)
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V0(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) 
			priority = 0;
		else
			priority = 3;

	return m_priorities.get(priority);
}



/*----------------------------------------------------------------------------------------------------
* Scheme 200:
* 4 levels scheme (0..3)
* Prefer detection's cams : mid gain 
* No-detection priority  -> 0
* With- Detection priority  -> 2
* Active cam priority  -> 3
* We prefered to delay the releasing of detected Detected camera rather to miss \ delay a new detection
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V200(int motion, int detections, int alerts, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alerts > 0)
			priority = 2;
		//else if (detections > 0)  priority = 2;
		else
			priority = 0;
	}
	else {
		// Active Camera
		priority = 3;
	}

	return m_priorities.get(priority);
}


/*----------------------------------------------------------------------------------------------------
* Scheme 201:
* Prefer detection's cams : low gain
* 4 levels scheme (0..3)
* No-detection priority  -> 0
* With- Detection priority  -> 2
* Active cam priority  -> 3
* We prefered to delay the releasing of detected Detected camera rather to miss \ delay a new detection
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V201(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 1;
		//else if (detections > 0)  priority = 1;
		else
			priority = 0;
	}
	else {
		// Active Camera
		priority = 3;
	}

	return m_priorities.get(priority);
}

/*----------------------------------------------------------------------------------------------------
* Scheme 300:
* Prefer none-detection's cams : mid gain
* 4 levels scheme (0..3)
* No-detection priority  -> 1
* With- Detection priority  -> 0
* Active cam priority  -> 2
* We prefered to delay the releasing of detected Detected camera rather to miss \ delay a new detection
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V300(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 0;
		//else if (detections > 0)  priority = 0;
		else
			priority = 2;
	}
	else {
		// Active Camera
		priority = 3;
	}

	return m_priorities.get(priority);
}


/*----------------------------------------------------------------------------------------------------
* Scheme 301:
* * Prefer none-detection's cams : low gain
* 4 levels scheme (0..3)
* No-detection priority  -> 1
* With- Detection priority  -> 0
* Active cam priority  -> 2
* We prefered to delay the releasing of detected Detected camera rather to miss \ delay a new detection
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V301(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 0;
		//else if (detections > 0)  priority = 0;
		else
			priority = 1;
	}
	else {
		// Active Camera
		priority = 2;
	}

	return m_priorities.get(priority);
}



int CLoadBalaner::calcPriority(int motion, int detections, int alerts, int activeCamera)
{
	int priority = 0;
	switch (m_scheme) {
	case LB_SCHEME::V0:
		priority = calcPriority_V0(motion, detections, alerts, activeCamera);
		break;
	case LB_SCHEME::V1:
		priority = calcPriority_V1(motion, detections, alerts, activeCamera);
		break;
	case LB_SCHEME::V2:
		priority = calcPriority_V2(motion, detections, alerts, activeCamera);
		break;
	case LB_SCHEME::V3:
		priority = calcPriority_V3(motion, detections, alerts, activeCamera);
		break;
	case LB_SCHEME::V30:
		priority = calcPriority_V30(motion, detections, alerts, activeCamera);
		break;
	// Smooth schemes:
	//-----------------
	case LB_SCHEME::V200:
		priority = calcPriority_V200(motion, detections, alerts, activeCamera);
		break;
	case LB_SCHEME::V201:
		priority = calcPriority_V201(motion, detections, alerts, activeCamera);
		break;
	case LB_SCHEME::V300:
		priority = calcPriority_V300(motion, detections, alerts, activeCamera);
		break;
	case LB_SCHEME::V301:
		priority = calcPriority_V301(motion, detections, alerts, activeCamera);
		break;
	default:
		LOGGER::log(DLEVEL::INFO2, " Load balancer got a Wrong Scheme " + std::to_string(m_scheme));
	}

	return priority;
}


#if 0
/*----------------------------------------------------------------------------------------------------
* Scheme 0:
* All cams born equal 
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V0(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0;

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0)
		priority = 0;
	else
		priority = 1;

	int queuePriority = convertToQueuePriority(priority);

	return queuePriority;
}
#endif 
/*----------------------------------------------------------------------
	Convert priority 0-5 to realtime queue priority number.
	Chack current topPrior of the queue
	?? Consider the qeueu length and other parameters ??
 ----------------------------------------------------------------------*/
int CLoadBalaner::convertToQueuePriority(int priority)
{
	int queuePriority = 0;

	switch (priority) {
		// Backend camera
	case 0:
		queuePriority = 0;
		break;
	case 1:
	case 2:
		queuePriority = m_priorities.m_3dPrio;
		break;
		// Observation camera
	case 3:
	case 4:
		queuePriority = m_priorities.m_2ndPrio;
		break;
	case 5:
		queuePriority = m_priorities.m_highPrio;
		break;
	}

	return queuePriority;
}

/*----------------------------------------------------------------------
	Convert priority 0-5 to realtime queue priority number.
	Chack current topPrior of the queue
	?? Consider the qeueu length and other parameters ??
 ----------------------------------------------------------------------*/
int CLoadBalaner::convertToQueuePriority_simple(int priority)
{
	int queuePriority = 0;

	switch (priority) {
		// Backend camera
	case 0:
		queuePriority = m_priorities.m_lowPrio;
		break;
	case 1:
		queuePriority = m_priorities.m_MidPrio;
	case 2:
		queuePriority = m_priorities.m_highPrio;
		break;
	default:
		queuePriority = m_priorities.m_lowPrio;
		break;
	}

	return queuePriority;
}



#if 0
int CLoadBalaner::convertToQueuePriority_V1(int priority)
{
	int queuePriority = 0;

	switch (priority) {
		// Backend camera
	case 0:
		queuePriority = 0;
		break;
	case 1:
		queuePriority = max(m_thirdPriority, 1);
		break;
	case 2:
		queuePriority = max(m_2thirdPriority, 2);
		break;
		// Observation camera
	case 3:
		queuePriority = m_priorityQueue.top().priority - 3;
		queuePriority = max(queuePriority, 3);
		break;
	case 4:
		queuePriority = m_priorityQueue.top().priority - 2;
		queuePriority = max(queuePriority, 4);
		break;
	case 5:
		queuePriority = m_priorityQueue.top().priority;
		//queuePriority = max(queuePriority, 5);
		break;
	}

	return queuePriority;
}
#endif 


/*-------------------------------------------------
 -------------------------------------------------*/
void CLoadBalaner::updatePriorThresholds()
{
	m_actualTopPriority = m_priorityQueue.top().priority;
	m_actualTopPriority = max(m_actualTopPriority, 1); /// min top = 1
	/*
	auto sortedHeap = m_priorityQueue.status(); // not neccessary with new queu
	m_2thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.33)].priority; // 0 ind is highest prior
	m_thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.67)].priority;
	*/
}



void CLoadBalaner::initCamera(int videoIndex)
{
	if (m_active)
		m_priorityQueue.add(videoIndex);
}


void CLoadBalaner::SetCameraType(int camID, int type)
{
	if (camID >= m_camType.size())
		std::cerr << "SetCameraType got illegal camera ID \n";
	else
		m_camType[camID] = type;
}

void CLoadBalaner::set(int camID, int proir)
{
	std::lock_guard<std::mutex> lockGuard(qeue_mutex);
	m_priorityQueue.set(camID, proir);
}


/*----------------------------------------------------------------------
Check camera status (location in queue) and Resources status 
 (exception - first frame of camera allowed and gets high-prior 
 'allowOverflow' flag - run camera and push to queue even if no resource or not in top of queue 
 return if OVERFLOW was accured 
----------------------------------------------------------------------*/
AQUIRE_ERROR CLoadBalaner::acquire(int camID, bool allowOverflow)
{
	if (!m_active)
		return AQUIRE_ERROR::NOT_ACTIVE;

	AQUIRE_ERROR error = AQUIRE_ERROR::OK;

	// Camera first frame: Put in high prior
	//if (std::find(m_camInQueue.begin(), m_camInQueue.end(), camID) == m_camInQueue.end()) { 		//m_camInQueue.push_back(camID);
	
	// Set priority to a NEW camera
	if (m_priorityQueue.getPriority(camID) < 0) { // camera new - not in queue
		m_priorityQueue.set(camID, m_actualTopPriority);
	}
	else //Check 1: in top priority list 
		if (!m_priorityQueue.inTop(camID, m_resourceNum)) {
			LOGGER::log(DLEVEL::ERROR2, "LB System : acquired camera that is not in top (Load Balancer missmatching?) cam = " + std::to_string(camID));
			error = AQUIRE_ERROR::NOT_IN_TOP;
		}

	m_camPriority_Debug[camID] = m_priorityQueue.getPriority(camID); // DDEBUG 


	if (error == AQUIRE_ERROR::OK || allowOverflow) // in case this cam will be processed: 
		m_priorityQueue.reset(camID); // reset prior 


	return error;

}


void CLoadBalaner::remove(int camID)
{
	//m_camInQueue.erase(std::remove(m_camInQueue.begin(), m_camInQueue.end(), camID), m_camInQueue.end());
	m_priorityQueue.remove(camID); 
}




/*----------------------------------------------------------------------------------------
* Tune queue params (beatStep etc.) according to cameras num and resources (batch-size)
* beatSize is set so that after batchSize steps it will have the (regular) highest priority 
* highest priority
------------------------------------------------------------------------------------------*/
void CLoadBalaner::tuneQueueParams( int cameraNum)
{
	if (0)
	{// suspended suspended DDEBUG 
		for (auto range : m_resourcesRange) {

			if (range.inRange(cameraNum)) {
				if (range.resourcesNum < 0) // cameras num lower that min resource
					m_resourceNum = max(1, cameraNum);
				else
					m_resourceNum
					= range.resourcesNum;
				break;
			}
		}
	}

	int cyclesToFinishAllCams = ROUND(((float)cameraNum / (float)m_resourceNum));
	cyclesToFinishAllCams = max(cyclesToFinishAllCams, 1);

	m_beatStep = m_topPriority / cyclesToFinishAllCams;

	m_priorities.init(cameraNum, m_resourceNum, m_topPriority);

	m_resourceBouncer.set(m_resourceNum);
}


#if 1
bool CLoadBalaner::releaseDebug(int camID)
{
	m_resourceBouncer.release();
	m_priorityQueue.reset(camID); // reset prior 
	return true;
}

#endif  



#endif 

