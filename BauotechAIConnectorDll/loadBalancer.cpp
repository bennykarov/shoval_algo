#include <thread> 
#include <functional>  // std::ref
#include <future>    // std::promise
#include <vector>
#include <algorithm>
#include <condition_variable> 
#include <chrono>
#include <iostream> 
#include <mutex> 
#include <queue> 
#include <stdlib.h>     /* srand, rand */
#include <numeric>
#include <string>

#include "opencv2/opencv.hpp"

#include "config.hpp"

#include "AlgoApi.h" // for MAX_VIDEO const
#include "logger.hpp"
#include "loadBalancer.hpp"
#include "testBalancer.hpp" 

#include "database.hpp"
#include "yolo/YOLO_mngr.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

using namespace std::chrono_literals;
/*---------------------------------------------------------------------------------------------
* Queues description
* 1. m_priorityQueue -  priority queue of cameras
* 2. m_cameraBatchList - list of cameras to be processed in the next batch
*
* 3. g_resQueue - Temp queue of processed frames. stores in  'm_cameraBatchListDone; 
* 4. m_cameraBatchListDone - list of cameras that were processed in the current batch
* 5. m_cameraBatchListAquired - list of cameras that were aquired in the current batch
* 6. m_logBatchQueue - list of cameras that were processed in the current batch (for logging)

* 7. m_resourceBouncer - semaphore for resource management
---------------------------------------------------------------------------------------------*/
//---------------------------------------------------------------------------------
// Singletone YOLO initialization:
std::map<int, std::shared_ptr<ST_yoloDetectors>> ST_yoloDetectors::instances;

ThreadSafeVector <int> ST_yoloDetectors::freeDetectors;
ThreadSafeVector <int>  m_cameraBatchListDone;

std::mutex ST_yoloDetectors::st_mtx;
int ST_yoloDetectors::m_maxDetectors = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
//---------------------------------------------------------------------------------

//std::mutex qeue_mutex;
std::shared_mutex bacthlist_MTX;
std::shared_mutex API_MTX;
std::stringstream  g_logMsg;

// sync results

std::condition_variable g_resCondV;
std::mutex              g_resCondVMtx;

std::queue<CCycle>      g_resQueue;
std::shared_mutex       g_resQMtx;


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


	int batchCounter = 0; // for print statistics 
	m_cycleCounter = 0;
	int statisticsModulo = (m_debugLevel > DLEVEL::ERROR2 ? 30 * 40 : 30 * 100);
	statisticsModulo = 100; // 400; // 50  
	//timer.start();

	StatisticsInfo statisticsRes;
	statisticsRes.clear();

	timer.start();

	while (m_priorityQueue.empty())
		Sleep(4); //  std::cout << "wait for camera to fillup \n"; // DDEBUG PRINT 

	prepareNextBatch();// kickoff balancer list

	int imageProccessed = 0;
	while (!m_terminate) {

		if (!priorityUpdateTop())
			continue;


#if 0
		{
			timer.start();
			Sleep(10);
			float elapsedMS = timer.sampleFromStart();
			int debug = 10;

		}
#endif 
		// Batch handling: Count 'm_resourceNum' cameras inputs , beat() queue and than cacl next batch  list 
		//------------------------------------------------------------------------------------------------------
		bool timeOut = false;
		/*
		float elapsedMS = timer.sampleFromStart();
		if (elapsedMS  > CONSTANTS::MAX_BATCH_PERIOD)
			std::cout << "t=" << (float)elapsedMS/1000000.;// timeOut = true;
		// short version: timeOut = (timer.sample() > CONSTANTS::MAX_BATCH_PERIOD);
		*/

		if (m_cameraBatchListDone.size() >= m_resourceNum || timeOut) { // == timeToUpdateBatchList()
			m_cycleCounter++;
			batchCounter++;
			prepareNextBatch();
			//processCounter = 0;

			// Dashbaord 
			m_dashboard.update(m_priorityQueue.getCamList(), statisticsRes, m_actualTopPriority);
			m_dashboard.show();
			timer.start();
			//std::cout << ">> start a new batch \n";
			
		}


		if (m_printStatistics && batchCounter > statisticsModulo) { 

			LOGGER::log(DLEVEL::ERROR1, ">> Number of Cameras = " + std::to_string(m_camerasNum) + " Batch size = " + std::to_string(m_resourceNum));

			statisticsRes = printStatistics(m_logBatchQueue, m_priorityQueue.getCamList(), m_resourceNum, m_actualTopPriority);
			m_logBatchQueue.clear();
			batchCounter = 0;
		}



	}

}


/*-----------------------------------------------------------------------------
* resQueue handling : 
  1. push, 2. notify (called bY algoProcess thread)
* 3. priorityUpdate() runs after m_resource inputs 
* All three functions are protected by the g_resQMtx mutex
-----------------------------------------------------------------------------*/
void CLoadBalaner::ResQueuePush(CCycle info)
{

	//LOGGER::log(DLEVEL::DEBUG_HIGH, std::string(std::string(" 'reQueue' get processed frame : " + std::to_string(info.camID))));

	const std::lock_guard<std::shared_mutex> lock(g_resQMtx);

	g_resQueue.push(info); // unhandled cam

	m_cameraBatchListDone.push_back(info.camID); // Done cam list 

	
}

void CLoadBalaner::ResQueueNotify()
{
	std::lock_guard<std::mutex> lock_cv(g_resCondVMtx);
	g_resCondV.notify_one();
}

// Set priority to TOP cam in resQueue (res from detecor thread)
bool CLoadBalaner::priorityUpdateTop()
{

	// Get cam detectror info (g_resQueue) from algoDetection thread(s) 
	while (g_resQueue.empty()) {
		Sleep(5);
	}
	//s_lock.unlock();


	// Store g_resQueue top int m_cameraBatchListDone vec:
	CCycle info;
	{
		std::unique_lock<std::shared_mutex> lock(g_resQMtx);

		info = g_resQueue.front();
		g_resQueue.pop();  // release top queue element
		lock.unlock();

	}

	m_resourceBouncer.release();


	LOGGER::log(DLEVEL::INFO2, std::string("LB: update priority  for camera = " + std::to_string(info.camID)));


	info.activeCamera = m_camType[info.camID] == CAMERA_TYPE::Active ? 1 : 0;

	// Set new priority 
	info.priority = calcPriority(info.motion, info.detections, info.alerts, info.activeCamera);

	m_priorityQueue.set(info.camID, info.priority);

	m_logBatchQueue.push_back(info); // keep for monitoring 


	return true;
}



/*------------------------------------------------------------------------------------
* Read the top '10' of the queue
* Send them to Server (cameras streamer)  via callback 
* Update the priority constants (3d, etc.)
 ------------------------------------------------------------------------------------*/
void CLoadBalaner::prepareNextBatch()
{
	static int printQueueCounter = 0;

	{
		std::lock_guard<std::shared_mutex> batchlist_loc(bacthlist_MTX);
		m_cameraBatchList.clear();
	}

	// first increase un-handled cams prior:
	m_priorityQueue.beat(m_beatStep);

	if (m_debugLevel > DLEVEL::ALL)
		LOGGER::log(DLEVEL::INFO2, "Beat()");
	
	// get next top priority cameras:			
	auto topQueue = m_priorityQueue.getTop(m_resourceNum);

	std::unique_lock<std::shared_mutex> batchlist_loc(bacthlist_MTX); 

	// Copy topQueue to m_cameraBatchList
	std::transform(std::cbegin(topQueue), std::cend(topQueue),
		std::back_inserter(m_cameraBatchList), [](const auto& data) { return data.key; });


	// Send batch info to server (via Callback):
	if (m_bachCallback != nullptr) {
		uint32_t cameras[100];

		//std::copy(m_cameraBatchList.begin(), m_cameraBatchList.end(), cameras);
		for (int i = 0; i < m_cameraBatchList.size(); i++)
			cameras[i] = (uint32_t)m_cameraBatchList[i];

		m_bachCallback(cameras, (int)m_cameraBatchList.size());
		batchlist_loc.unlock();
		//updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.

	}
	
	updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.



	// DDEBUG - print QUEUE info 
	//-----------------------------
	int skip = 1; // DDEBUG 
	if (++printQueueCounter % skip == 0) {
		printQueueInfo();
		printQueueCounter = 0;
	}

	// Clean only after calling printQueueInfo() - it uses these vector  
	m_cameraBatchListDone.clear();


}


/*-------------------------------------------------------------------
* Init resource size (batchNum)
* init() return the debugLevel (for algoAPI)
-------------------------------------------------------------------*/
int CLoadBalaner::init(uint32_t *batchSize)
{
	std::lock_guard<std::shared_mutex> lock(API_MTX);

	m_active = true;


	// setup resourceNum
	m_resourcesRange.push_back(CResourceRange(0, 10, -1));
	m_resourcesRange.push_back(CResourceRange(11, 90, 10));
	m_resourcesRange.push_back(CResourceRange(91, 150, 20));
	m_resourcesRange.push_back(CResourceRange(151, 250, 30));

	Config params;
	params.GPUBatchSize = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
	FILE_UTILS::readConfigFile(params);
	m_debugLevel = params.debugLevel_LB;
	if (isValidSchemeNum(params.LB_scheme))
		m_scheme = LB_SCHEME(params.LB_scheme);
	else {
		LOGGER::log(DLEVEL::ERROR2, " Unknownn Load balancer Scheme " + std::to_string(m_scheme) + ", use V0 (default)");
		m_scheme = LB_SCHEME::V0;
	}

	*batchSize = params.GPUBatchSize;
	*batchSize = (uint32_t)params.GPUBatchSize;


	LOGGER::log(DLEVEL::DEBUG_HIGH, " Load balancer Scheme = " + std::to_string(m_scheme));


	m_maxResourceNum = m_resourceNum = ST_yoloDetectors::m_maxDetectors =  params.GPUBatchSize; // Calc number of threads can run on this PC simultaneously  
	
	m_logBatchQueue.set_capacity(30*100*10); // DDEBUG CONST 

	m_PrioirityTh = std::thread(&CLoadBalaner::priorityTH, this);

	m_camType.assign(MAX_VIDEOS, CAMERA_TYPE::Normal);
	//m_camPriority_Debug.assign(MAX_VIDEOS, -1);


	bool DRAW_DASHBOARD = false;
	if (DRAW_DASHBOARD)
		m_dashboard.init(params.GPUBatchSize, m_resourceNum);


	if (YOLO_MNGR) {
		// Create & init yolo "floating" detectors 
		int resourceNum = params.GPUBatchSize;
		for (int i = 0; i < resourceNum; i++) {
			auto yolo2 = ST_yoloDetectors::getInstance(i);
			if (yolo2 == nullptr || !yolo2->init(params.modelFolder, true))
				std::cout << "Cant init YOLO net , quit \n";
		}
	}

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
* 3 levels scheme (0..3)
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
		else
			priority = 1;
	}
	else {
		// Active Camera
		priority = 2;
	}

	return m_priorities.get(priority);
}

/*----------------------------------------------------------------------------------------------------
* Scheme 302:
* low gain
* 4 levels scheme (0..3)
* No-detection priority  -> 2
* With-detection -> 1
* With-alert -> 0

* Active cam priority  -> 3
* ----------------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority_V302(int motion, int detections, int alert, int activeCamera)
{
	int priority = 0; // default

	// First set priority from 0 to 5:
	//-----------------------------------
	if (activeCamera == 0) {
		if (alert > 0)
			priority = 0;
		else if (detections > 0)
			priority = 1;
		else 
			priority = 2;
	}
	else {
		// Active Camera
		priority = 3;
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
	case LB_SCHEME::V302:
		priority = calcPriority_V302(motion, detections, alerts, activeCamera);
		break;
	default:
		LOGGER::log(DLEVEL::ERROR2, " Unknownn Load balancer Scheme " + std::to_string(m_scheme) + ", use V0 (default)");
		priority = calcPriority_V0(motion, detections, alerts, activeCamera);
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


/*-------------------------------------------------
 -------------------------------------------------*/
void CLoadBalaner::updatePriorThresholds()
{
	m_actualTopPriority = m_priorityQueue.top().priority;
	m_actualTopPriority = max(m_actualTopPriority, 1); /// min top = 1
}



void CLoadBalaner::initCamera(int camID)
{
	if (!m_active)
		return;

	std::lock_guard<std::shared_mutex> lock(API_MTX);

	m_priorityQueue.add(camID);
	// update m_resourceNum with new cam if possible 
	m_resourceNum = MIN(m_maxResourceNum, (int)m_priorityQueue.size());

	prepareNextBatch(); // kickoff balancer list (otherwise causes desklock with duplication check in Aqcuire())

}

void CLoadBalaner::SetCameraRequestCallback(CameraRequestCallback callback)
{
	std::lock_guard<std::shared_mutex> lock(API_MTX);
	m_bachCallback = callback;
}


void CLoadBalaner::SetCameraType(int camID, int type)
{
	std::lock_guard<std::shared_mutex> lock(API_MTX);

	if (camID >= m_camType.size())
		std::cerr << "SetCameraType got illegal camera ID \n";
	else
		m_camType[camID] = type;
}

void CLoadBalaner::set(int camID, int proir)
{
	//std::lock_guard<std::mutex> lockGuard(qeue_mutex); -> Unnecessary, since  m_priorityQueue is thread safe 
	m_priorityQueue.set(camID, proir);
}


/*----------------------------------------------------------------------
Check camera status (location in queue) and Resources status 
 (exception - first frame of camera allowed and gets high-prior 
 'allowOverflow' flag - run camera and push to queue even if no resource or not in top of queue 
 return if OVERFLOW was accured 
----------------------------------------------------------------------*/
AQUIRE_ERROR CLoadBalaner::acquire(int camID)
{
	if (!m_active)
		return AQUIRE_ERROR::NOT_ACTIVE;

	const bool haltLoadBalancer = false; // disable oad balancer aqcuire limitation 
	bool checkInList = true;
	bool checkDuplicated = true;

	if (haltLoadBalancer)
		checkInList = checkDuplicated = false;

	AQUIRE_ERROR ret;
	std::shared_lock<std::shared_mutex> s_loc(bacthlist_MTX);

	if (checkInList && std::find(m_cameraBatchList.begin(), m_cameraBatchList.end(), camID) == m_cameraBatchList.end())
		ret = AQUIRE_ERROR::NOT_IN_LIST;
	else if (checkDuplicated && m_cameraBatchListDone.findVal(camID))
		ret = AQUIRE_ERROR::DUPLICATE_AQUIRED;
	else 
		ret = AQUIRE_ERROR::OK;
	

	// DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG PRINT ---------------------------------------------
	if (1)
	{
		if (ret == AQUIRE_ERROR::NOT_IN_LIST)
		{
			std::string errorStr = "LB ignore cam = " + std::to_string(camID) + "(list:";
			for (auto cam : m_cameraBatchList)
				errorStr.append(std::to_string(cam) + ",");
			errorStr.append(")");

			LOGGER::log(DLEVEL::DEBUG_HIGH, errorStr);
		}
		else if (ret == AQUIRE_ERROR::DUPLICATE_AQUIRED)
		{
			std::string errorStr = "LB ignore cam = " + std::to_string(camID) + "-DUPLPICATED";
			LOGGER::log(DLEVEL::DEBUG_HIGH, errorStr);
		}
	}
	//-----------------------------------------------------------------------------------------------

	return ret;
}

void CLoadBalaner::remove(int camID)
{
	std::lock_guard<std::shared_mutex> lock(API_MTX);
	m_priorityQueue.remove(camID); 
}




/*----------------------------------------------------------------------------------------
* Tune queue params (beatStep etc.) according to cameras num and resources (batch-size)
* beatSize is set so that after batchSize steps it will have the (regular) highest priority 
* highest priority
------------------------------------------------------------------------------------------*/
void CLoadBalaner::tuneQueueParams( int cameraNum)
{
	int cyclesToFinishAllCams = ROUND(((float)cameraNum / (float)m_resourceNum));
	cyclesToFinishAllCams = max(cyclesToFinishAllCams, 1);

	m_beatStep = m_topPriority / cyclesToFinishAllCams;

	m_priorities.init(cameraNum, m_maxResourceNum, m_topPriority);

	m_resourceBouncer.set(m_maxResourceNum);
}



/*-----------------------------------------------------------------
* Print queue information to LOG
-----------------------------------------------------------------*/
void CLoadBalaner::printQueueInfo()
{

	DLEVEL cur_LOG_LEVEL = DLEVEL::INFO2;
	//DLEVEL cur_LOG_LEVEL = DLEVEL::DEBUG_HIGH;
	if (LOGGER::getDevLevel() >= cur_LOG_LEVEL) { // // optimize access to priorityQueue
			auto Queue = m_priorityQueue.status();
		std::string fullQueue = "P_QUEUE : ";
		for (auto cam : Queue)
			fullQueue.append("(" + std::to_string(cam.key) + "," + std::to_string(cam.priority) + ") ");
		LOGGER::log(cur_LOG_LEVEL, fullQueue);
	}


	cur_LOG_LEVEL = DLEVEL::INFO1;
	if (LOGGER::getDevLevel() >= cur_LOG_LEVEL) {  // optimize access to priorityQueue

		// DDEBUG print processed cameras -------------------------------------------
		std::string batchDoneStr = "LB camera list DONE (prev cycle): <( ";
		{
			for (int i = 0; i < m_cameraBatchListDone.size(); i++) {
				batchDoneStr.append(std::to_string(m_cameraBatchListDone.get(i)));
				batchDoneStr.append(",");
			}
		}

		batchDoneStr.append(" )");
		LOGGER::log(cur_LOG_LEVEL, batchDoneStr);
		//---------------------------------------------------------------------------

		std::string batchListStr = "LB camera list request:           >( ";
		for (auto cam : m_cameraBatchList) {
			batchListStr.append(std::to_string(cam));
			batchListStr.append(",");
		}
		batchListStr.append(" )");
		LOGGER::log(cur_LOG_LEVEL, batchListStr);
	}
}

#if 0
bool CLoadBalaner::isCamInBatchList(int camID)
{
	const std::lock_guard<std::mutex> batchlist_loc(bacthlist_MTX);

	return (std::find(m_cameraBatchList.begin(), m_cameraBatchList.end(), camID) != m_cameraBatchList.end());
}
#endif 




#endif 

