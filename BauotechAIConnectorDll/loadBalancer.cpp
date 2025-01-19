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

//const int LB_Mode = LB_MODE::FULL;
/*---------------------------------------------------------------------------------------------
* Queues description
* 1. m_priorityQueue -  priority queue of cameras
* 2. m_cameraBatchList - list of cameras to be processed in the next batch
*
* 3. g_resQueue - Temp queue of processed frames. stores in  'g_cameraBatchListDone; 
* 4. g_cameraBatchListDone - list of cameras that were processed in the current batch
* 5. m_cameraBatchListAquired - list of cameras that were aquired in the current batch
* 6. m_logBatchQueue - list of cameras that were processed in the current batch (for logging)

* 7. m_resourceBouncer - semaphore for resource management
---------------------------------------------------------------------------------------------*/
//---------------------------------------------------------------------------------
// Singletone YOLO initialization:
std::map<int, std::shared_ptr<ST_yoloDetectors>> ST_yoloDetectors::instances;

ThreadSafeVector <int> ST_yoloDetectors::freeDetectors;
ThreadSafeVector <int>  g_cameraBatchListDone;

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




//#ifdef USE_LOAD_BALANCER



void CLoadBalaner::cameraCounter(int cams)
{ 
	m_camerasNum += cams;  
	tuneQueueParams(m_camerasNum);

} // Add or remove cameras counter



void CLoadBalaner::statisticsTH()
{
	//CTimer timer;
	//std::vector <int> activeCameras;


	int batchCounter = 0; // for print statistics 
	m_cycleCounter = 0;
	int statisticsModulo = (m_debugLevel > DLEVEL::ERROR2 ? 30 * 40 : 30 * 100);
	statisticsModulo = 100; // 400; // 50  
	//timer.start();

	StatisticsInfo statisticsRes;
	statisticsRes.clear();

	//timer.start();

	while (!m_terminate) {

		// Batch handling: Count 'm_resourceNum' cameras inputs , beat() queue and than cacl next batch  list 
		//------------------------------------------------------------------------------------------------------
		bool timeOut = false;

		if (g_resQueue.size() >= m_resourceNum || timeOut) { // == timeToUpdateBatchList()
			m_cycleCounter++;
			batchCounter++;

			// Dashbaord 
			/*
			m_dashboard.update(m_priorityQueue.getCamList(), statisticsRes, m_actualTopPriority);
			m_dashboard.show();
			*/

			// Clean g_resQueue, push info to m_logBatchQueue
			CCycle info;

			std::unique_lock<std::shared_mutex> lock(g_resQMtx);
			//for (int i = 0; i < g_resQueue.size(); i++) {
			for (; !g_resQueue.empty(); g_resQueue.pop()) {
				info = g_resQueue.front();
				//g_resQueue.pop();  // release top queue element
				m_logBatchQueue.push_back(info);
			}
			lock.unlock();

			//timer.start();
		}


		if (m_printStatistics && batchCounter > statisticsModulo) {
			LOGGER::log(DLEVEL::_SYSTEM_INFO, ">> Number of Cameras = " + std::to_string(m_camerasNum) + " Batch size = " + std::to_string(m_resourceNum));

			statisticsRes = printStatistics(m_logBatchQueue, m_priorityQueue.getCamList(), m_resourceNum, m_actualTopPriority);
			m_logBatchQueue.clear();
			batchCounter = 0;
		}
	}

}
//--------------------------------------------------------
// thread to set priority according to detection results 
// recieved from algo process
//--------------------------------------------------------
void CLoadBalaner::priorityTH()
{
	//CTimer timer;


	int batchCounter = 0; // for print statistics 
	m_cycleCounter = 0;
	int statisticsModulo = (m_debugLevel > DLEVEL::ERROR2 ? 30 * 40 : 30 * 100);
	statisticsModulo = 100; // 400; // 50  
	//timer.start();

	StatisticsInfo statisticsRes;
	statisticsRes.clear();

	//timer.start();

	while (m_priorityQueue.empty())
		Sleep(4); //  std::cout << "wait for camera to fillup \n"; // DDEBUG PRINT 

	prepareNextBatch(m_Mode);// kickoff balancer list

	int imageProccessed = 0;
	while (!m_terminate) {

		if (!priorityUpdateTop())
			continue;

		// Batch handling: Count 'm_resourceNum' cameras inputs , beat() queue and than cacl next batch  list 
		//------------------------------------------------------------------------------------------------------
		bool timeOut = false;

 		if (g_cameraBatchListDone.size() >= m_resourceNum || timeOut) { // == timeToUpdateBatchList()
			m_cycleCounter++;
			batchCounter++;
			prepareNextBatch(m_Mode);

			// Dashbaord 
			/*
			m_dashboard.update(m_priorityQueue.getCamList(), statisticsRes, m_actualTopPriority);
			m_dashboard.show();
			timer.start();
			*/
			
		}


		if (m_printStatistics && batchCounter > statisticsModulo) { 
			LOGGER::log(DLEVEL::_SYSTEM_INFO, ">> Number of Cameras = " + std::to_string(m_camerasNum) + " Batch size = " + std::to_string(m_resourceNum));

			statisticsRes = printStatistics(m_logBatchQueue, m_priorityQueue.getCamList(), m_resourceNum, m_actualTopPriority);
			m_logBatchQueue.clear();
			batchCounter = 0;
		}
	}

}

//-----------------------------------------------------------------------
// "lihgt balancer" thread to switch to next batch list,
// right after bathSize frames arrived (before the detection process)
//-----------------------------------------------------------------------
void CLoadBalaner::priorityTHLight()
{
	//CTimer timer;


	int batchCounter = 0; // for print statistics 
	m_cycleCounter = 0;
	int statisticsModulo = (m_debugLevel > DLEVEL::ERROR2 ? 30 * 40 : 30 * 100);
	statisticsModulo = 100; // 400; // 50  
	//timer.start();

	StatisticsInfo statisticsRes;
	statisticsRes.clear();

	//timer.start();

	while (m_priorityQueue.empty())
		Sleep(4); //  std::cout << "wait for camera to fillup \n"; // DDEBUG PRINT 

	prepareNextBatch(m_Mode);// kickoff balancer list

	int imageProccessed = 0;
	while (!m_terminate) {

		if (!priorityUpdateTop())
			continue;

		// Batch handling: Count 'm_resourceNum' cameras inputs , beat() queue and than cacl next batch  list 
		//------------------------------------------------------------------------------------------------------
		bool timeOut = false;

		if (g_cameraBatchListDone.size() >= m_resourceNum || timeOut) { // == timeToUpdateBatchList()
 			m_cycleCounter++;
			batchCounter++;
			prepareNextBatch(m_Mode);

		}


		if (m_printStatistics && batchCounter > statisticsModulo) {
			LOGGER::log(DLEVEL::_SYSTEM_INFO, ">> Number of Cameras = " + std::to_string(m_camerasNum) + " Batch size = " + std::to_string(m_resourceNum));

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
	const std::lock_guard<std::shared_mutex> lock(g_resQMtx);

	g_resQueue.push(info); // unhandled cam

	g_cameraBatchListDone.push_back(info.camID); // Done cam list 

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


	// Store g_resQueue top int g_cameraBatchListDone vec:
	CCycle info;
	{
		std::unique_lock<std::shared_mutex> lock(g_resQMtx);

		info = g_resQueue.front();
		g_resQueue.pop();  // release top queue element
		lock.unlock();

	}

	m_resourceBouncer.release();


	LOGGER::log(DLEVEL::WARNING2, std::string("LB: update priority  for camera = " + std::to_string(info.camID)));


	info.activeCamera = m_camType[info.camID] == CAMERA_TYPE::Active ? 1 : 0;

	// Set new priority 
	info.priority = calcPriority(info.motion, info.detections, info.alerts, info.activeCamera);

	m_priorityQueue.set(info.camID, info.priority);

	m_logBatchQueue.push_back(info); // keep for monitoring 


	return true;
}


void CLoadBalaner::prepareNextBatch(int mode)
{
	if (mode == LB_MODE::FULL)
		prepareNextBatchFull();
	else
		prepareNextBatchLight();
}

/*------------------------------------------------------------------------------------
* Read the top '10' of the queue
* Send them to Server (cameras streamer)  via callback 
* Update the priority constants (3d, etc.)
 ------------------------------------------------------------------------------------*/
void CLoadBalaner::prepareNextBatchFull()
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

	// DDEBUG - print QUEUE info 
	//-----------------------------
	printQueueInfo();
	g_cameraBatchListDone.clear();
/*
	{
		int skipQueueCounter = 1; // DDEBUG 
		if (++printQueueCounter % skipQueueCounter == 0) {
			printQueueInfo();
			printQueueCounter = 0;
		}
		m_cameraBatchListDone.clear();
	}
*/


	// Send batch info to server (via Callback):
	if (m_bachCallback != nullptr) {
		uint32_t cameras[100];

		for (int i = 0; i < m_cameraBatchList.size(); i++)
			cameras[i] = (uint32_t)m_cameraBatchList[i];

		m_bachCallback(cameras, (int)m_cameraBatchList.size());
		batchlist_loc.unlock();
		//updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.

	}
	
	updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.
}

/*------------------------------------------------------------------
 *Prepare next cam batch - no priority, just const batches 
 ------------------------------------------------------------------*/
void CLoadBalaner::prepareNextBatchLight()
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

	m_cameraBatchList = getCamTopLight();
	
#if 1
	// DDEBUG PRINT 
	std::cout << ">>>> m_cameraBatchList = ";
	for (auto cam : m_cameraBatchList)
		std::cout << cam << ", ";
	std::cout << "\n";
#endif 
	for (int c=0;c< m_resourceNum;c++)
		m_cameraBatchList.push_back(c);


	// DDEBUG - print QUEUE info 
	//-----------------------------
	printQueueInfo();

	std::unique_lock<std::shared_mutex> batchlist_loc(bacthlist_MTX);
	g_cameraBatchListDone.clear();

	if (m_bachCallback != nullptr) {
		// Send batch info to server (via Callback):
		uint32_t cameras[100];

		for (int i = 0; i < m_cameraBatchList.size(); i++)
			cameras[i] = (uint32_t)m_cameraBatchList[i];

		m_bachCallback(cameras, (int)m_cameraBatchList.size());
	}
	batchlist_loc.unlock();
}

std::vector <int> CLoadBalaner::getCamTopLight()
{
	std::vector <int> topCameras;

		std::vector <int> camList = m_priorityQueue.getCamList();
		int camsSize = camList.size();
		for (int i = 0; i < m_resourceNum; i++)
			topCameras.push_back(m_priorityQueue.getCam((m_lastCamIndex+i+1) % camsSize));


		m_lastCamIndex = (m_lastCamIndex + m_resourceNum) % camsSize;

	return topCameras;
}





/*-------------------------------------------------------------------
* Init resource size (batchNum)
* init() return the debugLevel (for algoAPI)
-------------------------------------------------------------------*/
int CLoadBalaner::init(uint32_t *batchSize, LB_MODE loader_Mode)
{
	std::lock_guard<std::shared_mutex> lock(API_MTX);


	m_Mode = loader_Mode;



	Config params;
	APIData apiData;
	params.GPUBatchSize = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
	FILE_UTILS::readConfigFile(params, apiData);

	m_debugLevel = params.debugLevel_LB;
	if (isValidSchemeNum(params.LB_scheme))
		m_scheme = LB_SCHEME(params.LB_scheme);
	else {
		LOGGER::log(DLEVEL::INFO2, " Unknownn Load balancer Scheme " + std::to_string(m_scheme) + ", use V0 (default)");
		m_scheme = LB_SCHEME::V0;
	}

	// YOLO MNGR threads creation - this op is here since LB is the "holistic" thread, see all cameras threads 
	if (YOLO_MNGR)
		initYoloMngr(params.GPUBatchSize, params.modelFolder);

	*batchSize = (uint32_t)params.GPUBatchSize;



	LOGGER::log(DLEVEL::INFO1, " Load balancer Scheme = " + std::to_string(m_scheme));


	m_maxResourceNum = m_resourceNum = ST_yoloDetectors::m_maxDetectors =  params.GPUBatchSize; // Calc number of threads can run on this PC simultaneously  
	
	//m_logBatchQueue.set_capacity(30*100*10); // DDEBUG CONST 


	switch (m_Mode) {
		case LB_MODE::FULL:
			m_PrioirityTh = std::thread(&CLoadBalaner::priorityTH, this);
			m_active = true;
			break;
		case LB_MODE::LIGHT:
			m_PrioirityTh = std::thread(&CLoadBalaner::priorityTHLight, this);
			m_active = true;
			break;
		case LB_MODE::STATISTICS:
			m_PrioirityTh = std::thread(&CLoadBalaner::statisticsTH, this);
			m_active = false; // dont operate balancer, just record statistics
			break;
	}

	m_camType.assign(MAX_VIDEOS, CAMERA_TYPE::Normal);

	bool DRAW_DASHBOARD = false;
	if (DRAW_DASHBOARD)
		m_dashboard.init(params.GPUBatchSize, m_resourceNum);


	return m_debugLevel;
}


bool CLoadBalaner::initYoloMngr(int resourceNum, std::string modelFolder)
{
	// Create & init yolo "floating" detectors 
	
	for (int i = 0; i < resourceNum; i++) {
		auto yolo2 = ST_yoloDetectors::getInstance(i);
		if (yolo2 == nullptr || !yolo2->init(modelFolder, true)) {
			std::cout << "Cant init YOLO net , quit \n";
			return false;
		}
	}
	return true;
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
	//if (!m_active)  return;

	std::lock_guard<std::shared_mutex> lock(API_MTX);

	m_priorityQueue.add(camID);
	// update m_resourceNum with new cam if possible 
	m_resourceNum = MIN(m_maxResourceNum, (int)m_priorityQueue.size());

	if (m_active)
		prepareNextBatch(m_Mode); // kickoff balancer list (otherwise causes desklock with duplication check in Aqcuire())

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

	bool checkIfInList = false; // DDEBUG flag
	bool checkDuplications = false;  // DDEBUG flag

	AQUIRE_ERROR ret;
	std::shared_lock<std::shared_mutex> s_loc(bacthlist_MTX);

	if (checkIfInList && std::find(m_cameraBatchList.begin(), m_cameraBatchList.end(), camID) == m_cameraBatchList.end())
		ret = AQUIRE_ERROR::NOT_IN_LIST;
	else if (checkDuplications && g_cameraBatchListDone.findVal(camID))
		ret = AQUIRE_ERROR::DUPLICATE_AQUIRED;
	else 
		ret = AQUIRE_ERROR::OK;

#if 0
	if (m_Mode == LB_MODE::LIGHT && ret == AQUIRE_ERROR::OK)
	{
		CCycle info;
		info.detections = 0;
		info.alerts = 0;
		info.camID = camID;
		info.activeCamera = 0;
		info.motion = 0;
		info.timeStamp = -1; // DDEBUG 
		info.status = OBJECT_STATUS::IN_PROCESS;
		ResQueuePush(info);

	}
#endif

	// DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG PRINT ---------------------------------------------
	/*
	if (1)
	{
		if (ret == AQUIRE_ERROR::NOT_IN_LIST)
		{
			std::string errorStr = "LB ignore cam = " + std::to_string(camID) + "(list:";
			for (auto cam : m_cameraBatchList)
				errorStr.append(std::to_string(cam) + ",");
			errorStr.append(")");

			LOGGER::log(DLEVEL::_SYSTEM_INFO, errorStr);
		}
		else if (ret == AQUIRE_ERROR::DUPLICATE_AQUIRED)
		{
			std::string errorStr = "LB ignore cam = " + std::to_string(camID) + "-DUPLPICATED";
			LOGGER::log(DLEVEL::_SYSTEM_INFO, errorStr);
		}
	}
	*/
	//-----------------------------------------------------------------------------------------------

	return ret;
}

void CLoadBalaner::remove(int camID)
{
	std::lock_guard<std::shared_mutex> lock(API_MTX);
	m_camerasNum -= 1;
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


	bool PRINT_REQUEST_VS_DONE = false;
	cur_LOG_LEVEL = DLEVEL::INFO1;
	if (PRINT_REQUEST_VS_DONE)
		cur_LOG_LEVEL = DLEVEL::_SYSTEM_INFO;

	if (LOGGER::getDevLevel() >= cur_LOG_LEVEL) {  // optimize access to priorityQueue

		// DDEBUG print processed cameras -------------------------------------------
		// CAMs had been rocessed in prev cycle (DONE)
		std::string batchDoneStr = "LB camera list DONE : <( ";
		{
			for (int i = 0; i < g_cameraBatchListDone.size(); i++) {
				batchDoneStr.append(std::to_string(g_cameraBatchListDone.get(i)));
				batchDoneStr.append(",");
			}
		}


		// Cams list Requested for next cycle :
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
