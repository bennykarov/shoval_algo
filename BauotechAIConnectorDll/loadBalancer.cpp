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

#include "AlgoApi.h" // for MAX_VIDEO const
#include "logger.hpp"
#include "loadBalancer.hpp"
#include "testBalancer.hpp" 

#include "database.hpp"
#include "yolo/YOLO_mngr.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

using namespace std::chrono_literals;

//---------------------------------------------------------------------------------
// Singletone YOLO initialization:
std::map<int, std::shared_ptr<ST_yoloDetectors>> ST_yoloDetectors::instances;
std::mutex ST_yoloDetectors::st_mtx;
ThreadSafeVector <int> ST_yoloDetectors::freeDetectors;
int ST_yoloDetectors::m_maxDetectors = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
//---------------------------------------------------------------------------------

std::mutex qeue_mutex;
std::mutex bacthlist_MTX;
std::stringstream  g_logMsg;




// sync results
static std::queue<CCycle>         g_resQueue;
static std::mutex              g_resQMtx;
static std::condition_variable g_resCondV;
static std::mutex              g_resCondVMtx;


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
	//CTimer timer;

	int batchCounter = 0; // for print statistics 
	m_cycleCounter = 0;
	int statisticsModulo = (m_debugLevel > DLEVEL::ERROR2 ? 30 * 40 : 30 * 100);
	statisticsModulo = 100; // 400; // 50  
	//timer.start();

	StatisticsInfo statisticsRes;
	statisticsRes.clear();


	CTimer timer_while;

	while (m_priorityQueue.empty())
		std::cout << "wait for camera to fillup \n";

	prepareNextBatch();// kickoff balancer list

	int imageProccessed = 0;
	while (!m_terminate) {

		if (!priorityUpdateTop())
			continue;

		// Batch handling: Count 'm_resourceNum' cameras inputs , beat() queue and than cacl next batch  list 
		//------------------------------------------------------------------------------------------------------
		//processCounter++; 
		//if (processCounter >= m_resourceNum) { // == timeToUpdateBatchList()
		if (m_cameraBatchListDone.size()  >= m_resourceNum) { // == timeToUpdateBatchList()
			m_cycleCounter++;
			batchCounter++;
			prepareNextBatch();
			//processCounter = 0;

			// Dashbaord 
			m_dashboard.update(m_priorityQueue.getCamList(), statisticsRes, m_actualTopPriority);
			m_dashboard.show();
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

	LOGGER::log(DLEVEL::INFO2, std::string(std::string(" 'reQueue' get processed frame : " + std::to_string(info.camID))));

	const std::lock_guard<std::mutex> lock(g_resQMtx);

	g_resQueue.push(info);
	
}

void CLoadBalaner::ResQueueNotify()
{
	std::lock_guard<std::mutex> lock_cv(g_resCondVMtx);
	g_resCondV.notify_one();
}

// Set priority to top cam in queue
bool CLoadBalaner::priorityUpdateTop()
{
	auto dataTimeOut = 40ms; // DDEBUG CONST 

#if 1
	// Get data from resQueue:
	{
		while (g_resQueue.empty()) {
			//LOGGER::log(DLEVEL::WARNING2, std::string("Waiting 10ms for data from detector (resQueu is empty"));
			Sleep(5);
			// missing TIMEOUT + return FALSE !!!!
		}
	}
#else
	//  Naive waiting to res in queue
	{
		while (g_resQueue.empty()) {
			//LOGGER::log(DLEVEL::WARNING2, std::string("Waiting 10ms for data from detector (resQueu is empty"));
			Sleep(5);
		}
	}
#endif 

	// Read top resQueue element from queue (thread protected):
	static CCycle info;
	{
		std::lock_guard<std::mutex> lock(g_resQMtx);

		info = g_resQueue.front();
		g_resQueue.pop();  // release top queue element

		if (g_resQueue.size() > 1)  // DDEBUG PRINT
			std::cout << "g_resQueue size > 1 : (" << g_resQueue.size() << ")\n";

		/*
		auto camID = std::find(m_cameraBatchList.begin(), m_cameraBatchList.end(), info.camID);
		if (camID == m_cameraBatchList.end())   std::cout << "Unmatched camera pushed to resQueue";
		*/
		m_cameraBatchListDone.push_back(info.camID);

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



/*------------------------------------------------------------------------------------
* Read the top '10' of the queue
* Send them to Server (cameras streamer)  via callback 
* Update the priority constants (3d, etc.)
 ------------------------------------------------------------------------------------*/
void CLoadBalaner::prepareNextBatch()
{
	static int printQueueCounter = 0;
	// first increase un-handled cams prior:
	m_priorityQueue.beat(m_beatStep);

	if (m_debugLevel > DLEVEL::ALL)
		LOGGER::log(DLEVEL::INFO2, "Beat()");
	
	// get next top priority cameras:			
	auto topQueue = m_priorityQueue.getTop(m_resourceNum);


	{
		std::lock_guard<std::mutex> batchlist_loc(bacthlist_MTX);

		m_cameraBatchList.clear();
		//retrieve only the cams list 
		std::transform(std::cbegin(topQueue), std::cend(topQueue),
			std::back_inserter(m_cameraBatchList), [](const auto& data) { return data.key; });

	}

	if (1) {// DDEBUG CHECK 
		const auto duplicate = std::adjacent_find(m_cameraBatchList.begin(), m_cameraBatchList.end());

		if (duplicate != m_cameraBatchList.end())
			LOGGER::log(DLEVEL::DEBUG_HIGH, std::string("Duplicate element = " + std::to_string(*duplicate)));
	}


	// Send batch info to server (via Callback):
	if (m_bachCallback != nullptr) {
		uint32_t cameras[100];
		std::copy(m_cameraBatchList.begin(), m_cameraBatchList.end(), cameras);
		m_bachCallback(cameras, (int)m_cameraBatchList.size());

		updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.

	}



	// DDEBUG - print QUEUE info 
	int skip = 1; // DDEBUG 
	if (++printQueueCounter % skip == 0) {
		printQueueInfo();
		printQueueCounter = 0;
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
	//tuneQueueParams(m_camerasNum);


	m_debugLevel = params.debugLevel_LB;


	m_maxResourceNum = m_resourceNum = ST_yoloDetectors::m_maxDetectors =  params.GPUBatchSize; // Calc number of threads can run on this PC simultaneously  
	
	m_logBatchQueue.set_capacity(30*100*10); // DDEBUG CONST 

	m_PrioirityTh = std::thread(&CLoadBalaner::priorityTH, this);

	m_camType.assign(MAX_VIDEOS, CAMERA_TYPE::Normal);
	//m_camPriority_Debug.assign(MAX_VIDEOS, -1);


	bool DRAW_DASHBOARD = false;
	if (DRAW_DASHBOARD)
		m_dashboard.init(params.GPUBatchSize, m_resourceNum);

	// Create & init yolo "floating" detectors 
	int resourceNum = params.GPUBatchSize; // DDEBUG DDEBUG CONST reouceNum
	for (int i = 0; i < resourceNum; i++) {
		auto yolo2 = ST_yoloDetectors::getInstance(i);
		if (yolo2 == nullptr || !yolo2->init(params.modelFolder, true))
			std::cout << "Cant init YOLO net , quit \n";
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
	if (!m_active)
		return;

	m_priorityQueue.add(videoIndex);
	// update m_resourceNum with new cam if possible 
	m_resourceNum = MIN(m_maxResourceNum, (int)m_priorityQueue.size());


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
	//std::lock_guard<std::mutex> lockGuard(qeue_mutex); -> Unnecessary, since  m_priorityQueue is thread safe 
	m_priorityQueue.set(camID, proir);
}


/*----------------------------------------------------------------------
Check camera status (location in queue) and Resources status 
 (exception - first frame of camera allowed and gets high-prior 
 'allowOverflow' flag - run camera and push to queue even if no resource or not in top of queue 
 return if OVERFLOW was accured 
----------------------------------------------------------------------*/
AQUIRE_ERROR CLoadBalaner::acquire(int camID, bool processNotInList)
{
	if (!m_active)
		return AQUIRE_ERROR::NOT_ACTIVE;

	// Check that there ius a YOLO resource 
	if (YOLO_MNGR) {
		if (ST_yoloDetectors::getFreeInstenceInd() < 0)
			return AQUIRE_ERROR::RESOURCE_OVERFLOW;
	}

	AQUIRE_ERROR error = AQUIRE_ERROR::OK;

	// Camera first frame: Put in high prior
	//if (std::find(m_camInQueue.begin(), m_camInQueue.end(), camID) == m_camInQueue.end()) { 		//m_camInQueue.push_back(camID);
	
	// Set priority to a NEW camera
	if (m_priorityQueue.getPriority(camID) < 0) { // camera new - not in queue
		m_priorityQueue.set(camID, m_actualTopPriority);
	}
	else //Check 1: in top priority list 
		if (!m_priorityQueue.inTop(camID, m_resourceNum)) {
			LOGGER::log(DLEVEL::INFO2, "LB ignore cam = " + std::to_string(camID) + " (not in top)");
			//LOGGER::log(DLEVEL::INFO1, "LB ignore cam = " + std::to_string(camID) + " (not in top)");
			error = AQUIRE_ERROR::NOT_IN_TOP;
		}

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
	int cyclesToFinishAllCams = ROUND(((float)cameraNum / (float)m_resourceNum));
	cyclesToFinishAllCams = max(cyclesToFinishAllCams, 1);

	m_beatStep = m_topPriority / cyclesToFinishAllCams;

	m_priorities.init(cameraNum, m_maxResourceNum, m_topPriority);

	m_resourceBouncer.set(m_maxResourceNum);
}


bool CLoadBalaner::isCamInBatchList(int camID)
{
	const std::lock_guard<std::mutex> batchlist_loc(bacthlist_MTX);

	return (std::find(m_cameraBatchList.begin(), m_cameraBatchList.end(), camID) != m_cameraBatchList.end());
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


	cur_LOG_LEVEL = DLEVEL::DEBUG_HIGH;
	if (LOGGER::getDevLevel() >= cur_LOG_LEVEL) {  // optimize access to priorityQueue

		// DDEBUG print processed cameras -------------------------------------------
		std::string batchDoneStr = "LB camera list DONE (prev cycle): ( ";
		for (auto cam : m_cameraBatchListDone) {
			batchDoneStr.append(std::to_string(cam));
			batchDoneStr.append(",");
		}
		batchDoneStr.append(" )");
		LOGGER::log(cur_LOG_LEVEL, batchDoneStr);

		m_cameraBatchListDone.clear();

		//---------------------------------------------------------------------------

		std::string batchListStr = "LB camera list request: ( ";
		for (auto cam : m_cameraBatchList) {
			batchListStr.append(std::to_string(cam));
			batchListStr.append(",");
		}
		batchListStr.append(" )");
		LOGGER::log(cur_LOG_LEVEL, batchListStr);
	}
}

#if 0
bool CLoadBalaner::releaseDebug(int camID)
{
	m_resourceBouncer.release();
	m_priorityQueue.reset(camID); // reset prior 
	return true;
}
#endif  



#endif 

