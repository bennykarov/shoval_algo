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

#ifdef USE_LOAD_BALANCER



#ifdef _DEBUG
const int beatTick = 99;
#else
const int beatTick = 33;
#endif 


void CLoadBalaner::cameraCounter(int cams)
{ 
	m_camerasNum += cams;  
	updatePCResource(m_camerasNum);



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

			printStatistics(m_logBatchQueue, m_priorityQueue.getCamList(), m_resourceNum);
			m_logBatchQueue.clear();
			batchCounter = 0;
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
	bool balancerLogActive = true;
	std::unique_lock<std::mutex> lock(g_resMtx);

	//g_resCondV.wait(lock, [] { return !g_resQueue.empty(); });
	auto timeOut = 250ms;
	bool dataExist = g_resCondV.wait_for(lock, timeOut, [] { return !g_resQueue.empty(); });
	if (!dataExist) {
		LOGGER::log(DLEVEL::WARNING2, "Waiting for camera data (Res Queue is empty) !(\n");
		return false;
	}
	// else  std::cout << " Res Queue size = " << g_resQueue.size() << "\n";


	CCycle info = g_resQueue.front();
	g_resQueue.pop();  // release queue element
	m_resourceBouncer.release();

	// Set new priority 
	info.activeCamera = m_camType[info.camID] == CAMERA_TYPE::Active ? 1 : 0;

	if ((info.motion + info.detections + info.alert + info.activeCamera) > 0) {
		info.priority = calcPriority_V2(info.motion, info.detections, info.alert, info.activeCamera);
		m_priorityQueue.set(info.camID, info.priority);
	}
	else
		info.priority = m_priorityQueue.getPriority(info.camID); // for printing the proir


	m_logBatchQueue.push_back(info); // DDEBUG (low) : keep for monitoring 
	
	lock.unlock();  // Release the lock before processing the data to allow other threads to access the queue


	if (m_debugLevel > 3) {
		info.priority = m_camPriority_Debug[info.camID];
		std::cout << "cam " << info.camID << " (P=" << info.priority << ", A=" << info.activeCamera << ") detection=" << info.detections << "\n";
	}

	return true;
}




/*------------------------------------------------------------------------------------
* Read the top '10' of the queue
* Send them to loadBalancer via callback 
* Update the priority constants (3d, etc.)
 ------------------------------------------------------------------------------------*/
void CLoadBalaner::prepareNextBatch()
{
	static int debugCounter = 0;
	// first increase un-handled cams prior:
	m_priorityQueue.beat();
	if (m_debugLevel > DLEVEL::ALL)
		LOGGER::log(DLEVEL::INFO2, "Beat()");
	
	//m_priorityQueue.beatExclusive(m_cameraBatchList);
	// get next top priority cameras:			
	auto topQueue = m_priorityQueue.getTop(m_resourceNum);

	m_cameraBatchList.clear();
	//retrieve only the cams list 
	std::transform(std::cbegin(topQueue), std::cend(topQueue),
		std::back_inserter(m_cameraBatchList), [](const auto& data) { return data.key;});

	if (0) {// DDEBUG CHECK 
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

		/*
		if (m_debugLevel > DLEVEL::INFO2) {// DEBUG LOGGER
			std::string batchListStr = "( ";
			for (auto cam : m_cameraBatchList) {
				batchListStr.append(std::to_string(cam));
				batchListStr.append(",");
			}
			batchListStr.append(" )");

			LOGGER::log(DLEVEL::INFO1, batchListStr);
		}
		*/

		/*
		m_cameraBatchList_prev.assign(m_cameraBatchList.begin(), m_cameraBatchList.end());
		if (!m_cameraBatchList_prev.empty() && equal(m_cameraBatchList.begin(), m_cameraBatchList.end(), m_cameraBatchList_prev.begin()))
			int debug = 10;
		*/
	}
	//else  std::cout << "DUplicate batch ??? \n";

	// DEBUG PRINTS:
	//if (m_debugLevel > DLEVEL::INFO2) 
	if (true) { // DDEBUG
		if (++debugCounter % int(30 / 4 * 50) == 0) {
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
int CLoadBalaner::init()
{
	m_active = true;


	// setup resourceNum
	m_resourcesRange.push_back(CResourceRange(0, 10, -1));
	m_resourcesRange.push_back(CResourceRange(11, 90, 10));
	m_resourcesRange.push_back(CResourceRange(91, 150, 20));
	m_resourcesRange.push_back(CResourceRange(151, 250, 30));

	Config params;
	params.GPUBatchSize = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
	FILE_UTILS::readConfigFile(params);
	m_bestResourceNum = params.GPUBatchSize;
	updatePCResource(m_camerasNum);
	//m_resourceBouncer.set(m_resourceNum);

	m_debugLevel = params.debugLevel;

	//m_resourceNum = calcPCResource(); // Calc number of threads can run on this PC simultaneously  
	//m_resourceNum = 100; // DDEBUG DDEBUG TEST !!
	
	//m_logBatchQueue.set_capacity(30*m_resourceNum*10); // DDEBUG CONST 
	m_logBatchQueue.set_capacity(30*100*10); // DDEBUG CONST 

	m_PrioirityTh = std::thread(&CLoadBalaner::priorityTH, this);

	m_camType.assign(MAX_VIDEOS, CAMERA_TYPE::Normal);
	m_camPriority_Debug.assign(MAX_VIDEOS, -1);





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
int CLoadBalaner::calcPriority(int motion, int detections, int alert, int activeCamera)
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

	int queuePriority = convertToQueuePriority_V1(priority);
	//camsInTopPriority()

	return queuePriority;
}

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

	int queuePriority = convertToQueuePriority_V2(priority);

	return queuePriority;
}

/*----------------------------------------------------------------------
	Convert priority 0-5 to realtime queue priority number.
	Chack current topPrior of the queue
	?? Consider the qeueu length and other parameters ??
 ----------------------------------------------------------------------*/
int CLoadBalaner::convertToQueuePriority_V2(int priority)
{
	int queuePriority = 0;

	switch (priority){
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


/*-------------------------------------------------
 -------------------------------------------------*/
void CLoadBalaner::updatePriorThresholds()
{
	m_topPriority = m_priorityQueue.top().priority;
	m_topPriority = max(m_topPriority, 1); /// min top = 1
	auto sortedHeap = m_priorityQueue.status(); // not neccessary with new queu
	m_2thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.33)].priority; // 0 ind is highest prior
	m_thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.67)].priority;
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
	//if (std::find(m_camInQueue.begin(), m_camInQueue.end(), camID) == m_camInQueue.end()) { 		
		//m_camInQueue.push_back(camID);
	if (m_priorityQueue.getPriority(camID) < 0) { // camera new - not in queue
		m_priorityQueue.set(camID, m_topPriority);
	}
	else //Check 1: in top priority list 
		if (!m_priorityQueue.inTop(camID, m_resourceNum)) 
			error = AQUIRE_ERROR::NOT_IN_TOP;

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





void CLoadBalaner::updatePCResource( int cameraNum)
{
	for (auto range : m_resourcesRange) {

		if (range.inRange(cameraNum)) {
			if (range.resourcesNum < 0) // cameras num lower that min resource
				m_resourceNum = max(1, cameraNum);
			else
				m_resourceNum = range.resourcesNum;
			break;
		}
	}

	/*
	if (m_resourceNum > m_camerasNum)
		m_resourceNum = max(1, m_camerasNum);

	else if (m_camerasNum )

		m_resourceNum = min(m_bestResourceNum, m_camerasNum);
	*/
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

