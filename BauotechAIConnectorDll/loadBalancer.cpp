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

#include "logger.hpp"
#include "loadBalancer.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

using namespace std::chrono_literals;


//const int MaxCameras = 20;
std::mutex qeue_mutex;

// sync results
static std::queue<CCycle>         m_resQueue;
static std::mutex              m_resMtx;
static std::condition_variable m_resCondV;
//CameraRequestCallback *m_bachCallback = nullptr;

#ifdef USE_LOAD_BALANCER



#ifdef _DEBUG
const int beatTick = 99;
#else
const int beatTick = 33;
#endif 



/*-------------------------------------------------------------------------------------
* m_resQueue is a queue used for communicate between camera thread (algoProcess) -
*  to loadbalancer priorityTH() thread.
 -------------------------------------------------------------------------------------*/
std::queue<CCycle> *CLoadBalaner::getResQueuePtr()
{
	return &m_resQueue;
}

std::condition_variable* CLoadBalaner::getResCondVPtr()
{
	return &m_resCondV;
}

#if 0
/*-----------------------------------------------------------------------------------
* CALLBACK Function - to be claled by the Camera thread - insert the results to 'm_resQueue'
* ( m_resQueue is a thread safe function)
------------------------------------------------------------------------------------*/
void updateResults(CCycle info)
{
	if (info.detections > 0)
	{
		// send the data to the priority thread (=load balancer)
		//--------------------------------------------------------
		std::lock_guard<std::mutex> lock(m_resMtx);
		CCycle info;
		info.alert = 0;
		info.activeCamera = 0;
		info.motion = info.detections > 0;
		//info.timeStamp = m_frameNum; // DDEBUG 
		m_resQueue.push(info);
		m_resCondV.notify_one();  // Notify loaderTH that a new data is available
	}
}
#endif 

//--------------------------------------------------------
// thread to set priority according to detection results 
// recieved from algo process
//--------------------------------------------------------
void CLoadBalaner::priorityTH()
{
	int processCounter = 0;

	while (!m_terminate) {

		priorityUpdate();

		// Batch handling:
		processCounter++;
		if (processCounter == m_resourceNum) { // == timeToUpdateBatchList()
			nextBatch();
			processCounter = 0;
		}

	}

}//--------------------------------------------------------
// priority calc 
// main function of priorityTH( thread
//--------------------------------------------------------
bool CLoadBalaner::priorityUpdate()
{
	std::unique_lock<std::mutex> lock(m_resMtx);

	auto timeOut = 500ms;
	bool dataExist = m_resCondV.wait_for(lock, timeOut, [] { return !m_resQueue.empty(); });
	if (!dataExist) {
		std::cout << "Res Queue is empty !\n";
		return false;
	}
	else
		std::cout << " Res Queue size = " << m_resQueue.size() << "\n";
		
	// Get data from the queue
	CCycle info = m_resQueue.front();
	// release queue element
	m_resQueue.pop();
	m_resourceBouncer.release();
	m_camInProcess.erase(std::remove(m_camInProcess.begin(), m_camInProcess.end(), info.camID), m_camInProcess.end());
	lock.unlock();  // Release the lock before processing the data to allow other threads to access the queue

	info.activeCamera = m_camType[info.camID] == CAMERA_TYPE::Active ? 1 : 0;

	// Set new priority:
	if ((info.motion + info.detections + info.alert + info.activeCamera) > 0) {
		info.priority = calcPriority(info.motion, info.detections, info.alert, info.activeCamera);
		m_priorityQueue.set(info.camID, info.priority);
	}
}


void CLoadBalaner::nextBatch()
{
	// first increase un-handled cams prior:
	//m_priorityQueue.beatExclusive(m_cameraBatchList);
	// get next top priority cameras:			
	auto topQueue = m_priorityQueue.getTop(m_resourceNum);
	m_cameraBatchList.clear();
	//retrieve only the cams list 
	std::transform(std::cbegin(topQueue), std::cend(topQueue),
		std::back_inserter(m_cameraBatchList),
		[](const auto& data) {
			return data.key;
		});

	// Send batch info to server:
	if (m_bachCallback != nullptr) {
		uint32_t cameras[100];
		std::copy(m_cameraBatchList.begin(), m_cameraBatchList.end(), cameras);
		m_bachCallback(cameras, (int)m_cameraBatchList.size());
	}

	// step a batch cycle 
	updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.


}


void CLoadBalaner::init()
{
	m_active = true;
	m_resourceNum = calcPCResource(); // Calc number of threads can run on this PC simultaneously  
	//m_resourceNum = 100; // DDEBUG DDEBUG TEST !!
	m_resourceBouncer.set(m_resourceNum);
	m_PrioirityTh = std::thread(&CLoadBalaner::priorityTH, this);

	m_camType.assign(MAX_CAMERAS, CAMERA_TYPE::Normal);
}


void CLoadBalaner::beatTimer()
{
	while (!m_terminate) {
		LOGGER::log(" BEAT()              ----->");
		beat();
		Sleep(beatTick);
	}
}


/*------------------------------------------------------------------------------------------
* Set priority according to camera online parameters 
 ------------------------------------------------------------------------------------------*/
int CLoadBalaner::calcPriority(int motion, int detections, int alert, int observed)
{
	int priority = 0;

	// First set priority from 0 to 5:
	//-----------------------------------



	if (observed == 0) {
		if (alert > 0 )
			priority = 2;
		else if (detections > 0)
			priority = 1;
		else 
			priority = 0;
	}
	else {
		int topPrior = m_priorityQueue.top().priority;
		// camera is an observed cam
		if (detections > 0 || alert > 0)
			priority = 5;
		else if (motion > 0)
			priority = 4;
		else // no motion 
			priority = 3;
	}

	int queuePriority = convertToQueuePriority(priority);
	//camsInTopPriority()

	return queuePriority;
}

/*----------------------------------------------------------------------
	Convert priority 0-5 to realtime queue priority number.
	Chack current topPrior of the queue
	?? Consider the qeueu length and other parameters ??
 ----------------------------------------------------------------------*/
int CLoadBalaner::convertToQueuePriority(int priority)
{
	int queuePriority = 0;

	switch (priority){
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
	if (camID >= MAX_CAMERAS)
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
 'force' flag - run camera anycase ! 
----------------------------------------------------------------------*/
bool CLoadBalaner::acquire(int camID, bool allowOverflow)
{
	if (!m_active)
		return true;

	bool overFlow = false;
	int maxWaitToResourceSec = 0; // DDEBUG CONST 


	// Camera first frame: Put in high prior
	if (std::find(m_camInQueue.begin(), m_camInQueue.end(), camID) == m_camInQueue.end()) { 		
		m_camInQueue.push_back(camID);
		m_priorityQueue.set(camID, m_topPriority);
	}
	else //Check 1: in top priority list 
		if (!m_priorityQueue.inTop(camID, m_resourceNum)) 
			overFlow = true;

	// Check 2: for a free resource 
	if (!m_resourceBouncer.take()) {
		/*
		std::cout << "Overflow in resources (run  anyway) \n";
		std::cerr << "Overflow in resources (run  anyway) \n"; // should be LOG FILE
		*/
		overFlow = true;
	}

	//std::cout << " CLoadBalaner::acquire() has " << m_resourceBouncer.get() << " resources \n";

	if (!allowOverflow && overFlow)
		return false;

	// Add to launch list
	m_priorityQueue.reset(camID); // reset prior 
	m_camInProcess.push_back(camID);

	return true;

}




void CLoadBalaner::setPrior(CCycle info)
{
	if ((info.motion + info.detections + info.alert + info.activeCamera) > 0) {
		info.priority = calcPriority(info.motion, info.detections, info.alert, info.activeCamera);
		m_priorityQueue.set(info.camID, info.priority);

		//updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.

	}


}


void CLoadBalaner::remove(int camID)
{
	m_camInQueue.erase(std::remove(m_camInQueue.begin(), m_camInQueue.end(), camID), m_camInQueue.end());
	// Missing functionL: m_priorityQueue.remove(camID); DDEBUG DDEBUG MISSING 
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