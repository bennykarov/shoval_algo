#include <thread> 
#include <functional>  // std::ref
#include <future>    // std::promise
#include <condition_variable> 
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

boost::random::mt19937 gen;


const int MaxCameras = 20;
std::mutex qeue_mutex;
std::mutex print_mutex;


#ifdef USE_LOAD_BALANCER


const int RAND_RATIO = 7.;
const int CYCLES_NUM = 5500;
const int processDuration = 0;
int g_run6Counter = 0;

#ifdef _DEBUG
const int beatTick = 99;
#else
const int beatTick = 33;
#endif 

int MAX_DETECTIONS = 10;




void CLoadBalaner::beatTimer()
{
	while (!m_terminate) {
		LOGGER::log(" BEAT()              ----->");
		beat();
		Sleep(beatTick);
	}
}



void CLoadBalaner::init(CSemaphore* sem)
{
	m_timestamps.set_capacity(TIMESTAMP_LEN);
	m_resourceNum = calcPCResource(); // Calc number of threads can run on this PC simultaneously  
	setRosourceSemaphore(sem);
}





void test_run6(CCycle &info, std::promise <CCycle>& pInfo, int duration_ms)
{
	g_run6Counter++;

	// -3- Get free resource 
	/*
	int maxWaitToResourceSec = 2000;
	if (!m_resourceBouncer->take(maxWaitToResourceSec)) {
		std::cout << "no resource available for N elapse time\n";
		std::cout << " Error waiting too  long to free resource \n";
	}
	m_priorityQueue.reset(info.camID); // reset prior
	*/


	if (duration_ms > 0)
		Sleep(duration_ms);
	// Detection results:
	if (info.camID == 11) {
		//info.activeCamera = 1;
		info.detections = 1;
		info.alert = 1;
	}
	else if (info.camID == 7) {
		//info.activeCamera = 0;
		info.detections = 0;
		info.alert = 0;
	}
	else if (info.camID % 3 == 0) {
		// return 0, 1 ranodmaly , 
			boost::random::uniform_int_distribution<> dist(0, 5);
			info.detections = dist(gen) == 1  ? 1 : 0;
	}
	else
		info.detections = 0; // rand() < int((float)RAND_MAX / (RAND_RATIO / 2)) ? 1 : 0;

	{
		std::lock_guard<std::mutex> lockGuard(qeue_mutex);
		std::cout << "cam " << info.camID << " (P" << info.priority << ", A" << info.activeCamera << ") detection=" << info.detections << "\n";

		if (info.camID == 0) {
			if (info.detections == 0)
				int debug = 10;
			else
				int debug = 11;
		}
	}


	pInfo.set_value(info); // update for caller thread 

	// -5- Free resource 
	//m_resourceBouncer->release();


	//std::cout << "  detections= " << detections << "\n";
}
#if 0
//------------------------------------------------------------------
// "process" a frame , return detection "status" (detected objects) 
//------------------------------------------------------------------
void CLoadBalaner::test_run3_async(int camID, int printPrior, int duration_ms, CCycle &info)
{
	int alert = 0;
	int observed = 0;
	//info.reset();
	//info.camID = camID;


	// -3- Get free resource 
	int maxWaitToResourceSec = 2000;
	if (!m_resourceBouncer->take(maxWaitToResourceSec)) {
		std::cout << "no resource available for N elapse time\n";
		std::cout << " Error waiting too  long to free resource \n";
	}
	m_priorityQueue.reset(camID); // reset prior


	if (duration_ms > 0)
		Sleep(duration_ms);

	// Special cameras:
	/*
	if (camID == 11) {
		info.activeCamera = 1;
		info.detections = 1;
		info.alert = 1;
	}
	*/
	if (camID == 7) {
		info.activeCamera = 0;
		info.detections = 0;
		info.alert = 0;
	}
	else {
		info.activeCamera = 0;
		info.alert = 0;
		// return 0, 1 ranodmaly 
		info.detections = rand() < int((float)RAND_MAX / RAND_RATIO) ? 1 : 0;
		//detections = camID % 2 == 0 ? 2 : 0;
	}

	// -5- Free resource 
	m_resourceBouncer->release();


	//{std::lock_guard<std::mutex> lockGuard(mutex);
	//std::cout << "\n * Process cam " << camID << " (P" << printPrior << ")  detects  " << detections << \n";}

	//return { detections, alert, observed };
}
#endif 


bool printCycles(std::string fname, std::vector < CCycle> cycs)
{
	std::ofstream queueFile(fname);

	if (!queueFile.is_open()) {
		std::cout << "ERROR : Cant open " << fname << " file to write \n";
		return false;
	}

	queueFile << " camNum , priority , deetction Res  \n";


	for (auto cycle : cycs)
		queueFile << cycle.camID << "," << cycle.priority << "," << cycle.detections << ",\n";

	queueFile.close();
	return true;
}


bool printCyclesSummary(std::string fname, std::vector < CCycle> cyclesInfo, int activeCameras)
{
	std::ofstream queueFile(fname);

	if (!queueFile.is_open()) {
		std::cout << "ERROR : Cant open " << fname << " file to write \n";
		return false;
	}

	queueFile << " camNum , detecions#,  totalCycles, avg-Ellapsd, min-Ellapsd, max-Ellapsd, \n";
	

	std::cout << "total cycles = " << cyclesInfo.size() << "\n";
	std::cout << "\n\n\n";
	std::cout << " camNum , detecions#,  totalCycles, avgEllapsd \n";
	std::cout << "-------------------------------------------------\n";

	// Summarize info for each cam
	for (int cam = 0; cam < activeCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.camID == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam; });
		int activeFrames = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam && c.activeCamera > 0; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.camID == cam)
				sumDetectoins += cycleInf.detections > 0 ? 1 : 0;
		}
		//int detectionsNum = std::count(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.res  .camID == cam; });

		//--------------------------
		// Calc avg ellapsed time
		//--------------------------
		int prevFrameNum = -1;
		std::vector <int> framesNum;
		for (int i = 0; i < cyclesInfo.size();i++) {
			if (cyclesInfo[i].camID == cam)
				framesNum.push_back(cyclesInfo[i].timeStamp);
		}

		int ellapsedTime = 0;
		int max_ellapsedTime = 0;
		int min_ellapsedTime = 99999;
		for (int i = 1; i < framesNum.size(); i++) {
			int ellapsed = framesNum[i] - framesNum[i - 1];
			if (ellapsed == 1)
				int debug = 10;
			ellapsedTime += ellapsed;
			max_ellapsedTime = max(max_ellapsedTime, ellapsed);
			min_ellapsedTime = min(min_ellapsedTime, ellapsed);
		}

		if (!framesNum.empty())
			ellapsedTime /= framesNum.size();




		std::cout << "cam= " << cam << " ; active frame = " << activeFrames << " ; detections = " << sumDetectoins << " ; cycles = " << processNum << " ; avg Wait = " << ellapsedTime << " (min, max: " << min_ellapsedTime << "," << max_ellapsedTime <<")\n";


		queueFile << cam << "," << sumDetectoins << "," << processNum << ","  << ellapsedTime << "," << min_ellapsedTime << "," << max_ellapsedTime << ",\n";

	}


	queueFile.close();
	return true;
}



#if 0
std::tuple <int,int> CLoadBalaner::getNextCam()
{
	// -2- Get next camera + reset prior + beat()
	if (!m_priorityQueue.empty())
		return  { -1, -1 };

	auto topCam = m_priorityQueue.top();
	int camID = topCam.key;
	int camPrior = topCam.priority;
	m_priorityQueue.reset(camID); // reset 
	//???m_priorityQueue.beat();


	return std::tuple<int, int>(camID, camPrior);

}
#endif 

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

	int queuePriority = converToQueuePriority(priority);
	//camsInTopPriority()

	return queuePriority;
}

/*----------------------------------------------------------------------
	Convert priority 0-5 to realtime queue priority number.
	Chack current topPrior of the queue
	?? Consider the qeueu length and other parameters ??
 ----------------------------------------------------------------------*/
int CLoadBalaner::converToQueuePriority(int priority)
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
		queuePriority = m_priorityQueue.top().priority - 1;
		queuePriority = max(queuePriority, 5);
		break;
	}


	return queuePriority;

}


void printStatistics(std::vector <CCycle> cyclesInfo, int camerasLen)
{


	std::cout << "\n\n\n=====================================================================\n\n";
	std::cout << "g_run6Counter = " << g_run6Counter;

	int avgCall = int((float)(cyclesInfo.size()) / (float)camerasLen);
	for (int cam = 0; cam < MaxCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.camID == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.camID == cam)
				sumDetectoins += cycleInf.detections;
		}
		std::cout << "Cam " << cam << " ; dections = " << sumDetectoins << " ; cycles# = " << processNum << " (" << processNum - avgCall << ")\n";
	}
	if (1)
		printCycles("c:\\tmp\\priorQueue.csv", cyclesInfo);

	printCyclesSummary("c:\\tmp\\priorQueueSumm.csv", cyclesInfo, camerasLen);
	/*

	std::cout << "\n\n\n=====================================================================\n\n";

	int avgCall = int((float)(cyclesInfo.size()) / (float)camerasLen);
	for (int cam = 0; cam < MaxCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.camID == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.camID == cam)
				sumDetectoins += cycleInf.detections;
		}
		std::cout << "Cam " << cam << " ; dections = " << sumDetectoins << " ; cycles# = " << processNum << " (" << processNum - avgCall << ")\n";
	}
	if (1)
		printCycles("c:\\tmp\\priorQueue.csv", cyclesInfo);

	printCyclesSummary("c:\\tmp\\priorQueueSumm.csv", cyclesInfo, camerasLen);
*/
}


void CLoadBalaner::test()
{
	if (1)
		std::srand(std::time(nullptr));

	std::vector <CCycle> cyclesInfo;

	// Init  resouce & cameras:
	m_resourceBouncer->set(m_resourceNum);

	int testedCameras = 20;
	for (int i = 0; i < testedCameras; i++)
		m_priorityQueue.set(i, 0);

	m_camerasLen = m_priorityQueue.size();

	if (m_priorityQueue.empty()) {
		std::cout << "No camera had been init ! Quit \n";
		return;
	}



	int cyclesCount = CYCLES_NUM / m_resourceNum;
	for (int cycle = 0; cycle < cyclesCount; cycle++) {

		m_priorityQueue.beat();

		for (int threadsRun = 0; threadsRun < m_resourceNum; threadsRun++) {

			CCycle info;
			// -2- Get next camera + reset prior 
			auto topCam = m_priorityQueue.top();
			info.camID = topCam.key;
			info.priority = topCam.priority;
			info.timeStamp = cycle;

			// -4- process frame
			int sleep = 0;

			// -3- Get free resource 
			int maxWaitToResourceSec = 2000;
			if (!m_resourceBouncer->take(maxWaitToResourceSec)) {
				std::cout << "no resource available for N elapse time\n";
				std::cout << " Error waiting too  long to free resource \n";
			}
			m_priorityQueue.reset(info.camID); // reset prior

			std::promise <CCycle>  pInfo;
			test_run6(std::ref(info), std::ref(pInfo), processDuration);
			//test_run3(info, processDuration);


			// -5- Free resource 
			m_resourceBouncer->release();

			// Update m_topPriority from the queue
			// -6- (Callback) : In case of detection - set a new higher cam priority 
			if (0)
			{
				int motion = 0;
				if ((info.motion + info.detections + info.alert + info.activeCamera) > 0) {
					info.priority = calcPriority(info.motion, info.detections, info.alert, info.activeCamera);
					m_priorityQueue.set(info.camID, info.priority);
				}
			}

			cyclesInfo.push_back(info);
		}

		int startBatchInd = cyclesInfo.size() - m_resourceNum - 1;

		for (int i= startBatchInd; i < cyclesInfo.size();i++)
		{
			auto info = cyclesInfo[i];

			int motion = 0;
			if ((info.motion + info.detections + info.alert + info.activeCamera) > 0) {
				info.priority = calcPriority(info.motion, info.detections, info.alert, info.activeCamera);
				m_priorityQueue.set(info.camID, info.priority);
			}
		}

		updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.
	}


	printStatistics(cyclesInfo, m_camerasLen);
#if 0
	std::cout << "\n\n\n=====================================================================\n\n";

	int avgCall = int((float)(cyclesInfo.size()) / (float)m_camerasLen);
	for (int cam = 0; cam < MaxCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.camID == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.camID == cam)
				sumDetectoins += cycleInf.detections;
		}
		std::cout << "Cam " << cam << " ; dections = " << sumDetectoins << " ; cycles# = " << processNum << " (" << processNum - avgCall << ")\n";
	}
	if (1)
		printCycles("c:\\tmp\\priorQueue.csv", cyclesInfo);

	printCyclesSummary("c:\\tmp\\priorQueueSumm.csv", cyclesInfo, m_camerasLen);
#endif 
	int end = 10;

}

void CLoadBalaner::test_async()
{
	std::thread camTH[MaxCameras];
	std::vector <CCycle> cyclesInfo;

	//---------
	// INIT
	//---------
	if (1) 
		std::srand(std::time(nullptr));


	// Init  resouce & cameras:
	m_resourceBouncer->set(m_resourceNum);

	int testedCameras = 20;
	for (int i = 0; i < testedCameras; i++)
		m_priorityQueue.set(i, 0);

	m_camerasLen = m_priorityQueue.size();

	if (m_priorityQueue.empty()) {
		std::cout << "No camera had been init ! Quit \n";
		return;
	}


	int cyclesCount = CYCLES_NUM / m_resourceNum;
	for (int cycle = 0; cycle < cyclesCount; cycle++) {
		
		m_priorityQueue.beat();

		std::vector <std::promise<CCycle>> promises(m_camerasLen);
		std::vector < std::future<CCycle>> futures(m_camerasLen);
		for (int i=0;i< promises.size();i++)
			futures[i] = promises[i].get_future();

		std::vector <CCycle> batchsInfo(m_resourceNum);

		for (int threadsRun = 0; threadsRun < m_resourceNum; threadsRun++)
		{
			auto info = &batchsInfo[threadsRun];
			// -1- Get next camera + reset prior 
			auto topCam = m_priorityQueue.top();
			info->camID = topCam.key;
			info->priority = topCam.priority;
			info->timeStamp = cycle;

			if (1)
			if (info->camID == 11) // simulate activeCam 
				info->activeCamera = 1;

			// -2- get working reaource 
			int maxWaitToResourceSec = 2000;
			if (!m_resourceBouncer->take(maxWaitToResourceSec))  std::cout << "no resource available for N elapse time\n";
			m_priorityQueue.reset(info->camID); // reset prior

			if (info->detections > 0)
				int debug = 10;

			// -3- process frame
			camTH[info->camID] = std::thread(test_run6, std::ref(*info), std::ref(promises[info->camID]) , processDuration);

			if (info->camID == 0 && info->detections != 0)
				int debug = 10;

			// -4- Free resource 
			m_resourceBouncer->release();
		}


		// Post process
		//--------------
		for (int threadsRun = 0; threadsRun < m_resourceNum; threadsRun++)
		{
			auto info = &batchsInfo[threadsRun];
			if (camTH[info->camID].joinable())
				camTH[info->camID].join();
			else
				int debug = 10;


			if (info->detections > 0)
				int debug = 10;

			auto threadInfo = futures[info->camID].get();
			if (info->detections != threadInfo.detections)
				int debug = 10;// get thread results 

			// Update m_topPriority from the queue
			// -6- (Callback) : In case of detection - set a new higher cam priority 
			int motion = 0;
			if ( (info->motion+ info->detections+ info->alert + info->activeCamera) > 0) {
				info->priority = calcPriority(info->motion , info->detections , info->alert , info->activeCamera);
				m_priorityQueue.set(info->camID, info->priority);
			}

			cyclesInfo.push_back(*info);

		}

		updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.

	}

	//---------------------------------------
	// analyse cycles flow 
	//---------------------------------------
	printStatistics(cyclesInfo, m_camerasLen);
#if 0
	std::cout << "\n\n\n=====================================================================\n\n";

	int avg = int((float)(cyclesInfo.size()) / (float)m_camerasLen);
	for (int cam = 0; cam < MaxCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.camID == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.camID == cam)
				sumDetectoins += cycleInf.detections;
		}
		std::cout << "Cam " << cam << " ; dections = " << sumDetectoins << " ; cycles# = " << processNum << " (" << processNum - avg << ")\n";
	}
	if (1) 
		printCycles("c:\\tmp\\priorQueue.csv", cyclesInfo);
	
	printCyclesSummary("c:\\tmp\\priorQueueSumm.csv", cyclesInfo, m_camerasLen);
#endif 
	int end = 10;

}


/*-------------------------------------------------
 -------------------------------------------------*/
void CLoadBalaner::updatePriorThresholds()
{
	m_topPriority = m_priorityQueue.top().priority;
	auto sortedHeap = m_priorityQueue.status();
	m_2thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.33)].priority; // 0 ind is highest prior
	m_thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.67)].priority;
}




void CLoadBalaner::set(int camID, int proir)
{
	std::lock_guard<std::mutex> lockGuard(qeue_mutex);
	m_priorityQueue.set(camID, proir);
}





#if 0


void CLoadBalaner::initCamera(int videoIndex)
{
	std::lock_guard<std::mutex> lockGuard(qeue_mutex);

	int startPrior = 1;
	if (!m_priorityQueue.empty())
		startPrior = m_priorityQueue.top().priority + 1;

	m_priorityQueue.set(videoIndex, startPrior);
	//m_priorityQueue.push(videoIndex, startPrior);

	m_camerasLen++;
}  
void CLoadBalaner::test_async()
{
	std::thread camTH[MaxCameras];

	std::vector <CCycle> cyclesInfo;
	std::vector <int> camsBatch;

	if (1)	std::srand(std::time(nullptr));


	// Init  resouce & cameras:
	m_resourceBouncer->set(m_resourceNum);

	int testedCameras = 20;
	for (int i = 0; i < testedCameras; i++)
		initCamera(i);

	if (m_priorityQueue.empty()) {
		std::cout << "No camera had been init ! Quit \n";
		return;
	}


	//int camID;
	int camPrior;

	int cyclesCount = CYCLES_NUM / m_resourceNum;
	for (int cycle = 0; cycle < cyclesCount; cycle++) {
		// batch init
		m_priorityQueue.beat(1);

		//-------------------
		// Batch Preprocess
		//-------------------
		std::vector <CCycle> batchsInfo;
		for (int batch = 0; batch < m_resourceNum; batch++) {
			// -2- Get next camera + reset prior sl
			auto topCam = m_priorityQueue.top();
			camPrior = topCam.priority;
			batchsInfo.push_back(CCycle(topCam.key, topCam.priority, 0, 0));
			if (topCam.key == 11) // DDEBUG set active Cam
				batchsInfo.back().activeCamera = 1;
			m_priorityQueue.reset(topCam.key);
		}

		//-------------------
		// Run Batch 
		// (batchsInfo[] update by run3 thread)
		//-------------------
		for (auto& batch : batchsInfo) {
			int sleep = 0;
			CCycle cycleInfo;
			camTH[batch.camID] = std::thread(&CLoadBalaner::test_run3_async, this, batch.camID, batch.priority, sleep, std::ref(batch));
		}

		updatePriorThresholds(); //  update topPriority, m_2thirdPriority etc.

		//----------------------------------------------------------------------------------------
		// Batch post process:
		// Set new priorities according to detections results (first wait for batch to complete)
		//----------------------------------------------------------------------------------------
		for (auto& batch : batchsInfo)
			if (camTH[batch.camID].joinable()) {
				camTH[batch.camID].join();

				// set priority according to detection status :
				{
					std::lock_guard<std::mutex> lockGuard(mutex);
					std::cout << "cam " << batch.camID << " detect " << batch.detections << " (P" << batch.priority << ",A" << batch.activeCamera << ")\n";
				}
				if (batch.detections > 0 || batch.activeCamera > 0 || batch.alert > 0) {
					int newPrior = calcPriority(batch.motion, batch.detections, batch.alert, batch.activeCamera);
					m_priorityQueue.set(batch.camID, batch.priority);
				}

				// update cyclesInfo
				//batch.detections = cyclesInfo[batch.camID].detections;
				cyclesInfo.push_back(batch);

			}

		/*
			// Update m_topPriority from the queue
			// -6- (Callback) : In case of detection - set a new higher cam priority
			int motion = 0;
			if ((motion + detections + alert + observed) > 0) {
				int newPrior = calcPriority(motion, detections, alert, observed); // maxPrior / 2;
				m_priorityQueue.set(camID, newPrior);
			}
			//else			m_priorityQueue.reset(camID);

			//if (cycle % m_camerasLen == 0)
			*/


	}
	std::cout << "\n\n\n=====================================================================\n\n";

	int avg = int((float)(cyclesCount * m_resourceNum) / (float)m_camerasLen);
	for (int cam = 0; cam < MaxCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.camID == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.camID == cam)
				sumDetectoins += cycleInf.detections;
		}
		/*
		int detectionSum = std::accumulate(cyclesInfo.begin(), cyclesInfo.end(), 0,
			[](int accumulator, const CCycle& cyc) {
				return accumulator + cyc.res;
			});
		detectionSum /=
		auto lambda = [&](CCycle a, CCycle b) {return a.res + b.res / cyclesInfo.size(); };
		int detectionsNum = std::accumulate(cyclesInfo.begin(), cyclesInfo.end(), 0.0, lambda);
		*/
		//int detectionsNum = std::count(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.res  .camID == cam; });
		std::cout << "Cam " << cam << " ; dections = " << sumDetectoins << " ; cycles# = " << processNum << " (" << processNum - avg << ")\n";
	}
	if (1)
		printCycles("c:\\tmp\\priorQueue.csv", cyclesInfo);

	printCyclesSummary("c:\\tmp\\priorQueueSumm.csv", cyclesInfo, m_camerasLen);
	int end = 10;

}


//------------------------------------------------------------------
// "process" a frame , return detection "status" (detected objects) 
//------------------------------------------------------------------
void CLoadBalaner::test_run3(CCycle& info, int duration_ms)
{

	// -3- Get free resource 
	int maxWaitToResourceSec = 2000;
	if (!m_resourceBouncer->take(maxWaitToResourceSec)) {
		std::cout << "no resource available for N elapse time\n";
		std::cout << " Error waiting too  long to free resource \n";
	}
	m_priorityQueue.reset(info.camID); // reset prior

	std::cout << "Process cam " << info.camID << " (P" << info.priority << ", A" << info.activeCamera << ")\n";

	if (duration_ms > 0)
		Sleep(duration_ms);
	// Detection results:
	if (info.camID == 11) {
		//info.activeCamera = 1;
		info.detections = 1;
		info.alert = 1;
	}
	else if (info.camID == 7) {
		//info.activeCamera = 0;
		info.detections = 0;
		info.alert = 0;
	}
	else if (info.camID % 3 == 0)
		// return 0, 1 ranodmaly 
		info.detections = rand() < int((float)RAND_MAX / RAND_RATIO) ? 1 : 0;
	else
		info.detections = 0; // rand() < int((float)RAND_MAX / (RAND_RATIO / 2)) ? 1 : 0;

	if (info.detections > 0)
		int debug = 10;
	// -5- Free resource 
	m_resourceBouncer->release();
}

#endif 

#endif 