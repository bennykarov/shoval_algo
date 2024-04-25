#include <thread> 
#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include <stdlib.h>     /* srand, rand */
#include <numeric>

#include "opencv2/opencv.hpp"

#include "logger.hpp"
#include "loadBalancer.hpp"


const int MaxCameras = 20;
const int Maxthreads = 5;


//CSemaphore m_resourceBouncer(Maxthreads);



#ifdef USE_LOAD_BALANCER


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

void CLoadBalaner::init(int reoucesLen, int camerasLen)
{
	m_timestamps.set_capacity(TIMESTAMP_LEN);

	m_reoucesLen = reoucesLen;
	m_camerasLen = camerasLen;
	m_topPriority = camerasLen;

	for (int i = 0; i < m_camerasLen; i++)
		m_priorityQueue.push(i, i); // set sequantial priority for good starter 


	//g_ResourceSemaphore.set(reoucesLen);
	/*
	m_resources.assign(reoucesLen, CResource());
	m_waitingCams.assign(camerasLen, CWaitingElement());
	*/
}

//------------------------------------------------------------------
// "process" a frame , return detection "status" (detected objects) 
//------------------------------------------------------------------
std::tuple<int, int, int> CLoadBalaner::test_run3(int camID, int printPrior , int duration_ms)
	{
	int alert = 0;
	int observed = 0;
	int detections = 0;
	std::cout << "Process cam " << camID  << " (P" << printPrior << ")";
	if (duration_ms > 0)
		Sleep(duration_ms);
	// Detection results:
	if (camID == 11) {
		observed = 1;
		detections = 1;
		alert = 1;
	}
	else 
		// return 0, 1 ranodmaly 
		detections = rand() <  int((float)RAND_MAX / 5.) ? 1 : 0;

	//detections = 0; // DDEBUG DDEBUG 

	std::cout << "  with result " << detections << "\n";
	return { detections, alert, observed } ;
}


class CCycle {
public:
	CCycle(int runCams_, int priority_, int detections_)
	{
		runCams = runCams_;
		priority = priority_;
		detections  = detections_;
	}

	int runCams=-1;
	int priority=-1;
	int detections=-1;

};

bool printCycles(std::string fname, std::vector < CCycle> cycs)
{
	std::ofstream queueFile(fname);

	if (!queueFile.is_open()) {
		std::cout << "ERROR : Cant open " << fname << " file to write \n";
		return false;
	}

	queueFile << " camNum , priority , deetction Res  \n";


	for (auto cycle : cycs)
		queueFile << cycle.runCams << "," << cycle.priority << "," << cycle.detections << ",\n";

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
	
	std::cout << "\n\n\n";
	std::cout << " camNum , detecions#,  totalCycles, avgEllapsd \n";
	std::cout << "-------------------------------------------------\n";

	// Summarize info for each cam
	for (int cam = 0; cam < activeCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.runCams == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.runCams == cam; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.runCams == cam)
				sumDetectoins += cycleInf.detections > 0 ? 1 : 0;
		}
		//int detectionsNum = std::count(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.res  .runCams == cam; });

		//--------------------------
		// Calc avg ellapsed time
		//--------------------------
		int prevFrameNum = -1;
		std::vector <int> framesNum;
		for (int i = 0; i < cyclesInfo.size();i++) {
			if (cyclesInfo[i].runCams == cam)
				framesNum.push_back(i);
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
		ellapsedTime /= framesNum.size();




		std::cout << "cam= " << cam << " ; detections = " << sumDetectoins << " ; cycles = " << processNum << " ; avg Wait = " << ellapsedTime << " (min, max: " << min_ellapsedTime << "," << max_ellapsedTime <<")\n";


		queueFile << cam << "," << sumDetectoins << "," << processNum << ","  << ellapsedTime << "," << min_ellapsedTime << "," << max_ellapsedTime << ",\n";

	}


	queueFile.close();
	return true;
}




std::tuple <int,int> CLoadBalaner::getNextCam()
{
	// -2- Get next camera + reset prior + beat()
		auto topCam = m_priorityQueue.top();
		int camID = topCam.key;
		int camPrior = topCam.priority;
		m_priorityQueue.reset(camID); // reset 
		//???m_priorityQueue.beat();


		return std::tuple<int, int>(camID, camPrior);

}

/*------------------------------------------------------------------------------------------
* Set priority according to camera online parameters 
 ------------------------------------------------------------------------------------------*/
int CLoadBalaner::setPriority(int motion, int detections, int alert, int observed)
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
		queuePriority = m_thirdPriority;
		break;
	case 2:
		queuePriority = m_2thirdPriority;
		break;
		// Observation camera
	case 3:
		queuePriority = m_priorityQueue.top().priority - 3;
		break;
	case 4:
		queuePriority = m_priorityQueue.top().priority - 2;
		break;
	case 5:
		queuePriority = m_priorityQueue.top().priority - 1;
		break;
	}


	return queuePriority;

}


void CLoadBalaner::test(int reourceNum, int camerasLen)
{

	std::srand(std::time(nullptr));

	std::vector <CCycle> cyclesInfo;

	std::thread th[MaxCameras];
	std::thread th_single;

	m_topPriority = m_camerasLen;
	m_resourceNum = reourceNum;


	// Init  resouce & cameras:

	m_resourceBouncer->set(m_resourceNum);

	// -1- init (max) priorities - new cam must process as fast as possible 
	//for (int i = 0; i < MaxCameras; i++)    m_priorityQueue.set(i, m_topPriority);

	int camID;
	int camPrior;

	int cyclesCount = 1000;
	for (int cycle = 0; cycle < cyclesCount; cycle++) {
		
		m_priorityQueue.beat();

		for (int threadsRun = 0; threadsRun < m_resourceNum; threadsRun++) {

			// -2- Get next camera + reset prior + beat()
			auto topCam = m_priorityQueue.top();
			camID = topCam.key;
			camPrior = topCam.priority;
			m_priorityQueue.reset(camID); // reset 

			// -3- Get free resource 
			int maxWaitToResourceSec = 2000;
			if (!m_resourceBouncer->take(maxWaitToResourceSec)) {
				std::cout << "no resource available for N elapse time\n";
				std::cout << " Error waiting too  long to free resource \n";
			}


			// -4- process frame
			int sleep = 0;
			auto [detections, alert, observed] = test_run3(camID, camPrior, sleep);
			// -5- Free resource 
			m_resourceBouncer->release();

			cyclesInfo.push_back(CCycle(camID, camPrior, detections));

			// Update m_topPriority from the queue
			// -6- (Callback) : In case of detection - set a new higher cam priority 
			int motion = 0;
			if ( (motion+ detections+  alert + observed) > 0) {
				int newPrior = setPriority(motion, detections, alert, observed); // maxPrior / 2;
				m_priorityQueue.set(camID, newPrior);
			}
			//else			m_priorityQueue.reset(camID);

			//if (cycle % m_camerasLen == 0) 
			updateThresholds(); //  update topPriority, m_2thirdPriority etc.
			/*
				m_topPriority = m_priorityQueue.top().priority;
				auto sortedHeap = m_priorityQueue.status();
				m_2thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.67)].priority;
				m_thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.33)].priority;
			*/


		}
	}
	std::cout << "\n\n\n=====================================================================\n\n";

	int avg = int((float)(cyclesCount * m_resourceNum) / (float)m_camerasLen);
	for (int cam = 0; cam < MaxCameras; cam++) {
		auto lambda = [cam](CCycle c) { return c.runCams == cam; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		//int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.runCams == cam; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.runCams == cam)
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
		//int detectionsNum = std::count(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.res  .runCams == cam; });
		std::cout << "Cam " << cam << " ; dections = " << sumDetectoins << " ; cycles# = " << processNum << " (" << processNum - avg << ")\n";
	}
	if (1)
		printCycles("c:\\tmp\\priorQueue.csv", cyclesInfo);
	
	printCyclesSummary("c:\\tmp\\priorQueueSumm.csv", cyclesInfo, m_camerasLen);
	int end = 10;

}


void CLoadBalaner::updateThresholds()
{
	m_topPriority = m_priorityQueue.top().priority;
	auto sortedHeap = m_priorityQueue.status();
	m_2thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.67)].priority;
	m_thirdPriority = sortedHeap[int((float)sortedHeap.size() * 0.33)].priority;
}



#endif 